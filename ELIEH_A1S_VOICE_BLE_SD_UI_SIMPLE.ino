// =====================================================
// ELIEH_A1S_BLE_UART_V1.ino
// ESP32-A1S: BLE UART (NUS) szerver/peripheral
// - RX (Write): 6E400002...
// - TX (Notify): 6E400003...
// Kompatibilis az S3 klienseddel (ELIEH_S3_BLE...)
// =====================================================

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ===== PROTOKOLL TAG-ek =====
static const char* STT_START   = "<STT>";
static const char* STT_END     = "</STT>";
static const char* SAY_START   = "<SAY>";
static const char* SAY_END     = "</SAY>";

static const char* HELLO_START = "<HELLO>";
static const char* HELLO_END   = "</HELLO>";
static const char* PING_START  = "<PING>";
static const char* PING_END    = "</PING>";
static const char* PONG_START  = "<PONG>";
static const char* PONG_END    = "</PONG>";
static const char* READY_START = "<READY>";
static const char* READY_END   = "</READY>";

static const char* UI_START    = "<UI>";
static const char* UI_END      = "</UI>";
static const char* BTN_START   = "<BTN>";
static const char* BTN_END     = "</BTN>";
static const char* STATE_START = "<STATE>";
static const char* STATE_END   = "</STATE>";
static const char* INTENT_START= "<INTENT>";
static const char* INTENT_END  = "</INTENT>";

// ===== NUS UUID-k =====
static BLEUUID SERVICE_UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
static BLEUUID RX_UUID     ("6E400002-B5A3-F393-E0A9-E50E24DCCA9E"); // S3->A1S write ide
static BLEUUID TX_UUID     ("6E400003-B5A3-F393-E0A9-E50E24DCCA9E"); // A1S->S3 notify innen

// ===== ÁLLAPOT / UI =====
enum Mode { MODE_LEARN = 0, MODE_EXEC = 1 };
static Mode mode = MODE_LEARN;

static const int MAX_SLOTS = 20;
static String slotText[MAX_SLOTS];
static int menuIndex = 0;
static bool armTeachNextSTT = false;

static int lastButtonCode = -1;

// ===== BLE állapot =====
static BLEServer* server = nullptr;
static BLECharacteristic* chrRx = nullptr;
static BLECharacteristic* chrTx = nullptr;

static bool deviceConnected = false;
static bool s3ReadySeen = false;
static String s3Hello = "";

// ===== Bejövő buffer (RX write összefűzése) =====
static String inBuf;

// ===== Segédek =====
static String makeFrame(const char* startTag, const char* endTag, const String& payload) {
  String f;
  f.reserve(strlen(startTag) + payload.length() + strlen(endTag) + 2);
  f += startTag;
  f += payload;
  f += endTag;
  f += "\n";
  return f;
}

static void bleNotifyFramed(const char* startTag, const char* endTag, const String& payload) {
  if (!deviceConnected || !chrTx) return;
  String msg = makeFrame(startTag, endTag, payload);
  chrTx->setValue((uint8_t*)msg.c_str(), msg.length());
  chrTx->notify();
}

static bool readFrameFromBuf(const char* startTag, const char* endTag, String& out) {
  int s = inBuf.indexOf(startTag);
  if (s >= 0) {
    int e = inBuf.indexOf(endTag, s + (int)strlen(startTag));
    if (e >= 0) {
      out = inBuf.substring(s + (int)strlen(startTag), e);
      out.trim();
      inBuf.remove(0, e + (int)strlen(endTag));
      return true;
    }
  }
  return false;
}

static String buildStatePayload() {
  // MODE|IDX|ARM|TXT
  String p;
  p.reserve(64 + slotText[menuIndex].length());
  p += (mode == MODE_LEARN ? "LEARN" : "EXEC");
  p += "|";
  p += String(menuIndex);
  p += "|";
  p += String(armTeachNextSTT ? 1 : 0);
  p += "|";
  p += slotText[menuIndex];
  return p;
}

static void sendState() {
  bleNotifyFramed(STATE_START, STATE_END, buildStatePayload());
}

static void setIdx(int idx) {
  if (idx < 0) idx = MAX_SLOTS - 1;
  if (idx >= MAX_SLOTS) idx = 0;
  menuIndex = idx;
  sendState();
}

static void setMode(Mode m) {
  mode = m;
  sendState();
}

// ===== BLE callbackok =====
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    deviceConnected = true;
    s3ReadySeen = false;
    s3Hello = "";
    inBuf = "";
    Serial.println("[A1S] ✅ S3 connected");
    // Köszönünk azonnal (S3 oldal várja)
    bleNotifyFramed(HELLO_START, HELLO_END, "ELIEH_A1S_BLE_UART_V1");
  }

  void onDisconnect(BLEServer*) override {
    deviceConnected = false;
    s3ReadySeen = false;
    s3Hello = "";
    Serial.println("[A1S] ❌ S3 disconnected, advertising again...");
    BLEDevice::startAdvertising();
  }
};

class MyRxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    std::string v = c->getValue();
    if (v.empty()) return;

    for (char ch : v) inBuf += ch;
    if (inBuf.length() > 8192) inBuf.remove(0, 4096);
  }
};

// ===== Parancs feldolgozás =====
static void processFrames() {
  String tmp;

  // HELLO
  if (!s3Hello.length() && readFrameFromBuf(HELLO_START, HELLO_END, tmp)) {
    s3Hello = tmp;
    Serial.print("[A1S] S3 HELLO: "); Serial.println(s3Hello);
    // vissza köszönés már connectkor ment, de küldhetünk megerősítést is:
    bleNotifyFramed(HELLO_START, HELLO_END, "A1S_HELLO_ok");
  }

  // PING -> PONG
  while (readFrameFromBuf(PING_START, PING_END, tmp)) {
    Serial.println("[A1S] PING");
    bleNotifyFramed(PONG_START, PONG_END, "!");
  }

  // READY (S3 -> A1S) -> A1S READY válasz
  while (readFrameFromBuf(READY_START, READY_END, tmp)) {
    Serial.print("[A1S] READY from S3: "); Serial.println(tmp);
    s3ReadySeen = true;
    bleNotifyFramed(READY_START, READY_END, "A1S_OK");
    sendState(); // rögtön küldünk állapotot is
  }

  // UI parancsok
  while (readFrameFromBuf(UI_START, UI_END, tmp)) {
    Serial.print("[A1S] UI: "); Serial.println(tmp);

    if (tmp == "STATE?") {
      sendState();
      continue;
    }

    if (tmp.startsWith("IDX=")) {
      int idx = tmp.substring(4).toInt();
      setIdx(idx);
      continue;
    }

    if (tmp == "BTN?") {
      bleNotifyFramed(BTN_START, BTN_END, String(lastButtonCode));
      continue;
    }
  }

  // BTN parancsok (S3 gombok)
  while (readFrameFromBuf(BTN_START, BTN_END, tmp)) {
    Serial.print("[A1S] BTN: "); Serial.println(tmp);

    if (tmp == "OK") {
      // példa INTENT
      bleNotifyFramed(INTENT_START, INTENT_END, "OK@" + String(menuIndex));
      return;
    }

    if (tmp == "DEL") {
      slotText[menuIndex] = "";
      bleNotifyFramed(INTENT_START, INTENT_END, "DEL@" + String(menuIndex));
      sendState();
      return;
    }

    if (tmp == "MODE_LEARN") {
      setMode(MODE_LEARN);
      bleNotifyFramed(INTENT_START, INTENT_END, "MODE=LEARN");
      return;
    }

    if (tmp == "MODE_EXEC") {
      setMode(MODE_EXEC);
      bleNotifyFramed(INTENT_START, INTENT_END, "MODE=EXEC");
      return;
    }
  }

  // SAY (S3 -> A1S)
  while (readFrameFromBuf(SAY_START, SAY_END, tmp)) {
    Serial.print("[A1S] SAY: "); Serial.println(tmp);
    // Itt később mehet TTS lejátszás az A1S AudioKiton.
  }
}

// ===== “A1S BTN” teszt sorosról =====
// Írj a Serial monitorba egy számot és Enter:
// pl: 7 -> elküldi <BTN>7</BTN> notify-val az S3-nak
static void serialButtonTestPoll() {
  static String line;
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      line.trim();
      if (line.length()) {
        int code = line.toInt();
        lastButtonCode = code;
        Serial.print("[A1S] Serial BTN code = "); Serial.println(code);
        bleNotifyFramed(BTN_START, BTN_END, String(code));     // S3 “A1S BTN” gombja ezt fogja látni
        bleNotifyFramed(INTENT_START, INTENT_END, "BTN=" + String(code)); // extra info
      }
      line = "";
    } else {
      if (line.length() < 32) line += ch;
    }
  }
}

// ===== Periodikus életjel =====
static uint32_t lastHeartbeatMs = 0;
static void heartbeat() {
  if (!deviceConnected) return;
  uint32_t now = millis();
  if (now - lastHeartbeatMs < 5000) return;
  lastHeartbeatMs = now;

  // ha már láttunk S3 READY-t, akkor időnként küldünk állapotot is
  bleNotifyFramed(PONG_START, PONG_END, "hb");
  if (s3ReadySeen) sendState();
}

void setup() {
  Serial.begin(115200);
  delay(300);

  // alap slotok (hogy legyen mit látni)
  for (int i = 0; i < MAX_SLOTS; i++) slotText[i] = "SLOT " + String(i);

  Serial.println("\n[A1S] ELIEH_A1S_BLE_UART_V1 starting...");

  BLEDevice::init("ELIEH_A1S");
  server = BLEDevice::createServer();
  server->setCallbacks(new MyServerCallbacks());

  BLEService* service = server->createService(SERVICE_UUID);

  chrTx = service->createCharacteristic(
    TX_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  chrTx->addDescriptor(new BLE2902());

  chrRx = service->createCharacteristic(
    RX_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  chrRx->setCallbacks(new MyRxCallbacks());

  service->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  adv->start();

  Serial.println("[A1S] Advertising started");
}

void loop() {
  if (deviceConnected) {
    processFrames();
    serialButtonTestPoll();
    heartbeat();
  }
  delay(2);
}