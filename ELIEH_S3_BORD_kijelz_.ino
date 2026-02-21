// =====================================================
// ELIEH_S3_DISPLAY_BLE_UI_V1.ino
// ESP32-S3 + ILI9341 (Adafruit_ILI9341) + XPT2046 Touch (külön SPI: FSPI)
// BLE Central -> ELIEH_A1S (Nordic UART service)
//
// UI:
//  [CONNECT]   [DISCONNECT]
//  [A1S: ...]  [A1S BTN]
//
// Touch csak akkor indul, ha a kijelzés már felállt és stabil.
// =====================================================

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <XPT2046_Touchscreen.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEClient.h>

// ---------------- TFT PINS (a te bevált kiosztásod) ----------------
#define TFT_DC   47
#define TFT_CS   14
#define TFT_MOSI 45
#define TFT_CLK  3
#define TFT_RST  21
#define TFT_MISO 43

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST, TFT_MISO);

// ---------------- TOUCH PINS (XPT2046) ----------------
#define TOUCH_CS   1
#define TOUCH_IRQ  255   // nincs bekötve
#define TOUCH_MOSI 2
#define TOUCH_MISO 41
#define TOUCH_CLK  42    // a fotód szerint 42 !

SPIClass touchSPI(FSPI);
XPT2046_Touchscreen ts(TOUCH_CS, TOUCH_IRQ);

// Touch késleltetett indítás
static bool touchEnabled = false;
static uint32_t touchEnableAt = 0;

// Touch kalibráció (RAW -> kijelző koordináta)
// Ha kell, később finomítjuk a te panelodra:
#define TS_MINX 200
#define TS_MAXX 3800
#define TS_MINY 200
#define TS_MAXY 3800

// ---------------- PROTOKOLL TAG-ek ----------------
static const char* STT_START    = "<STT>";
static const char* STT_END      = "</STT>";
static const char* HELLO_START  = "<HELLO>";
static const char* HELLO_END    = "</HELLO>";
static const char* PING_START   = "<PING>";
static const char* PING_END     = "</PING>";
static const char* PONG_START   = "<PONG>";
static const char* PONG_END     = "</PONG>";
static const char* READY_START  = "<READY>";
static const char* READY_END    = "</READY>";

static const char* UI_START     = "<UI>";
static const char* UI_END       = "</UI>";
static const char* BTN_START    = "<BTN>";
static const char* BTN_END      = "</BTN>";
static const char* STATE_START  = "<STATE>";
static const char* STATE_END    = "</STATE>";
static const char* INTENT_START = "<INTENT>";
static const char* INTENT_END   = "</INTENT>";

// ---------------- BLE (Nordic UART) ----------------
static const char* TARGET_NAME = "ELIEH_A1S";
static BLEUUID SERVICE_UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
static BLEUUID RX_UUID     ("6E400002-B5A3-F393-E0A9-E50E24DCCA9E"); // S3 ír ide (A1S RX)
static BLEUUID TX_UUID     ("6E400003-B5A3-F393-E0A9-E50E24DCCA9E"); // S3 innen notify (A1S TX)

// ---------------- ÁLLAPOTOK ----------------
static BLEAdvertisedDevice* foundDevice = nullptr;
static BLEClient* client = nullptr;
static BLERemoteCharacteristic* chrRx = nullptr;
static BLERemoteCharacteristic* chrTx = nullptr;

static bool connected = false;
static bool a1sReady = false;
static String a1sHello = "";
static uint32_t lastScanMs = 0;

// bejövő buffer
static String inBuf;

// A1S tükör állapot
static String curMode = "LEARN";
static int curIdx = 0;
static int curArm = 0;
static String curTxt = "";

// UI státusz sorok
static String lineBLE   = "BLE: DISCONNECTED";
static String lineReady = "A1S READY: NO";
static String lineMode  = "MODE: LEARN IDX:0";
static String lineTXT   = "TXT:";

// UI “dirty” frissítés
static bool dirtyStatus = true;
static bool dirtyTXT    = true;
static bool dirtyMode   = true;
static bool dirtyReady  = true;
static uint32_t lastUiTick = 0;

// gomb “kérések”
static bool requestConnect = false;
static bool requestDisconnect = false;
static bool requestBtnQuery = false;

// ---------------- Frame segédek ----------------
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

static String makeFrame(const char* startTag, const char* endTag, const String& payload) {
  String f;
  f.reserve(strlen(startTag) + payload.length() + strlen(endTag) + 2);
  f += startTag;
  f += payload;
  f += endTag;
  f += "\n";
  return f;
}

static void bleWriteFramed(const char* startTag, const char* endTag, const String& payload) {
  if (!connected || !chrRx) return;
  String msg = makeFrame(startTag, endTag, payload);
  chrRx->writeValue((uint8_t*)msg.c_str(), msg.length(), false);
}

// ---------------- Notify callback ----------------
static void notifyCallback(BLERemoteCharacteristic*, uint8_t* data, size_t length, bool) {
  for (size_t i = 0; i < length; i++) inBuf += (char)data[i];
  if (inBuf.length() > 8192) inBuf.remove(0, 4096);
}

// ---------------- Scan callback ----------------
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) override {
    String name = advertisedDevice.getName().c_str();
    bool nameOk = (name.length() && name == TARGET_NAME);
    bool svcOk  = advertisedDevice.haveServiceUUID() &&
                  advertisedDevice.isAdvertisingService(SERVICE_UUID);

    if (nameOk || svcOk) {
      if (foundDevice) delete foundDevice;
      foundDevice = new BLEAdvertisedDevice(advertisedDevice);
      BLEDevice::getScan()->stop();
    }
  }
};

// ---------------- BLE connect/disconnect ----------------
static void setBleStatus(const String& s) { lineBLE = "BLE: " + s; dirtyStatus = true; }
static void setReadyStatus(bool ok) { lineReady = String("A1S READY: ") + (ok ? "YES" : "NO"); dirtyReady = true; }

static void resetBleState() {
  connected = false;
  a1sReady = false;
  a1sHello = "";
  chrRx = nullptr;
  chrTx = nullptr;
  inBuf = "";
  setBleStatus("DISCONNECTED");
  setReadyStatus(false);
}

static bool connectToA1S() {
  if (!foundDevice) return false;

  setBleStatus("CONNECTING");
  dirtyStatus = true;

  client = BLEDevice::createClient();
  if (!client->connect(foundDevice)) {
    resetBleState();
    return false;
  }

  BLERemoteService* service = client->getService(SERVICE_UUID);
  if (!service) {
    client->disconnect();
    resetBleState();
    return false;
  }

  chrRx = service->getCharacteristic(RX_UUID);
  chrTx = service->getCharacteristic(TX_UUID);
  if (!chrRx || !chrTx) {
    client->disconnect();
    resetBleState();
    return false;
  }

  if (!chrTx->canNotify()) {
    client->disconnect();
    resetBleState();
    return false;
  }
  chrTx->registerForNotify(notifyCallback);

  connected = true;
  a1sReady = false;
  a1sHello = "";
  inBuf = "";

  setBleStatus("CONNECTED");
  setReadyStatus(false);

  // handshake
  bleWriteFramed(HELLO_START, HELLO_END, "ELIEH_S3_DISPLAY_V1");
  bleWriteFramed(PING_START,  PING_END,  "?");
  bleWriteFramed(READY_START, READY_END, "S3_OK");
  bleWriteFramed(UI_START, UI_END, "STATE?");

  return true;
}

static void disconnectA1S() {
  if (client && client->isConnected()) client->disconnect();
  resetBleState();
}

// Scan + connect folyamat (kíméletes)
static void ensureConnectLoop() {
  if (connected && client && client->isConnected()) return;

  // ha leesett
  if (connected && (!client || !client->isConnected())) resetBleState();

  // csak akkor scan-elünk, ha kértünk connectet
  if (!requestConnect) return;

  if (millis() - lastScanMs < 1200) return;
  lastScanMs = millis();

  BLEScan* scan = BLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks(), true);
  scan->setActiveScan(true);
  scan->start(1, false);

  if (foundDevice) {
    if (connectToA1S()) requestConnect = false;
  }
}

// ---------------- A1S STATE parse ----------------
static void parseState(const String& payload) {
  int p1 = payload.indexOf('|');
  int p2 = payload.indexOf('|', p1+1);
  int p3 = payload.indexOf('|', p2+1);
  if (p1 < 0 || p2 < 0 || p3 < 0) return;

  curMode = payload.substring(0, p1);
  curIdx  = payload.substring(p1+1, p2).toInt();
  curArm  = payload.substring(p2+1, p3).toInt();
  curTxt  = payload.substring(p3+1);
  curMode.trim(); curTxt.trim();

  lineMode = "MODE: " + curMode + " IDX:" + String(curIdx);
  dirtyMode = true;

  // a TXT sor lehet a slot szöveg is
  if (curTxt.length()) {
    lineTXT = "TXT: " + curTxt;
    dirtyTXT = true;
  }
}

// =====================================================
// UI rajzolás (gyors, “dirty” frissítés)
// =====================================================

static void drawButton(int x, int y, int w, int h, const char* label) {
  tft.drawRoundRect(x, y, w, h, 6, ILI9341_WHITE);
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE);
  int16_t bx, by; uint16_t bw, bh;
  tft.getTextBounds(label, 0, 0, &bx, &by, &bw, &bh);
  int tx = x + (w - (int)bw)/2;
  int ty = y + (h - (int)bh)/2;
  tft.setCursor(tx, ty);
  tft.print(label);
}

static void uiDrawStatic() {
  tft.fillScreen(ILI9341_BLACK);
  tft.setRotation(1);

  // header
  tft.fillRect(0, 0, 320, 26, ILI9341_CYAN);
  tft.setTextColor(ILI9341_BLACK);
  tft.setTextSize(2);
  tft.setCursor(8, 5);
  tft.print("ELIEH S3");

  // gombok
  // (x,y,w,h)
  drawButton(18, 40, 140, 48, "CONNECT");
  drawButton(162, 40, 140, 48, "DISCONNECT");

  // alsó sor: A1S státusz + A1S BTN
  drawButton(18, 96, 140, 40, "A1S: CONNECT");
  drawButton(162, 96, 140, 40, "A1S BTN");

  // státusz doboz
  tft.drawRect(10, 142, 300, 88, ILI9341_WHITE);

  dirtyStatus = dirtyReady = dirtyMode = dirtyTXT = true;
}

static void uiUpdateDynamic() {
  // frissítési limit (gyorsabb, de nem villog)
  if (millis() - lastUiTick < 40) return;
  lastUiTick = millis();

  // státusz terület törlés + újraír (csak soronként)
  // Y kiosztás:
  // 148: BLE...
  // 166: READY...
  // 184: MODE...
  // 202: TXT...

  tft.setTextSize(2);

  if (dirtyStatus) {
    tft.fillRect(14, 146, 292, 18, ILI9341_BLACK);
    tft.setTextColor(ILI9341_ORANGE);
    tft.setCursor(16, 148);
    tft.print(lineBLE);
    dirtyStatus = false;
  }

  if (dirtyReady) {
    tft.fillRect(14, 164, 292, 18, ILI9341_BLACK);
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(16, 166);
    tft.print(lineReady);
    dirtyReady = false;
  }

  if (dirtyMode) {
    tft.fillRect(14, 182, 292, 18, ILI9341_BLACK);
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(16, 184);
    tft.print(lineMode);
    dirtyMode = false;
  }

  if (dirtyTXT) {
    tft.fillRect(14, 200, 292, 26, ILI9341_BLACK);
    tft.setTextColor(ILI9341_CYAN);
    tft.setCursor(16, 202);
    tft.print(lineTXT);
    dirtyTXT = false;
  }

  // A1S státusz gomb felirat frissítés (egyszerű: újrarajzoljuk a gombot)
  // (ritkán változik)
  static String lastA1SBtn = "";
  String label = "A1S: CONNECT";
  if (connected) label = a1sReady ? "A1S: ONLINE" : "A1S: WAIT";
  if (label != lastA1SBtn) {
    // gomb terület letöröl + újra
    tft.fillRect(18, 96, 140, 40, ILI9341_BLACK);
    drawButton(18, 96, 140, 40, label.c_str());
    lastA1SBtn = label;
  }
}

// =====================================================
// Touch init + touch -> gombnyomás
// =====================================================

static void touchInitNow() {
  touchSPI.begin(TOUCH_CLK, TOUCH_MISO, TOUCH_MOSI, TOUCH_CS);
  ts.begin(touchSPI);
  ts.setRotation(1);     // egyezzen a tft rotation-nel
  touchEnabled = true;
}

static bool mapTouchToScreen(int16_t &sx, int16_t &sy) {
  if (!touchEnabled) return false;
  if (!ts.touched()) return false;

  TS_Point p = ts.getPoint();

  // RAW -> screen (0..319, 0..239)
  sx = map(p.x, TS_MINX, TS_MAXX, 0, 319);
  sy = map(p.y, TS_MINY, TS_MAXY, 0, 239);

  // clamp
  if (sx < 0) sx = 0; if (sx > 319) sx = 319;
  if (sy < 0) sy = 0; if (sy > 239) sy = 239;
  return true;
}

static bool inRect(int x, int y, int w, int h, int px, int py) {
  return (px >= x && px < x+w && py >= y && py < y+h);
}

static void handleTouchUI() {
  int16_t x, y;
  if (!mapTouchToScreen(x, y)) return;

  // egyszerű debounce
  static uint32_t lastTouchMs = 0;
  if (millis() - lastTouchMs < 180) return;
  lastTouchMs = millis();

  // CONNECT gomb
  if (inRect(18, 40, 140, 48, x, y)) {
    requestConnect = true;
    setBleStatus("CONNECTING");
    lineTXT = "TXT: connect pressed";
    dirtyTXT = true;
    return;
  }

  // DISCONNECT gomb
  if (inRect(162, 40, 140, 48, x, y)) {
    requestDisconnect = true;
    lineTXT = "TXT: disconnect pressed";
    dirtyTXT = true;
    return;
  }

  // A1S: CONNECT/ONLINE/WAIT gomb (alsó bal)
  if (inRect(18, 96, 140, 40, x, y)) {
    if (!connected) {
      requestConnect = true;
      setBleStatus("CONNECTING");
      lineTXT = "TXT: A1S connect";
      dirtyTXT = true;
    } else {
      // ha már connected, kérjünk state-et
      bleWriteFramed(UI_START, UI_END, "STATE?");
      lineTXT = "TXT: STATE?";
      dirtyTXT = true;
    }
    return;
  }

  // A1S BTN gomb (alsó jobb)
  if (inRect(162, 96, 140, 40, x, y)) {
    requestBtnQuery = true;
    lineTXT = "TXT: BTN?";
    dirtyTXT = true;
    return;
  }
}

// =====================================================
// SETUP / LOOP
// =====================================================

void setup() {
  Serial.begin(115200);
  delay(150);

  // TFT init
  tft.begin();
  tft.setRotation(1);

  uiDrawStatic();
  uiUpdateDynamic();

  // Touch csak később!
  touchEnabled = false;
  touchEnableAt = millis() + 700;   // stabil kijelzés után

  // BLE init
  BLEDevice::init("ELIEH_S3");

  resetBleState();
  lineTXT = "TXT: ready";
  dirtyTXT = true;
}

void loop() {
  // Touch késleltetett init (stabil képernyő után)
  if (!touchEnabled && (int32_t)(millis() - touchEnableAt) >= 0) {
    touchInitNow();
    lineTXT = "TXT: touch enabled";
    dirtyTXT = true;
  }

  // UI frissítés
  uiUpdateDynamic();

  // Touch kezelése
  handleTouchUI();

  // BLE connect/disconnect kérések
  if (requestDisconnect) {
    requestDisconnect = false;
    disconnectA1S();
  }

  // csak akkor próbálkozzon, ha requestConnect = true
  ensureConnectLoop();

  // ha csatlakozva, kezeljük a bejövő frame-eket
  if (connected && client && client->isConnected()) {
    String tmp;

    // HELLO
    if (!a1sHello.length() && readFrameFromBuf(HELLO_START, HELLO_END, tmp)) {
      a1sHello = tmp;
      lineTXT = "TXT: A1S HELLO ok";
      dirtyTXT = true;
    }

    // PONG
    while (readFrameFromBuf(PONG_START, PONG_END, tmp)) {
      // opcionális
    }

    // READY
    if (!a1sReady && readFrameFromBuf(READY_START, READY_END, tmp)) {
      a1sReady = true;
      setReadyStatus(true);
      // kérjünk state-et rögtön
      bleWriteFramed(UI_START, UI_END, "STATE?");
    }

    // STATE
    while (readFrameFromBuf(STATE_START, STATE_END, tmp)) {
      parseState(tmp);
    }

    // STT
    String stt;
    while (readFrameFromBuf(STT_START, STT_END, stt)) {
      lineTXT = "TXT: STT " + stt;
      dirtyTXT = true;
    }

    // INTENT
    while (readFrameFromBuf(INTENT_START, INTENT_END, tmp)) {
      lineTXT = "TXT: INTENT " + tmp;
      dirtyTXT = true;
    }

    // BTN válasz (A1S küldheti <BTN>...</BTN> formában)
    while (readFrameFromBuf(BTN_START, BTN_END, tmp)) {
      lineTXT = "TXT: A1S BTN " + tmp;
      dirtyTXT = true;
    }

    // A1S BTN lekérdezés gomb
    if (requestBtnQuery) {
      requestBtnQuery = false;
      bleWriteFramed(UI_START, UI_END, "BTN?");
    }

  } else {
    // ha leesett
    if (connected && (!client || !client->isConnected())) {
      resetBleState();
      lineTXT = "TXT: link lost";
      dirtyTXT = true;
    }
  }

  delay(2);
}