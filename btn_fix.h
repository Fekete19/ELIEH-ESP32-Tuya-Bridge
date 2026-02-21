#pragma once
#include <Arduino.h>

// BTNFXH - egyszerű, stabil debouncer + longpress
// activeLow = true esetén: gomb lenyomva => LOW

struct Btn {
  int pin;
  bool activeLow;
  bool stableState;        // stabil logikai állapot (pressed = true)
  bool lastRaw;            // utolsó nyers olvasás (pressed = true)
  uint32_t lastChangeMs;   // nyers változás ideje

  bool pressEvent;         // egyszeri esemény (short press)
  bool longEvent;          // egyszeri esemény (long press)

  bool longFired;          // long már elsütve?
  uint32_t pressStartMs;   // lenyomás kezdete
};

static inline bool _btnRawPressed(const Btn& b) {
  int v = digitalRead(b.pin);
  if (b.activeLow) return (v == LOW);
  return (v == HIGH);
}

static inline void btnInit(Btn& b) {
  // Megjegyzés: GPIO36 (A1S) input-only, belső pullup nincs.
  // Itt csak INPUT-ot állítunk, a hardver pullup/pulldown a te kötésed.
  pinMode(b.pin, INPUT);
  bool raw = _btnRawPressed(b);

  b.activeLow = b.activeLow;
  b.stableState = raw;
  b.lastRaw = raw;
  b.lastChangeMs = millis();

  b.pressEvent = false;
  b.longEvent = false;
  b.longFired = false;
  b.pressStartMs = 0;
}

static inline void btnUpdate(Btn& b, uint32_t debounceMs, uint32_t longPressMs) {
  b.pressEvent = false;
  b.longEvent  = false;

  bool raw = _btnRawPressed(b);
  uint32_t now = millis();

  if (raw != b.lastRaw) {
    b.lastRaw = raw;
    b.lastChangeMs = now;
  }

  // debounce: ha ennyi ideje nem változott a nyers jel, elfogadjuk stabilnak
  if ((now - b.lastChangeMs) >= debounceMs) {
    if (b.stableState != raw) {
      b.stableState = raw;

      if (b.stableState) { // lenyomás kezdete
        b.pressStartMs = now;
        b.longFired = false;
      } else { // felengedés -> short press, ha nem volt long
        if (!b.longFired) b.pressEvent = true;
      }
    }
  }

  // longpress: csak akkor, ha stabilan nyomva van
  if (b.stableState && !b.longFired) {
    if ((now - b.pressStartMs) >= longPressMs) {
      b.longFired = true;
      b.longEvent = true;
    }
  }
}

static inline bool btnPressed(Btn& b)     { return b.pressEvent; }
static inline bool btnLongPressed(Btn& b) { return b.longEvent; }