// =====================================================================
//  Sterownik hamulca kołowrotka – firmware
//  Płytka:     Seeed Studio XIAO ESP32-S3 (rdzeń Arduino-ESP32 3.x)
//  Biblioteka: NimBLE-Arduino 2.x (autor: h2zero)
//
//  Połączenia (zgodnie z planem płytki):
//    D0  – środek dzielnika napięcia baterii 2 x 100 kΩ (+100 nF do GND)
//    D1  – HX711 DT          D3 – HX711 SCK
//    D4  – sterownik silnika IN1 (PWM)   D5 – sterownik silnika IN2 (PWM)
//    D8  – enkoder kanał A   D9 – enkoder kanał B
//  Sterownik silnika: dwuwejściowy mostek H (np. DRV8833, MX1508).
//  HX711: wejście RATE podłączone do VCC (80 próbek/s).
//
//  Konwencja znaków: duty > 0 = dokręcanie hamulca, duty < 0 = luzowanie.
//  Pozycja enkodera rośnie przy dokręcaniu (w razie potrzeby komendy MINV/EINV).
//
//  Komendy można wysyłać przez BLE albo przez Serial Monitor (115200, Enter).
// =====================================================================

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include "soc/gpio_reg.h"

#define FW_VERSION "1.0"

// ----------------------------- PINY ----------------------------------
static const uint8_t PIN_BAT    = D0;
static const uint8_t PIN_HX_DT  = D1;
static const uint8_t PIN_HX_SCK = D3;
static const uint8_t PIN_M_IN1  = D4;
static const uint8_t PIN_M_IN2  = D5;
static const uint8_t PIN_ENC_A  = D8;
static const uint8_t PIN_ENC_B  = D9;
static_assert(D8 < 32 && D9 < 32, "Enkoder musi byc na GPIO 0-31 (odczyt z GPIO_IN_REG)");

// ----------------------------- BLE UUID ------------------------------
#define UUID_SVC "6a4e0001-6c2b-4f5e-9a3d-1f0c2b7e9a10"
#define UUID_TEL "6a4e0002-6c2b-4f5e-9a3d-1f0c2b7e9a10"   // telemetria: READ | NOTIFY, 20 B
#define UUID_CMD "6a4e0003-6c2b-4f5e-9a3d-1f0c2b7e9a10"   // komendy:    WRITE | WRITE_NR, tekst
#define UUID_CFG "6a4e0004-6c2b-4f5e-9a3d-1f0c2b7e9a10"   // ustawienia: READ, JSON
#define UUID_LOG "6a4e0005-6c2b-4f5e-9a3d-1f0c2b7e9a10"   // zdarzenia:  NOTIFY, 12 B

// ----------------------------- KODY ZDARZEŃ --------------------------
// 1–39 informacje, 40–79 ostrzeżenia, 80–119 błędy (opisy w aplikacji)
enum : uint8_t {
  I_BOOT = 1, I_BLE_CONN = 2, I_BLE_DISC = 3, I_MODE_MAN = 4, I_MODE_AUTO = 5,
  I_TARE = 6, I_CAL = 7, I_ZERO = 8, I_PMAX = 9, I_PCLR = 10, I_HOME_START = 11,
  I_HOME_DONE = 12, I_POS_RESTORED = 13, I_SETTING = 14, I_STOP = 15, I_HX_OK = 16,
  I_ERR_CLEARED = 17,
  W_BAT_LOW = 40, W_OVERLOAD = 41, W_AT_MIN = 42, W_AT_MAX = 43, W_NO_LIMITS = 44,
  W_HX_SLOW = 45, W_JOG_TIMEOUT = 46, W_CMD_UNKNOWN = 47, W_CMD_BAD_ARG = 48,
  W_NO_ZERO = 49, W_MODE_BLOCKED = 50,
  E_HX_TIMEOUT = 80, E_HX_SAT = 81, E_NOT_CAL = 82, E_STALL = 83, E_ENC_DIR = 84,
  E_BAT_CRIT = 85, E_HOME_TIMEOUT = 86, E_CAL_FAIL = 87
};
// identyfikatory ustawień w zdarzeniu I_SETTING
enum : uint8_t { S_TGT = 1, S_FMAX, S_THR, S_KP, S_KI, S_MAXD, S_BCAL, S_MINV, S_EINV };

// ----------------------------- PARAMETRY STAŁE -----------------------
static const uint32_t PWM_FREQ   = 20000;
static const uint8_t  PWM_BITS   = 10;
static const int      PWM_MAX    = (1 << PWM_BITS) - 1;
static const int      MIN_DUTY   = 25;     // % – minimum, przy którym silnik rusza
static const int      RET_DUTY   = 35;     // % – powrót do ustawienia bazowego w trybie auto
static const int      HOME_DUTY  = 35;     // % – bazowanie
static const int32_t  POS_TOL    = 5;      // imp. – tolerancja pozycji bazowej
static const uint32_t CTRL_MS    = 20;     // okres regulatora (50 Hz)
static const uint32_t TEL_MS     = 50;     // okres telemetrii (20 Hz)
static const uint32_t BAT_MS     = 500;
static const uint32_t JOG_WDT_MS = 600;    // brak odświeżenia komendy J -> stop
static const uint32_t STALL_MS   = 400;
static const uint32_t HX_TIMEOUT_MS = 500;
static const int      AVG_N      = 40;     // próbki do tarowania/kalibracji (~0,5 s)

// ----------------------------- USTAWIENIA (NVS) ----------------------
struct Settings {
  int32_t offset = 0;      // odczyt HX711 bez obciążenia
  float   scale  = 0;      // jednostki surowe na 1 kg napięcia żyłki (0 = brak kalibracji)
  float   target = 2.0f;   // kg – wartość zadana w trybie auto
  float   fmax   = 4.0f;   // kg – siła maksymalna (bezpieczeństwo)
  float   thr    = 0.3f;   // kg – próg napięcia żyłki
  float   kp     = 40.0f;  // %/kg
  float   ki     = 20.0f;  // %/(kg*s)
  uint8_t maxd   = 80;     // % – maks. wypełnienie PWM
  int32_t pmax   = 0;      // imp. – górny limit pozycji (0 = brak limitów)
  float   bcal   = 1.0f;   // korekta pomiaru baterii
  bool    minv   = false;  // odwrócenie kierunku silnika
  bool    einv   = false;  // odwrócenie kierunku enkodera
} S;

Preferences prefs;

void saveSettings() {
  prefs.putInt("off", S.offset);   prefs.putFloat("scale", S.scale);
  prefs.putFloat("tgt", S.target); prefs.putFloat("fmax", S.fmax);
  prefs.putFloat("thr", S.thr);    prefs.putFloat("kp", S.kp);
  prefs.putFloat("ki", S.ki);      prefs.putUChar("maxd", S.maxd);
  prefs.putInt("pmax", S.pmax);    prefs.putFloat("bcal", S.bcal);
  prefs.putBool("minv", S.minv);   prefs.putBool("einv", S.einv);
}
void loadSettings() {
  S.offset = prefs.getInt("off", S.offset);   S.scale  = prefs.getFloat("scale", S.scale);
  S.target = prefs.getFloat("tgt", S.target); S.fmax   = prefs.getFloat("fmax", S.fmax);
  S.thr    = prefs.getFloat("thr", S.thr);    S.kp     = prefs.getFloat("kp", S.kp);
  S.ki     = prefs.getFloat("ki", S.ki);      S.maxd   = prefs.getUChar("maxd", S.maxd);
  S.pmax   = prefs.getInt("pmax", S.pmax);    S.bcal   = prefs.getFloat("bcal", S.bcal);
  S.minv   = prefs.getBool("minv", S.minv);   S.einv   = prefs.getBool("einv", S.einv);
}
bool calibrated() { return fabsf(S.scale) > 1.0f; }

// ----------------------------- STAN -----------------------------------
enum : uint8_t { MODE_MAN = 0, MODE_AUTO = 1, MODE_HOME = 2 };
enum : uint8_t { REG_NONE = 0, REG_IDLE = 1, REG_HOLD = 2, REG_LOOSEN = 3, REG_RETURN = 4, REG_OVERLOAD = 5 };

uint8_t  mode = MODE_MAN;
uint8_t  regState = REG_NONE;
int      jogDuty = 0;
uint32_t lastJogMs = 0;
int32_t  basePos = 0;
float    integ = 0;
bool     zeroKnown = false;
bool     overload = false;
uint8_t  activeErr = 0;

float    force = 0;           // kg
int32_t  rawFilt = 0;
bool     hxOk = false;
bool     hxSat = false;
uint32_t lastHxMs = 0;
int      sps = 0;

float    batMv = 0;
bool     batLow = false, batCrit = false, batPresent = false;

int      motorDuty = 0;
uint32_t homeStart = 0, homeLastMove = 0;
int32_t  homeLastPos = 0;

// ----------------------------- DZIENNIK ZDARZEŃ -----------------------
struct LogE { uint32_t t; uint32_t seq; uint8_t code; int32_t val; };
static const int LOG_N = 48;
LogE     logs[LOG_N];
uint32_t logSeq = 0, logNext = 0;

void logEv(uint8_t code, int32_t val = 0) {
  LogE &e = logs[logSeq % LOG_N];
  e.t = millis(); e.seq = logSeq; e.code = code; e.val = val;
  logSeq++;
  if (code >= 80) activeErr = code;
  Serial.printf("[%8lu] %c%u val=%ld\n", (unsigned long)e.t,
                code >= 80 ? 'E' : (code >= 40 ? 'W' : 'I'), code, (long)val);
}

// ----------------------------- ENKODER --------------------------------
volatile int32_t encRaw = 0;
volatile uint8_t encPrev = 0;
portMUX_TYPE encMux = portMUX_INITIALIZER_UNLOCKED;
static const DRAM_ATTR int8_t QDEC[16] = {0,-1,1,0, 1,0,0,-1, -1,0,0,1, 0,1,-1,0};

static inline uint8_t IRAM_ATTR encReadAB() {
  uint32_t in = REG_READ(GPIO_IN_REG);       // odczyt bezpieczny także przy wyłączonym cache flash
  return (((in >> PIN_ENC_A) & 1) << 1) | ((in >> PIN_ENC_B) & 1);
}
void IRAM_ATTR encISR() {
  uint8_t s = encReadAB();
  portENTER_CRITICAL_ISR(&encMux);
  encRaw += QDEC[(encPrev << 2) | s];
  encPrev = s;
  portEXIT_CRITICAL_ISR(&encMux);
}
int32_t encGet() {
  portENTER_CRITICAL(&encMux); int32_t v = encRaw; portEXIT_CRITICAL(&encMux);
  return S.einv ? -v : v;
}
void encSet(int32_t v) {
  portENTER_CRITICAL(&encMux); encRaw = S.einv ? -v : v; portEXIT_CRITICAL(&encMux);
}

// ----------------------------- SILNIK ---------------------------------
#if ESP_ARDUINO_VERSION_MAJOR >= 3
void pwmInit() { ledcAttach(PIN_M_IN1, PWM_FREQ, PWM_BITS); ledcAttach(PIN_M_IN2, PWM_FREQ, PWM_BITS); }
void pwmOut(uint8_t pin, int v) { ledcWrite(pin, v); }
#else
void pwmInit() {
  ledcSetup(0, PWM_FREQ, PWM_BITS); ledcSetup(1, PWM_FREQ, PWM_BITS);
  ledcAttachPin(PIN_M_IN1, 0); ledcAttachPin(PIN_M_IN2, 1);
}
void pwmOut(uint8_t pin, int v) { ledcWrite(pin == PIN_M_IN1 ? 0 : 1, v); }
#endif

void motorSet(int duty) {
  duty = constrain(duty, -100, 100);
  motorDuty = duty;
  int d = S.minv ? -duty : duty;
  int v = abs(d) * PWM_MAX / 100;
  if (d > 0)      { pwmOut(PIN_M_IN1, v); pwmOut(PIN_M_IN2, 0); }
  else if (d < 0) { pwmOut(PIN_M_IN1, 0); pwmOut(PIN_M_IN2, v); }
  else            { pwmOut(PIN_M_IN1, PWM_MAX); pwmOut(PIN_M_IN2, PWM_MAX); } // hamowanie silnika
}

// ----------------------------- HX711 ----------------------------------
portMUX_TYPE hxMux = portMUX_INITIALIZER_UNLOCKED;

int32_t hxRead() {               // kanał A, wzmocnienie 128 (25 impulsów zegara)
  uint32_t v = 0;
  portENTER_CRITICAL(&hxMux);    // SCK w stanie wysokim > 60 µs usypia HX711
  for (int i = 0; i < 24; i++) {
    digitalWrite(PIN_HX_SCK, HIGH); delayMicroseconds(1);
    v = (v << 1) | (uint32_t)digitalRead(PIN_HX_DT);
    digitalWrite(PIN_HX_SCK, LOW);  delayMicroseconds(1);
  }
  digitalWrite(PIN_HX_SCK, HIGH); delayMicroseconds(1);
  digitalWrite(PIN_HX_SCK, LOW);  delayMicroseconds(1);
  portEXIT_CRITICAL(&hxMux);
  if (v & 0x800000) v |= 0xFF000000;
  return (int32_t)v;
}

// uśrednianie do tarowania i kalibracji
enum : uint8_t { OP_NONE = 0, OP_TARE, OP_CAL };
uint8_t avgOp = OP_NONE; int avgCnt = 0; int64_t avgAcc = 0; float calMass = 0;

int32_t med3[3]; uint8_t medI = 0; bool medFull = false;
float   emaRaw = 0; bool emaInit = false;
uint16_t spsCount = 0; uint32_t spsT0 = 0; bool spsWarned = false;
uint32_t lastSatLog = 0;

void hxStep() {
  uint32_t now = millis();
  if (digitalRead(PIN_HX_DT) == LOW) {
    int32_t r = hxRead();
    lastHxMs = now; spsCount++;
    bool sat = (r >= 0x7FFFFF || r <= -0x800000);
    if (sat) {
      if (now - lastSatLog > 5000) { logEv(E_HX_SAT, r); lastSatLog = now; }
      hxSat = true; hxOk = false;
      return;
    }
    hxSat = false;
    if (!hxOk) { hxOk = true; if (now > 2000) logEv(I_HX_OK); }

    // mediana z 3 próbek (usuwa pojedyncze zakłócenia) + filtr wykładniczy
    med3[medI] = r; medI = (medI + 1) % 3; if (medI == 0) medFull = true;
    int32_t m = r;
    if (medFull) {
      int32_t a = med3[0], b = med3[1], c = med3[2];
      m = max(min(a, b), min(max(a, b), c));
    }
    if (!emaInit) { emaRaw = m; emaInit = true; } else emaRaw += 0.35f * (m - emaRaw);
    rawFilt = (int32_t)emaRaw;
    force = calibrated() ? (emaRaw - S.offset) / S.scale : 0;

    if (avgOp != OP_NONE) {
      avgAcc += m; avgCnt++;
      if (avgCnt >= AVG_N) {
        int32_t avg = (int32_t)(avgAcc / avgCnt);
        if (avgOp == OP_TARE) {
          S.offset = avg; saveSettings(); logEv(I_TARE, avg);
        } else {
          float sc = (avg - S.offset) / calMass;
          if (fabsf(sc) < 50.0f) logEv(E_CAL_FAIL, (int32_t)sc);   // za mała zmiana sygnału
          else { S.scale = sc; saveSettings(); logEv(I_CAL, (int32_t)sc); }
        }
        avgOp = OP_NONE;
      }
    }
  } else if (hxOk && now - lastHxMs > HX_TIMEOUT_MS) {
    hxOk = false; logEv(E_HX_TIMEOUT);
  } else if (!hxOk && lastHxMs == 0 && now > 2000 && activeErr != E_HX_TIMEOUT) {
    logEv(E_HX_TIMEOUT);                                    // czujnik nie odpowiada od startu
  }

  if (now - spsT0 >= 1000) {
    sps = spsCount; spsCount = 0; spsT0 = now;
    if (!spsWarned && now > 3000 && sps > 0 && sps < 40) { logEv(W_HX_SLOW, sps); spsWarned = true; }
  }
}

// ----------------------------- BATERIA --------------------------------
uint32_t lastBatMs = 0;
void batStep() {
  uint32_t now = millis();
  if (now - lastBatMs < BAT_MS) return;
  lastBatMs = now;
  uint32_t s = 0;
  for (int i = 0; i < 16; i++) s += analogReadMilliVolts(PIN_BAT);
  float mv = s / 16.0f * 2.0f * S.bcal;            // dzielnik 1:2
  batMv = (batMv == 0) ? mv : batMv + 0.3f * (mv - batMv);
  batPresent = batMv > 2500;
  if (!batPresent || now < 3000) return;

  if (!batLow && batMv < 3400) { batLow = true; logEv(W_BAT_LOW, (int32_t)batMv); }
  else if (batLow && batMv > 3500) batLow = false;
  if (!batCrit && batMv < 3300) {
    batCrit = true; logEv(E_BAT_CRIT, (int32_t)batMv);
    if (mode != MODE_MAN) { mode = MODE_MAN; logEv(I_MODE_MAN, 2); }
    jogDuty = 0;
  } else if (batCrit && batMv > 3450) batCrit = false;
}

// ----------------------------- REGULACJA -------------------------------
uint32_t lastMoveMs = 0; int32_t stallRefPos = 0; int stallDir = 0; uint32_t lastStallLog = 0;
uint32_t dirStartMs = 0; int32_t dirStartPos = 0; int dirSign = 0;
bool atMinLogged = false, atMaxLogged = false;
int32_t lastPosForMove = 0; uint32_t lastPosChangeMs = 0;

void stopAll(uint8_t errCode, int32_t val) {
  jogDuty = 0; integ = 0;
  if (mode != MODE_MAN) { mode = MODE_MAN; logEv(I_MODE_MAN, 1); }
  motorSet(0);
  logEv(errCode, val);
}

int applyLimits(int duty, int32_t pos) {
  if (S.pmax <= 0 || !zeroKnown) return duty;
  if (duty > 0 && pos >= S.pmax) { duty = 0; if (!atMaxLogged) { logEv(W_AT_MAX, pos); atMaxLogged = true; } }
  else if (pos < S.pmax - POS_TOL) atMaxLogged = false;
  if (duty < 0 && pos <= 0) { duty = 0; if (!atMinLogged) { logEv(W_AT_MIN, pos); atMinLogged = true; } }
  else if (pos > POS_TOL) atMinLogged = false;
  return duty;
}

int homeStep(int32_t pos) {
  uint32_t now = millis();
  if (now - homeStart > 60000) { mode = MODE_MAN; logEv(E_HOME_TIMEOUT); return 0; }
  if (pos != homeLastPos) { homeLastPos = pos; homeLastMove = now; }
  if (now - homeStart > 600 && now - homeLastMove > 300) {   // silnik stoi mimo zasilania: opór
    motorSet(0);
    encSet(0); zeroKnown = true;
    prefs.putInt("pos", 0); prefs.putBool("zk", true);
    mode = MODE_MAN; logEv(I_HOME_DONE);
    return 0;
  }
  return -HOME_DUTY;
}

int autoStep(float dt, int32_t pos) {
  float db = fmaxf(0.05f, 0.05f * S.target);       // strefa nieczułości
  float e  = force - S.target;
  if (force > S.thr && e > db) {                   // siła za duża: luzuj (regulator PI)
    float imax = 100.0f / fmaxf(S.ki, 0.01f);      // ograniczenie całki
    integ = constrain(integ + e * dt, 0.0f, imax);
    regState = REG_LOOSEN;
    float u = S.kp * e + S.ki * integ;
    return -(int)constrain(u, (float)MIN_DUTY, (float)S.maxd);
  }
  integ = 0;
  if (pos < basePos - POS_TOL && (force < S.thr || e < -db)) {   // wróć do ustawienia bazowego
    regState = REG_RETURN;
    return RET_DUTY;
  }
  regState = (force > S.thr) ? REG_HOLD : REG_IDLE;
  return 0;
}

void controlStep(float dt) {
  int32_t pos = encGet();
  uint32_t now = millis();
  int duty = 0;
  regState = REG_NONE;
  bool forceOk = hxOk && calibrated();

  if (forceOk && force > S.fmax) {
    if (!overload) { overload = true; logEv(W_OVERLOAD, (int32_t)(force * 100)); }
  } else if (overload && (!forceOk || force < 0.9f * S.fmax)) overload = false;

  if (mode == MODE_AUTO && !forceOk) {             // bez pomiaru siły nie ma regulacji
    mode = MODE_MAN; logEv(I_MODE_MAN, 3);
  }
  if (mode == MODE_MAN && jogDuty != 0 && now - lastJogMs > JOG_WDT_MS) {
    jogDuty = 0; logEv(W_JOG_TIMEOUT);
  }

  if (batCrit) duty = 0;
  else if (mode == MODE_HOME) duty = homeStep(pos);
  else if (overload) { duty = -(int)S.maxd; regState = REG_OVERLOAD; }
  else if (mode == MODE_MAN) {
    duty = jogDuty;
    if (duty > 0 && forceOk && force > 0.9f * S.fmax) duty = 0;   // nie dokręcaj przy dużej sile
  } else if (mode == MODE_AUTO) duty = autoStep(dt, pos);

  if (mode != MODE_HOME) duty = applyLimits(duty, pos);
  motorSet(duty);

  // wykrywanie ruchu
  if (pos != lastPosForMove) { lastPosForMove = pos; lastPosChangeMs = now; }

  // zablokowanie silnika (brak impulsów enkodera mimo zasilania)
  int dir = (duty > 0) - (duty < 0);
  if (abs(duty) < 20 || dir != stallDir || mode == MODE_HOME) {
    stallDir = dir; stallRefPos = pos; lastMoveMs = now;
  } else if (pos != stallRefPos) {
    stallRefPos = pos; lastMoveMs = now;
  } else if (now - lastMoveMs > STALL_MS) {
    if (now - lastStallLog > 2000) { stopAll(E_STALL, pos); lastStallLog = now; }
    else { jogDuty = 0; motorSet(0); }
    lastMoveMs = now;
  }

  // zgodność kierunku enkodera z kierunkiem silnika
  if (abs(duty) >= 30 && mode != MODE_HOME) {
    if (dir != dirSign) { dirSign = dir; dirStartMs = now; dirStartPos = pos; }
    else if (now - dirStartMs > 300) {
      int32_t d = pos - dirStartPos;
      if ((int64_t)d * dir < 0 && abs(d) > 10) stopAll(E_ENC_DIR, d);
      dirStartMs = now; dirStartPos = pos;
    }
  } else dirSign = 0;
}

// zapis pozycji, gdy silnik stoi (przywracana po ponownym włączeniu)
int32_t savedPos = INT32_MIN; uint32_t idleSince = 0;
void posSaveStep() {
  uint32_t now = millis();
  if (motorDuty != 0) { idleSince = now; return; }
  if (now - idleSince < 1500) return;
  int32_t p = encGet();
  if (p != savedPos) { prefs.putInt("pos", p); prefs.putBool("zk", zeroKnown); savedPos = p; }
}

// ----------------------------- BLE ------------------------------------
NimBLECharacteristic *chTel, *chCmd, *chCfg, *chLog;
QueueHandle_t cmdQ;
volatile bool evConn = false, evDisc = false; volatile int discReason = 0;
bool bleConnected = false;
volatile bool restartReq = false;

class ServerCB : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* s, NimBLEConnInfo& info) override { evConn = true; }
  void onDisconnect(NimBLEServer* s, NimBLEConnInfo& info, int reason) override {
    discReason = reason; evDisc = true;
    NimBLEDevice::startAdvertising();
  }
};
class CmdCB : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& info) override {
    NimBLEAttValue v = c->getValue();
    char buf[32] = {0};
    size_t n = v.length(); if (n > sizeof(buf) - 1) n = sizeof(buf) - 1;
    memcpy(buf, v.data(), n);
    xQueueSend(cmdQ, buf, 0);
  }
};

void updateCfg() {
  char b[400];
  int n = snprintf(b, sizeof(b),
    "{\"fw\":\"%s\",\"off\":%ld,\"scale\":%.3f,\"tgt\":%.2f,\"fmax\":%.2f,\"thr\":%.2f,"
    "\"kp\":%.1f,\"ki\":%.1f,\"maxd\":%u,\"pmax\":%ld,\"bcal\":%.4f,\"minv\":%d,\"einv\":%d,"
    "\"sps\":%d,\"base\":%ld,\"zk\":%d}",
    FW_VERSION, (long)S.offset, S.scale, S.target, S.fmax, S.thr, S.kp, S.ki, (unsigned)S.maxd,
    (long)S.pmax, S.bcal, S.minv ? 1 : 0, S.einv ? 1 : 0, sps, (long)basePos, zeroKnown ? 1 : 0);
  chCfg->setValue((uint8_t*)b, (size_t)n);
}

struct __attribute__((packed)) Tel {
  uint32_t t;      // ms od uruchomienia
  int16_t  f;      // siła, 0,01 kg
  int32_t  pos;    // pozycja pokrętła, impulsy enkodera
  uint16_t bat;    // napięcie baterii, mV
  uint8_t  mode;   // bity 0–3 tryb, bity 4–7 stan regulatora
  uint8_t  flags;  // b0 HX ok, b1 skalibrowany, b2 limity, b3 ruch, b4 przeciążenie, b5 bateria niska, b6 zero znane, b7 bateria krytyczna
  int8_t   duty;   // wypełnienie PWM, %
  uint8_t  err;    // aktywny kod błędu (0 = brak)
  int32_t  raw;    // przefiltrowany odczyt HX711
};
static_assert(sizeof(Tel) == 20, "Telemetria musi miescic sie w 20 B");

void sendTel() {
  Tel t;
  int32_t pos = encGet();
  t.t = millis();
  t.f = (int16_t)constrain(lroundf(force * 100), -32768L, 32767L);
  t.pos = pos;
  t.bat = (uint16_t)constrain(lroundf(batMv), 0L, 65535L);
  t.mode = (mode & 0x0F) | (regState << 4);
  t.flags = (hxOk ? 1 : 0) | (calibrated() ? 2 : 0) | ((S.pmax > 0 && zeroKnown) ? 4 : 0)
          | ((millis() - lastPosChangeMs < 150) ? 8 : 0) | (overload ? 16 : 0) | (batLow ? 32 : 0)
          | (zeroKnown ? 64 : 0) | (batCrit ? 128 : 0);
  t.duty = (int8_t)motorDuty;
  t.err = activeErr;
  t.raw = rawFilt;
  chTel->setValue((uint8_t*)&t, sizeof(t));
  chTel->notify();
}

uint32_t lastLogSendMs = 0;
void logPump() {
  if (!bleConnected) return;
  uint32_t oldest = logSeq > (uint32_t)LOG_N ? logSeq - LOG_N : 0;
  if (logNext < oldest) logNext = oldest;
  if (logNext >= logSeq || millis() - lastLogSendMs < 15) return;
  const LogE &e = logs[logNext % LOG_N];
  uint8_t b[12];
  uint16_t seq16 = (uint16_t)e.seq;
  memcpy(b, &e.t, 4); memcpy(b + 4, &seq16, 2); b[6] = e.code; b[7] = 0; memcpy(b + 8, &e.val, 4);
  chLog->setValue(b, sizeof(b));
  chLog->notify();
  logNext++; lastLogSendMs = millis();
}

void bleInit() {
  NimBLEDevice::init("Hamulec");
  NimBLEServer* srv = NimBLEDevice::createServer();
  srv->setCallbacks(new ServerCB());
  NimBLEService* svc = srv->createService(UUID_SVC);
  chTel = svc->createCharacteristic(UUID_TEL, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  chCmd = svc->createCharacteristic(UUID_CMD, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  chCfg = svc->createCharacteristic(UUID_CFG, NIMBLE_PROPERTY::READ);
  chLog = svc->createCharacteristic(UUID_LOG, NIMBLE_PROPERTY::NOTIFY);
  chCmd->setCallbacks(new CmdCB());
  updateCfg();
  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->setName("Hamulec");
  adv->addServiceUUID(UUID_SVC);
  adv->start();
}

// ----------------------------- KOMENDY --------------------------------
bool argIn(float a, float lo, float hi) {
  if (isnan(a) || a < lo || a > hi) { logEv(W_CMD_BAD_ARG, (int32_t)(a * 100)); return false; }
  return true;
}

void handleCmd(char* s) {
  char* save = nullptr;
  char* name = strtok_r(s, " \r\n", &save);
  if (!name) return;
  for (char* p = name; *p; p++) *p = toupper(*p);
  char* argS = strtok_r(nullptr, " \r\n", &save);
  float a = argS ? (float)atof(argS) : NAN;
  bool cfgChanged = false;

  if (!strcmp(name, "J")) {                         // J <duty>: ruch ręczny, odświeżany co ~200 ms
    if (mode != MODE_MAN) { if (!isnan(a) && a != 0) logEv(W_MODE_BLOCKED, mode); return; }
    if (batCrit) return;
    jogDuty = isnan(a) ? 0 : constrain((int)a, -100, 100);
    if (jogDuty != 0 && abs(jogDuty) < MIN_DUTY) jogDuty = jogDuty > 0 ? MIN_DUTY : -MIN_DUTY;
    jogDuty = constrain(jogDuty, -(int)S.maxd, (int)S.maxd);
    lastJogMs = millis();
  }
  else if (!strcmp(name, "MAN"))  { jogDuty = 0; integ = 0; if (mode != MODE_MAN) { mode = MODE_MAN; logEv(I_MODE_MAN, 0); } }
  else if (!strcmp(name, "AUTO")) {
    if (!calibrated()) { logEv(E_NOT_CAL); return; }
    if (!hxOk || batCrit) { logEv(W_MODE_BLOCKED, batCrit ? 2 : 1); return; }
    jogDuty = 0; integ = 0; basePos = encGet(); mode = MODE_AUTO;
    logEv(I_MODE_AUTO, (int32_t)(S.target * 100));
    if (S.pmax <= 0 || !zeroKnown) logEv(W_NO_LIMITS);
    cfgChanged = true;
  }
  else if (!strcmp(name, "STOP")) { jogDuty = 0; integ = 0; mode = MODE_MAN; motorSet(0); logEv(I_STOP); }
  else if (!strcmp(name, "TGT"))  { if (argIn(a, 0.05f, S.fmax)) { S.target = a; cfgChanged = true; logEv(I_SETTING, S_TGT); } }
  else if (!strcmp(name, "FMAX")) {
    if (argIn(a, 0.1f, 60.0f)) { S.fmax = a; if (S.target > a) S.target = a; cfgChanged = true; logEv(I_SETTING, S_FMAX); }
  }
  else if (!strcmp(name, "THR"))  { if (argIn(a, 0.0f, 5.0f))   { S.thr = a;  cfgChanged = true; logEv(I_SETTING, S_THR); } }
  else if (!strcmp(name, "KP"))   { if (argIn(a, 0.0f, 500.0f)) { S.kp = a;   cfgChanged = true; logEv(I_SETTING, S_KP); } }
  else if (!strcmp(name, "KI"))   { if (argIn(a, 0.0f, 500.0f)) { S.ki = a;   cfgChanged = true; logEv(I_SETTING, S_KI); } }
  else if (!strcmp(name, "MAXD")) { if (argIn(a, 20.0f, 100.0f)) { S.maxd = (uint8_t)a; cfgChanged = true; logEv(I_SETTING, S_MAXD); } }
  else if (!strcmp(name, "TARE")) { avgOp = OP_TARE; avgCnt = 0; avgAcc = 0; }
  else if (!strcmp(name, "CAL"))  { if (argIn(a, 0.05f, 50.0f)) { calMass = a; avgOp = OP_CAL; avgCnt = 0; avgAcc = 0; } }
  else if (!strcmp(name, "ZERO")) { encSet(0); zeroKnown = true; prefs.putInt("pos", 0); prefs.putBool("zk", true); logEv(I_ZERO); cfgChanged = true; }
  else if (!strcmp(name, "PMAX")) {
    int32_t p = encGet();
    if (!zeroKnown) { logEv(W_NO_ZERO); return; }
    if (p <= POS_TOL) { logEv(W_CMD_BAD_ARG, p); return; }
    S.pmax = p; cfgChanged = true; logEv(I_PMAX, p);
  }
  else if (!strcmp(name, "PCLR")) { S.pmax = 0; cfgChanged = true; logEv(I_PCLR); }
  else if (!strcmp(name, "HOME")) {
    if (batCrit) { logEv(W_MODE_BLOCKED, 2); return; }
    jogDuty = 0; mode = MODE_HOME; homeStart = millis(); homeLastMove = homeStart; homeLastPos = encGet();
    logEv(I_HOME_START);
  }
  else if (!strcmp(name, "BCAL")) {
    if (argIn(a, 2.5f, 4.5f) && batMv > 1000) {
      float uncal = batMv / S.bcal;
      S.bcal = a * 1000.0f / uncal; batMv = a * 1000.0f;
      cfgChanged = true; logEv(I_SETTING, S_BCAL);
    }
  }
  else if (!strcmp(name, "MINV")) { motorSet(0); S.minv = !S.minv; cfgChanged = true; logEv(I_SETTING, S_MINV); }
  else if (!strcmp(name, "EINV")) { int32_t p = encGet(); S.einv = !S.einv; encSet(p); cfgChanged = true; logEv(I_SETTING, S_EINV); }
  else if (!strcmp(name, "CLR"))  { activeErr = 0; logEv(I_ERR_CLEARED); }
  else if (!strcmp(name, "LOGS")) { logNext = 0; }
  else if (!strcmp(name, "CFG"))  { /* tylko odświeżenie konfiguracji */ }
  else if (!strcmp(name, "RST"))  { restartReq = true; }
  else { logEv(W_CMD_UNKNOWN); return; }

  if (cfgChanged) saveSettings();
  updateCfg();
}

// komendy z Serial Monitora (do testów bez telefonu)
char serBuf[32]; uint8_t serLen = 0;
void serialStep() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serLen) { serBuf[serLen] = 0; handleCmd(serBuf); serLen = 0; }
    } else if (serLen < sizeof(serBuf) - 1) serBuf[serLen++] = c;
  }
}

// ----------------------------- SETUP / LOOP ---------------------------
uint32_t lastCtrlMs = 0, lastTelMs = 0, lastPrintMs = 0, lastCfgMs = 0;

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT); digitalWrite(LED_BUILTIN, HIGH);   // dioda aktywna stanem niskim

  prefs.begin("hamulec", false);
  loadSettings();

  pwmInit(); motorSet(0);

  pinMode(PIN_ENC_A, INPUT_PULLUP); pinMode(PIN_ENC_B, INPUT_PULLUP);
  encPrev = encReadAB();
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), encISR, CHANGE);

  pinMode(PIN_HX_DT, INPUT); pinMode(PIN_HX_SCK, OUTPUT); digitalWrite(PIN_HX_SCK, LOW);
  analogReadResolution(12);

  cmdQ = xQueueCreate(8, 32);
  logEv(I_BOOT, (int32_t)esp_reset_reason());

  zeroKnown = prefs.getBool("zk", false);
  int32_t p = prefs.getInt("pos", 0);
  encSet(p); savedPos = p;
  if (zeroKnown) logEv(I_POS_RESTORED, p);

  bleInit();
  lastJogMs = lastMoveMs = lastPosChangeMs = idleSince = millis();
}

void loop() {
  uint32_t now = millis();

  hxStep();
  batStep();
  serialStep();

  char cmd[32];
  while (xQueueReceive(cmdQ, cmd, 0) == pdTRUE) handleCmd(cmd);

  if (evConn) { evConn = false; bleConnected = true; logEv(I_BLE_CONN); }
  if (evDisc) {
    evDisc = false;
    bleConnected = NimBLEDevice::getServer()->getConnectedCount() > 0;
    logEv(I_BLE_DISC, discReason);
    if (mode == MODE_MAN) jogDuty = 0;                // tryb automatyczny działa dalej bez telefonu
  }

  if (now - lastCtrlMs >= CTRL_MS) { controlStep((now - lastCtrlMs) / 1000.0f); lastCtrlMs = now; }
  posSaveStep();

  if (bleConnected && now - lastTelMs >= TEL_MS) { sendTel(); lastTelMs = now; }
  if (now - lastCfgMs >= 2000) { updateCfg(); lastCfgMs = now; }   // odświeża m.in. liczbę próbek/s
  logPump();

  // dioda: świeci przy połączeniu, miga przy aktywnym błędzie
  bool led = activeErr ? ((now / 250) % 2) : bleConnected;
  digitalWrite(LED_BUILTIN, led ? LOW : HIGH);

  if (now - lastPrintMs >= 1000) {
    lastPrintMs = now;
    Serial.printf("F=%.2f kg raw=%ld pos=%ld bat=%.2f V tryb=%u reg=%u duty=%d sps=%d err=%u\n",
                  force, (long)rawFilt, (long)encGet(), batMv / 1000.0f, mode, regState, motorDuty, sps, activeErr);
  }

  if (restartReq) { motorSet(0); delay(200); ESP.restart(); }
}
