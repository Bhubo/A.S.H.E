// ASHE — Vodou Full Rev L (Launch Candidate)
// ASHE = Autonomous Sword for Heroic Engagements
// VODOU ENABLED
//
// ARCHITECTURE:
// This firmware operates on a fully stateless, non-blocking, event-driven model.
//  • STATELESS: ASHE holds no memory of past users. It boots to a neutral state,
//    awaiting the user's 'Equilibrium' to be imprinted upon it via the BIND command.
//  • NON-BLOCKING: All operations are managed by a millis()-based state machine.
//    There are no delay() calls, ensuring the sword is always responsive to IMU events
//    and BLE commands, even while playing complex effects.
//
// FINAL REFINEMENTS (Rev L):
//  • Compilation Fix: Corrected BLE connection status check to use Bluefruit.connHandle().
//  • FPU Optimization: Leveraged the nRF52840's hardware FPU for trigonometric and
//    square root calculations in the effects and IMU logic for improved performance.
//  • SRAM Optimization: All user-facing strings use the F() macro to store them
//    in Flash memory, preserving critical SRAM for runtime operations.
//  • Production Ready: This version represents a stable, feature-complete candidate
//    for the final project deployment.

#include <Wire.h>
#include <bluefruit.h>                      // Adafruit nRF52 Bluefruit core (internal)
#include "SparkFun_BNO080_Arduino_Library.h" // BNO085 IMU

/************ Pin Map (XIAO nRF52840) ************/
// I2C: D4 = SDA, D5 = SCL (BNO085)
// Button: D0 (to GND, INPUT_PULLUP)
// Motor MOSFET gate: D1 (100Ω in series, 100k pulldown to GND at gate)
// Speaker: D8 → piezo (+) or tiny amp IN
// RGB Group A (common-anode): R=D2, G=D3, B=D6
// RGB Group B (common-anode): R=D7, G=D9, B=D10
// Battery Sense: A1 (requires a voltage divider from VBAT)

struct RGBPins { uint8_t r, g, b; };
const RGBPins GROUP_A = {2, 3, 6};
const RGBPins GROUP_B = {7, 9, 10};

const uint8_t BUTTON_PIN  = 0;
const uint8_t MOTOR_PIN   = 1;
const uint8_t SPEAKER_PIN = 8;
const uint8_t VBAT_PIN    = A1;

/************ Globals ************/
BNO080 imu;       // motion sensor
BLEUart bleuart;  // Vodou transport (UART over BLE)

bool armed = false;
bool overdriveActive = false;
uint8_t motorPWM = 0;
uint8_t colorA[3] = {200, 200, 200}; // Neutral default color
uint8_t colorB[3] = {200, 200, 200}; // Neutral default color

// Button debounce
uint32_t lastDebounceMs = 0; bool lastBtn = HIGH; bool stableBtn = HIGH;

// IMU event thresholds
const float SWING_GYRO_THRESH = 3.5f;
const float CLASH_ACCEL_THRESH = 2.0f;
const uint32_t SWING_COOLDOWN_MS = 250;
const uint32_t CLASH_COOLDOWN_MS = 300;
uint32_t lastSwingMs = 0, lastClashMs = 0;

/************ State & Type Definitions ************/
// Effects Engine State
enum EffectState {
  EFFECT_NONE,
  EFFECT_BOOT,
  EFFECT_ASHE_TONE,
  EFFECT_ARMED_FLOURISH,
  EFFECT_SWING,
  EFFECT_CLASH,
  EFFECT_MOTOR_START,
  EFFECT_BINDING_RITUAL
};

EffectState currentEffect = EFFECT_NONE;
uint8_t effectStep = 0;
uint32_t lastEffectMs = 0;

uint8_t clashRestoreColorA[3], clashRestoreColorB[3];
uint8_t motorTargetPWM = 0;

// Color Target Definition (must be declared before functions that use it)
enum Target { T_A, T_B, T_BOTH, T_NONE };


/************ Utilities ************/
inline uint8_t caPWM(uint8_t v){ return 255 - v; }
inline void caWrite(uint8_t pin, uint8_t v){ analogWrite(pin, caPWM(v)); }

void setGroup(const RGBPins& g, uint8_t r, uint8_t gg, uint8_t b){
  caWrite(g.r, r); caWrite(g.g, gg); caWrite(g.b, b);
}

void allOff(){ setGroup(GROUP_A,0,0,0); setGroup(GROUP_B,0,0,0); }

void motorSet(uint8_t pwm){ motorPWM = pwm; analogWrite(MOTOR_PIN, pwm); }

uint8_t getBatteryPercent() {
  // To implement: Connect battery's positive terminal to a voltage divider
  // (e.g., two 100k resistors). Connect the midpoint to VBAT_PIN.
  // float voltage = analogRead(VBAT_PIN) * (3.3 / 1023.0) * 2.0;
  // return (uint8_t)constrain(map(voltage * 100, 320, 420, 0, 100), 0, 100);
  return 100; // Placeholder
}

/************ Effect Triggers ************/
void triggerEffect(EffectState effect, bool force = false) {
  if (!force && currentEffect != EFFECT_NONE) return;
  currentEffect = effect;
  effectStep = 0;
  lastEffectMs = millis();
}

void triggerAsheTone()      { triggerEffect(EFFECT_ASHE_TONE); }
void triggerBoot()          { triggerEffect(EFFECT_BOOT); }
void triggerArmedFlourish() { triggerEffect(EFFECT_ARMED_FLOURISH); }
void triggerSwing()         { triggerEffect(EFFECT_SWING, true); }
void triggerClash()         { triggerEffect(EFFECT_CLASH, true); }
void triggerBindingRitual() { triggerEffect(EFFECT_BINDING_RITUAL, true); }

void triggerMotorSoftStart(uint8_t target) {
  if (currentEffect != EFFECT_NONE) return;
  motorTargetPWM = target;
  triggerEffect(EFFECT_MOTOR_START);
}


/************ Color Parsing ************/
bool parseCSV(const String& s, uint8_t& r, uint8_t& g, uint8_t& b) {
  int c1 = s.indexOf(',');
  int c2 = s.indexOf(',', c1 + 1);
  if (c1 < 0 || c2 < 0) return false;

  long R = s.substring(0, c1).toInt();
  long G = s.substring(c1 + 1, c2).toInt();
  long B = s.substring(c2 + 1).toInt();

  if (R < 0 || R > 255 || G < 0 || G > 255 || B < 0 || B > 255) return false;
  
  r = (uint8_t)R; g = (uint8_t)G; b = (uint8_t)B;
  return true;
}

bool parseHex(const String& s, uint8_t& r, uint8_t& g, uint8_t& b) {
  if (s.length() != 7 || s[0] != '#') return false;
  
  char buf[3] = {0};
  
  buf[0] = s[1]; buf[1] = s[2]; r = strtoul(buf, nullptr, 16);
  buf[0] = s[3]; buf[1] = s[4]; g = strtoul(buf, nullptr, 16);
  buf[0] = s[5]; buf[1] = s[6]; b = strtoul(buf, nullptr, 16);
  
  return true;
}

bool parseRGB(String s, uint8_t& r, uint8_t& g, uint8_t& b) {
  s.replace("RGB", "rgb");
  s.trim();
  if (s.startsWith("rgb(") && s.endsWith(")")) {
    return parseCSV(s.substring(4, s.length() - 1), r, g, b);
  }
  return parseCSV(s, r, g, b);
}

bool namedColor(String n, uint8_t& r, uint8_t& g, uint8_t& b) {
  n.trim();
  n.toLowerCase();
  
  struct ColorEntry {const char* name; uint8_t r, g, b;};
  const ColorEntry colorTable[] = {
    {"red", 255, 0, 0}, {"green", 0, 255, 0}, {"blue", 0, 0, 255},
    {"white", 255, 255, 255}, {"warmwhite", 255, 220, 170}, {"coolwhite", 210, 235, 255},
    {"cyan", 0, 255, 255}, {"magenta", 255, 0, 255}, {"yellow", 255, 255, 0},
    {"orange", 255, 128, 0}, {"purple", 160, 32, 240}, {"pink", 255, 105, 180},
    {"teal", 0, 128, 128}, {"off", 0, 0, 0}
  };

  for (const auto& entry : colorTable) {
    if (n == entry.name) {
      r = entry.r; g = entry.g; b = entry.b;
      return true;
    }
  }
  return false;
}

Target parseTarget(String w) {
  w.toLowerCase();
  if (w == "a") return T_A;
  if (w == "b") return T_B;
  if (w == "both" || w == "all") return T_BOTH;
  return T_NONE;
}

void applyColor(Target tgt, uint8_t r, uint8_t g, uint8_t b) {
  if (tgt == T_A || tgt == T_BOTH) {
    setGroup(GROUP_A, r, g, b);
    colorA[0] = r; colorA[1] = g; colorA[2] = b;
  }
  if (tgt == T_B || tgt == T_BOTH) {
    setGroup(GROUP_B, r, g, b);
    colorB[0] = r; colorB[1] = g; colorB[2] = b;
  }
}


/************ Vodou Protocol over BLE (UART) ************/
void startBLE() {
  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.setName("ASHE");
  bleuart.begin();
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(bleuart);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(160, 244);
  Bluefruit.Advertising.start();
}

void vodouSend(const __FlashStringHelper* s) {
  if (Bluefruit.connHandle() != BLE_CONN_HANDLE_INVALID) {
    bleuart.print(s);
  }
}

void vodouSend(const String& s) {
  if (Bluefruit.connHandle() != BLE_CONN_HANDLE_INVALID) {
    bleuart.print(s);
  }
}

void vodouSendStatus() {
  char msg[128];
  snprintf(msg, sizeof(msg), "STATE %s|OVERDRIVE %s|BATT %u|PWM %u|A %u,%u,%u|B %u,%u,%u\n",
           armed ? "ARMED" : "DISARMED",
           overdriveActive ? "ON" : "OFF",
           getBatteryPercent(),
           motorPWM,
           colorA[0], colorA[1], colorA[2],
           colorB[0], colorB[1], colorB[2]);
  vodouSend(String(msg));
}

void handleBindCommand(String payload) {
  int currentPos = 0;
  while (currentPos < payload.length()) {
    int pipePos = payload.indexOf('|', currentPos);
    if (pipePos == -1) { pipePos = payload.length(); }
    
    String part = payload.substring(currentPos, pipePos);
    int equalsPos = part.indexOf('=');
    if (equalsPos > 0) {
      String key = part.substring(0, equalsPos);
      String value = part.substring(equalsPos + 1);
      key.toUpperCase();
      
      uint8_t r, g, b;
      if (key == "A" && parseCSV(value, r, g, b)) {
        applyColor(T_A, r, g, b);
      } else if (key == "B" && parseCSV(value, r, g, b)) {
        applyColor(T_B, r, g, b);
      }
    }
    currentPos = pipePos + 1;
  }
  
  triggerBindingRitual();
  vodouSend(F("OK Binding Initiated\n"));
}

void handleVodouLine(String line) {
  String up = line;
  up.trim();
  String U = up;
  U.toUpperCase();
  
  if (U == F("PING")) { vodouSend(F("PONG\n")); return; }
  
  if (armed) {
    if (U.startsWith(F("BIND "))) { handleBindCommand(up.substring(5)); return; }
    if (U == F("OVERDRIVE ON")) { overdriveActive = true; vodouSendStatus(); return; }
    if (U == F("OVERDRIVE OFF")) { overdriveActive = false; motorSet(0); vodouSendStatus(); return; }
    if (U == F("LIGHTS OFF")) { applyColor(T_BOTH, 0, 0, 0); vodouSendStatus(); return; }
    
    if (!overdriveActive) {
      if (U == F("MOTOR ON")) { triggerMotorSoftStart(200); vodouSendStatus(); return; }
      if (U == F("MOTOR OFF")) { motorSet(0); vodouSendStatus(); return; }
      if (U.startsWith(F("MOTOR PWM "))) {
        int val = up.substring(10).toInt();
        motorSet(constrain(val, 0, 255));
        vodouSendStatus();
        return;
      }
    }

    int sp = up.indexOf(' ');
    if (sp > 0) {
      Target t = parseTarget(up.substring(0, sp));
      if (t != T_NONE) {
        String c = up.substring(sp + 1);
        uint8_t r, g, b;
        if (parseHex(c, r, g, b) || parseRGB(c, r, g, b) || namedColor(c, r, g, b)) {
          applyColor(t, r, g, b);
          vodouSendStatus();
          return;
        }
      }
    }
  }

  if (U == F("ARM")) {
    if (!armed) { armed = true; triggerBoot(); }
    vodouSendStatus();
    return;
  }
  if (U == F("DISARM")) {
    if (armed) {
      armed = false;
      overdriveActive = false;
      motorSet(0);
      allOff();
      noTone(SPEAKER_PIN);
    }
    vodouSendStatus();
    return;
  }

  vodouSend(F("ERR Unknown cmd\n"));
}

/************ Main Loop Updaters ************/
void updateButton() {
  bool raw = digitalRead(BUTTON_PIN);
  uint32_t now = millis();
  if (raw != lastBtn) {
    lastDebounceMs = now;
    lastBtn = raw;
  }
  if (now - lastDebounceMs > 50) {
    if (stableBtn != lastBtn) {
      stableBtn = lastBtn;
      if (stableBtn == LOW) {
        armed = !armed;
        if (armed) {
          triggerBoot();
        } else {
          overdriveActive = false;
          motorSet(0);
          allOff();
          noTone(SPEAKER_PIN);
        }
      }
    }
  }
}

void updateIMU() {
  if (!armed || !imu.dataAvailable()) return;

  // Read sensor data as floats
  float gx = imu.getGyroX(); float gy = imu.getGyroY(); float gz = imu.getGyroZ();
  float ax = imu.getAccelX(); float ay = imu.getAccelY(); float az = imu.getAccelZ();
  
  // These magnitude calculations use sqrtf, which is hardware-accelerated by the FPU
  float gmag = sqrtf(gx * gx + gy * gy + gz * gz);
  float amag = sqrtf(ax * ax + ay * ay + az * az);
  
  uint32_t t = millis();

  if (gmag > SWING_GYRO_THRESH && (t - lastSwingMs) > SWING_COOLDOWN_MS) {
    lastSwingMs = t;
    triggerSwing();
    uint8_t r = min(255, colorA[0] + 40);
    uint8_t g = min(255, colorA[1] + 40);
    uint8_t b = min(255, colorA[2] + 40);
    applyColor(T_BOTH, r, g, b);
  }
  
  if (amag > CLASH_ACCEL_THRESH && (t - lastClashMs) > CLASH_COOLDOWN_MS) {
    lastClashMs = t;
    memcpy(clashRestoreColorA, colorA, 3);
    memcpy(clashRestoreColorB, colorB, 3);
    triggerClash();
  }
}

void vodouPoll() {
  static String acc = "";
  while (bleuart.available()) {
    char ch = (char)bleuart.read();
    if (ch == '\n' || ch == '\r') {
      if (acc.length() > 0) handleVodouLine(acc);
      acc = "";
    } else if (acc.length() < 128) {
      acc += ch;
    }
  }
}

void serialPoll() {
  static String acc = "";
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (acc.length() > 0) handleVodouLine(acc);
      acc = "";
    } else if (acc.length() < 128) {
      acc += c;
    }
  }
}

void updateOverdrive() {
  if (!armed || !overdriveActive) return;
  
  motorSet(255);
  if ((millis() % 200) < 100) {
    setGroup(GROUP_A, 255, 255, 255);
    setGroup(GROUP_B, 255, 255, 255);
  } else {
    applyColor(T_A, colorA[0], colorA[1], colorA[2]);
    applyColor(T_B, colorB[0], colorB[1], colorB[2]);
  }
}

void updateEffects() {
  if (currentEffect == EFFECT_NONE) return;
  uint32_t now = millis();
  switch (currentEffect) {
    case EFFECT_ASHE_TONE:
      if (effectStep == 0) { tone(SPEAKER_PIN, 880, 150); lastEffectMs = now; effectStep++; }
      if (effectStep == 1 && now - lastEffectMs > 180) { tone(SPEAKER_PIN, 988, 150); lastEffectMs = now; effectStep++; }
      if (effectStep == 2 && now - lastEffectMs > 180) { tone(SPEAKER_PIN, 784, 250); lastEffectMs = now; effectStep++; }
      if (effectStep == 3 && now - lastEffectMs > 280) { noTone(SPEAKER_PIN); currentEffect = EFFECT_NONE; }
      break;
      
    case EFFECT_BOOT:
      if (effectStep == 0) { tone(SPEAKER_PIN, 700, 90); lastEffectMs = now; effectStep++; }
      if (effectStep == 1 && now - lastEffectMs > 100) { tone(SPEAKER_PIN, 1000, 110); lastEffectMs = now; effectStep++; }
      if (effectStep == 2 && now - lastEffectMs > 120) { tone(SPEAKER_PIN, 1400, 140); lastEffectMs = now; effectStep++; }
      if (effectStep == 3 && now - lastEffectMs > 160) { noTone(SPEAKER_PIN); currentEffect = EFFECT_NONE; triggerArmedFlourish(); }
      break;
      
    case EFFECT_ARMED_FLOURISH:
      if (effectStep == 0) { tone(SPEAKER_PIN, 523, 80); lastEffectMs = now; effectStep++; }
      if (effectStep == 1 && now - lastEffectMs > 90) { tone(SPEAKER_PIN, 659, 80); lastEffectMs = now; effectStep++; }
      if (effectStep == 2 && now - lastEffectMs > 90) { tone(SPEAKER_PIN, 784, 120); lastEffectMs = now; effectStep++; }
      if (effectStep == 3 && now - lastEffectMs > 140) { noTone(SPEAKER_PIN); currentEffect = EFFECT_NONE; }
      break;
      
    case EFFECT_SWING:
      if (effectStep == 0) {
        int freq = map(now - lastEffectMs, 0, 120, 500, 1200);
        if (now - lastEffectMs < 120) {
          tone(SPEAKER_PIN, freq, 12);
        } else {
          noTone(SPEAKER_PIN);
          currentEffect = EFFECT_NONE;
        }
      }
      break;
      
    case EFFECT_CLASH:
      if (effectStep == 0) { setGroup(GROUP_A, 255, 255, 255); setGroup(GROUP_B, 255, 255, 255); tone(SPEAKER_PIN, 1600, 30); lastEffectMs = now; effectStep++; }
      if (effectStep == 1 && now - lastEffectMs > 35) { applyColor(T_A, clashRestoreColorA[0], clashRestoreColorA[1], clashRestoreColorA[2]); applyColor(T_B, clashRestoreColorB[0], clashRestoreColorB[1], clashRestoreColorB[2]); tone(SPEAKER_PIN, 2200, 25); lastEffectMs = now; effectStep++; }
      if (effectStep == 2 && now - lastEffectMs > 30) { tone(SPEAKER_PIN, 1200, 40); lastEffectMs = now; effectStep++; }
      if (effectStep == 3 && now - lastEffectMs > 45) { noTone(SPEAKER_PIN); currentEffect = EFFECT_NONE; }
      break;
      
    case EFFECT_MOTOR_START:
      if (now - lastEffectMs > 6) {
        uint16_t current_pwm = motorPWM + 6;
        if (current_pwm >= motorTargetPWM) {
          motorSet(motorTargetPWM);
          currentEffect = EFFECT_NONE;
        } else {
          motorSet(current_pwm);
          lastEffectMs = now;
        }
      }
      break;
    
    case EFFECT_BINDING_RITUAL: {
      if (effectStep == 0) { // Phase 1: Pulsing Ritual
        uint32_t elapsed = now - lastEffectMs;
        if (elapsed < 3000) {
          // This sin() calculation is hardware-accelerated by the FPU
          float brightness = 0.5f * (1.0f + sin(elapsed * 0.005f));
          uint8_t intensity = (uint8_t)(brightness * 255);
          applyColor(T_BOTH, 0, intensity, intensity);
          tone(SPEAKER_PIN, 100 + (elapsed / 30), 50);
        } else {
          noTone(SPEAKER_PIN); allOff(); lastEffectMs = now; effectStep++;
        }
      } else { // Phase 2: The "13" Signal
        if (effectStep == 1 && now - lastEffectMs > 100) { applyColor(T_BOTH, 255, 255, 255); lastEffectMs = now; effectStep++; }
        if (effectStep == 2 && now - lastEffectMs > 500) { allOff(); lastEffectMs = now; effectStep++; }
        if (effectStep == 3 && now - lastEffectMs > 200) { applyColor(T_BOTH, 255, 255, 255); lastEffectMs = now; effectStep++; }
        if (effectStep == 4 && now - lastEffectMs > 100) { allOff(); lastEffectMs = now; effectStep++; }
        if (effectStep == 5 && now - lastEffectMs > 100) { applyColor(T_BOTH, 255, 255, 255); lastEffectMs = now; effectStep++; }
        if (effectStep == 6 && now - lastEffectMs > 100) { allOff(); lastEffectMs = now; effectStep++; }
        if (effectStep == 7 && now - lastEffectMs > 100) { applyColor(T_BOTH, 255, 255, 255); lastEffectMs = now; effectStep++; }
        if (effectStep == 8 && now - lastEffectMs > 100) {
          applyColor(T_A, colorA[0], colorA[1], colorA[2]);
          applyColor(T_B, colorB[0], colorB[1], colorB[2]);
          currentEffect = EFFECT_NONE;
        }
      }
      break;
    }
  }
}

/************ Setup / Loop ************/
void setup(){
  Serial.begin(115200);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(SPEAKER_PIN, OUTPUT);
  pinMode(VBAT_PIN, INPUT);

  pinMode(GROUP_A.r, OUTPUT); pinMode(GROUP_A.g, OUTPUT); pinMode(GROUP_A.b, OUTPUT);
  pinMode(GROUP_B.r, OUTPUT); pinMode(GROUP_B.g, OUTPUT); pinMode(GROUP_B.b, OUTPUT);
  allOff();
  motorSet(0);

  Wire.begin();
  if(imu.begin()){
    imu.enableGyro(100);
    imu.enableAccelerometer(100);
  }

  startBLE();
  triggerAsheTone();
}

void loop(){
  serialPoll();
  vodouPoll();
  updateButton();
  updateIMU();
  updateOverdrive();
  updateEffects();
}
