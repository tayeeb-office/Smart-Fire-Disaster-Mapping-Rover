/*
 * ============================================================================
 *  FIRE RESCUE ROVER - ESP32 DevKit V1 (38-pin)
 * ============================================================================
 *  Revision notes (changes vs. previous "final" version):
 *   1. LD2410B: HARD software detection limit (LD2410_MAX_RANGE_CM, now 1.50 m - see 8-10).
 *      A target only counts if its moving/still bit is set AND that component's
 *      distance is 1..600 cm. Presence debounce now runs on the in-range result,
 *      so any target reported beyond the limit produces AREA CLEAR (buzzer off).
 *      Frame header byte 0xAA is now also validated. OUT pin stays informational.
 *   2. MQ-2: non-blocking 60 s warm-up + clean-air baseline learning, EMA
 *      filtering, rise-above-baseline thresholds with hysteresis and
 *      consecutive-sample confirmation. The original 1.50 V absolute HIGH
 *      threshold is kept, so strong gas is still flagged as before.
 *   3. VL53L0X: "out of range / no object" (RangeStatus 4 or >2000 mm) is no
 *      longer treated as a sensor failure, and the I2C/ranging return code is
 *      now checked. Genuine I2C failures still trigger the reinit/recovery cycle.
 *   4. System health: NORMAL / WARNING / CRITICAL with explicit reasons.
 *      A non-critical sensor fault now gives WARNING, not CRITICAL.
 *   5. Dashboard fully redesigned (vanilla HTML/CSS/JS, no external libs,
 *      works offline in AP mode). Routes, pins, motor, servo, WiFi unchanged.
 *   6. Buzzer (rev 4): sounds AUTOMATICALLY while a target is detected within
 *      the zone, can be silenced wirelessly from the dashboard (/buzzer?state=off,
 *      re-arm with state=on), and is ALWAYS off when the area is clear. The
 *      silence resets as soon as the area clears, so the next detection sounds
 *      again. It can never be turned on without a detected target.
 *   7. Rev 3: MQ-4 gets the same non-blocking warm-up display as MQ-7 (60 s,
 *      thresholds unchanged). One status vocabulary on every card (OK / WARNING /
 *      ERROR / OFFLINE / WARMING UP / SCANNING, CRITICAL for hazards). Health ring
 *      shows the real sensor count, not a percentage. Rescue card uses the exact
 *      specified wording incl. RADAR LINK ERROR. Buzzer gets a real ON/OFF switch
 *      synced to /buzzer responses and every /data poll.
 *   8. Rev 5: LD2410B detection radius reduced to 1.00 m for crowded test areas.
 *      Hardware: distance resolution set to 0.20 m (cmd 0x00AA, verified via
 *      0x00AB query, applied with restart 0x00A3) and max moving/still gate = 5
 *      (5 x 0.20 m = 1.00 m, cmd 0x0060). Every config command is ACK-checked,
 *      per HLK-LD2410B Serial Communication Protocol V1.07. Software hard limit
 *      100 cm is kept on top, so >1 m is ALWAYS "area clear" even if the module
 *      firmware rejects the resolution command. Keyboard driving: W/A/S/D or
 *      arrow keys (hold = drive, release = stop), Space = STOP, 1/2/3 = speed.
 *   9. Rev 6: radius reduced further to 0.60 m (0.20 m x gate 3). The hardware
 *      gate is now derived from LD2410_MAX_RANGE_CM, so changing that ONE number
 *      re-programs the module on next boot. Valid hardware range: 40..160 cm
 *      (protocol gates 2..8 at 0.20 m). Serial/dashboard texts follow it.
 *  10. Rev 7: demo standard radius 1.50 m (software) with module gate 8 at
 *      0.20 m/gate (1.60 m hardware, the module maximum at this resolution).
 * ============================================================================
 */

#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// ==================== WiFi ====================
const char* ssid = "RescueRover";
const char* password = "rover1234";
WebServer server(80);

// ==================== PIN DEFINITIONS (UNCHANGED) ====================
#define MQ2_PIN 39
#define MQ4_PIN 34
#define MQ7_PIN 35
#define DHTPIN 26
#define TRIG_PIN 33
// IMPORTANT: Standard HC-SR04 ECHO may be 5V.
// Use a voltage divider/level shifter before ESP32 GPIO32.
#define ECHO_PIN 32
#define MPU_ADDR 0x68
#define VL53_ADDR 0x29
#define MPU_INT 4
#define LD2410_OUT 27
#define LD2410_RX2 16
#define LD2410_TX2 17
#define VL53L0X_XSHUT 5
#define BUZZER_PIN 18

#define SERVO_PIN 19

#define L_RPWM 13
#define L_LPWM 15
#define L_EN   2
#define R_RPWM 25
#define R_LPWM 14
#define R_EN   23

#define TILT_LIMIT_DEG 45.0
#define I2C_SDA 21
#define I2C_SCL 22
#define GAS_SAMPLE_COUNT 24

HardwareSerial LD2410Serial(2);

void printDashboard(); // defined at the bottom

// ==================== SERVO CONFIG (configurable, full range) ====================
int SERVO_MIN_ANGLE = 0, SERVO_MAX_ANGLE = 180;   // mechanical limits - adjust if servo can't safely reach ends
int SERVO_MIN_US = 500, SERVO_MAX_US = 2400;
int SERVO_STEP_DEG = 2, SERVO_SCAN_INTERVAL_MS = 30; // smooth, small steps, no delay()

// ==================== MOTOR CONFIG ====================
int MOTOR_FORWARD_PWM = 180, MOTOR_BACKWARD_PWM = 180, MOTOR_TURN_PWM = 160, MOTOR_MAX_PWM = 255;
float LEFT_MOTOR_SCALE = 1.000, RIGHT_MOTOR_SCALE = 1.000;
int commandPWM = 0, leftPWM = 0, rightPWM = 0;
String motorState = "STOPPED";
String speedMode = "MEDIUM"; // LOW / MEDIUM / HIGH
unsigned long lastDriveCmdTime = 0;
const unsigned long DRIVE_WATCHDOG_MS = 600; // dead-man timeout

// ==================== VL53L0X STATE ====================
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
bool vl53DeviceFound = false, vl53InitOK = false, vl53EverInitialized = false;
int vl53LastValidMM = -1; unsigned long vl53LastValidTime = 0;
int vl53ConsecutiveFails = 0, vl53RawMM = -1, vl53FilteredMM = -1;
unsigned long tVl53Retry = 0; const unsigned long VL53_RETRY_INTERVAL = 5000;
int vl53History[3] = {-1, -1, -1}; int vl53HistIdx = 0;
const int VL53_MAX_PLAUSIBLE_MM = 2000; // sensor's realistic max range - anything beyond is out-of-range, never "valid"
const int VL53_FAIL_LIMIT = 20; // consecutive invalid reads before forcing a reinit/recovery cycle
bool vl53NoObject = false;      // sensor answered, but nothing within ~2 m (NOT a sensor fault)

// Explicit sensor state machine (Requirement 5)
enum Vl53State { VL53_NOT_FOUND, VL53_INIT_FAILED, VL53_VALID, VL53_STALE, VL53_RECOVERING };
Vl53State vl53CurrentState = VL53_NOT_FOUND;

// ==================== MPU6050 STATE ====================
bool mpuFound = false; int16_t ax, ay, az, gx, gy, gz;
float pitch = 0, roll = 0, lastValidPitch = 0, lastValidRoll = 0;
bool mpuReadOK = false;
unsigned long mpuLastValidTime = 0;

// ==================== DHT22 STATE ====================
byte dhtData[5];
float dhtTemp = 0, dhtHum = 0, lastValidTemp = 0, lastValidHum = 0;
bool dhtValidNow = false; unsigned long dhtLastValidTime = 0;
const float TEMP_WARN_C = 50.0;  // system WARNING at/above this (recent valid reading)
const float TEMP_CRIT_C = 65.0;  // system CRITICAL at/above this (DHT22 rated to 80 C)

// ==================== GAS STATE ====================
int mq2Raw = 0, mq4Raw = 0, mq7Raw = 0;
float mq2V = 0, mq4V = 0, mq7V = 0;
int mq2ZeroCount = 0, mq4ZeroCount = 0, mq7ZeroCount = 0;
unsigned long bootTime = 0; bool mq7WarmedUp = false;
const unsigned long MQ7_WARMUP_MS = 20000; // non-blocking warm-up
bool mq4WarmedUp = false;
const unsigned long MQ4_WARMUP_MS = 60000; // NEW: display-only warm-up; thresholds unchanged

// ---- MQ-2 false-warning fix: baseline + hysteresis + confirmation ----
// All values are in the SAME units the existing code already used (ADC voltage
// estimate after your divider, 0..3.3 V). Tune using the "baseline" and "rise"
// values shown on the dashboard in clean air.
const unsigned long MQ2_WARMUP_MS = 60000;   // heater stabilisation + baseline learning (non-blocking)
const float MQ2_EMA_ALPHA           = 0.40;  // light smoothing of the 24-sample average (sampled every 300 ms)
const float MQ2_BASE_ALPHA_WARMUP   = 0.05;  // baseline learning speed during warm-up
const float MQ2_BASE_ALPHA_DOWN     = 0.020; // baseline follows clean air downwards (~15 s)
const float MQ2_BASE_ALPHA_UP       = 0.002; // ...and upwards only very slowly (~150 s)
const float MQ2_BASELINE_MAX_V      = 1.00;  // baseline can never learn above this (prevents masking gas)
const float MQ2_WARN_DELTA_V        = 0.25;  // WARNING: filtered voltage this far above baseline
const float MQ2_WARN_CLEAR_DELTA_V  = 0.15;  // WARNING clears below this rise (hysteresis)
const float MQ2_HIGH_ABS_V          = 1.50;  // HIGH: ORIGINAL absolute threshold, preserved
const float MQ2_HIGH_DELTA_V        = 0.90;  // ...or this far above baseline
const float MQ2_HIGH_CLEAR_ABS_V    = 1.35;  // HIGH clears only below both of these
const float MQ2_HIGH_CLEAR_DELTA_V  = 0.75;
const int   MQ2_CONFIRM_SAMPLES     = 4;     // ~1.2 s above threshold to raise a state
const int   MQ2_CLEAR_SAMPLES       = 6;     // ~1.8 s below threshold to lower a state
enum Mq2State { MQ2_WARMING, MQ2_NORMAL, MQ2_WARNING, MQ2_HIGH };
Mq2State mq2State = MQ2_WARMING;
float mq2Filtered = 0, mq2Baseline = 0, mq2Delta = 0; bool mq2FilterInit = false;
int mq2WarnCount = 0, mq2HighCount = 0, mq2ClearCount = 0;

// ==================== HC-SR04 STATE ====================
float hcDist = -1, hcLastValid = -1; unsigned long hcLastValidTime = 0; int hcFailCount = 0;
int hcAngleAtReading = 0; // servo angle associated with the current HC-SR04 reading

int servoAngle = 0, servoDir = 1;

// ==================== LD2410B STATE ====================
bool presenceNow = false, presenceDebounced = false;
uint16_t ld2410MovingDist = 0, ld2410StillDist = 0, ld2410DetectDist = 0;
uint8_t ld2410MovingEnergy = 0, ld2410StillEnergy = 0, ld2410TargetState = 0;
unsigned long ld2410LastFrameTime = 0;
int presenceOnCount = 0, presenceOffCount = 0;
const int PRESENCE_ON_FRAMES = 3;   // consecutive VALID frames of target required to trigger ALERT
const int PRESENCE_OFF_FRAMES = 3;  // consecutive VALID frames of clear required to reset ALERT
const unsigned long LD2410_STALE_MS = 1000; // no valid frame for this long -> treat as stale
bool ld2410NewFrame = false;        // true only for the loop() iteration a new valid frame was parsed
int ld2410OutState = LOW;           // raw digital OUT pin state (informational only)

// ---- HARD detection zone (radius set by LD2410_MAX_RANGE_CM) ----
// Software limit (always enforced) + hardware limit programmed into the module.
// >>> CHANGE THE RADIUS HERE (cm). Module range at 0.20 m/gate: 40..160. Demo standard = 150. <<<
#define LD2410_MAX_RANGE_CM 150     // 1.50 m. Targets beyond this are NEVER treated as detected.
// Hardware gate at 0.20 m/gate, rounded UP so the module never cuts the zone short,
// clamped to the protocol's allowed 2..8 (i.e. 0.40 .. 1.60 m).
#define LD2410_GATE_FINE   ((LD2410_MAX_RANGE_CM + 19) / 20 < 2 ? 2 : ((LD2410_MAX_RANGE_CM + 19) / 20 > 8 ? 8 : (LD2410_MAX_RANGE_CM + 19) / 20))
#define LD2410_GATE_COARSE   2      // protocol minimum; with 0.75 m resolution = 1.50 m
const uint16_t LD2410_NOBODY_DURATION_S = 5; // factory default "no-one duration"
bool ld2410CfgAck = false;          // module answered the config session
bool ld2410GateOk = false;          // max-gate command ACKed
int  ld2410ResCm = 0;               // 20 or 75 cm per gate (0 = unknown)
int  ld2410HwGate = 0;              // gate actually programmed
char ld2410CfgNote[160] = "not configured";
bool ld2410EverReceived = false;    // at least one valid UART frame since boot
bool ld2410MovingInRange = false, ld2410StillInRange = false; // latest frame, per component
bool ld2410FrameInRange = false;    // latest frame: >=1 target component within the zone
bool ld2410FrameOutOfRange = false; // latest frame: target(s) reported, but none within the zone
uint16_t ld2410TargetDistCm = 0;    // nearest in-range distance (from the last in-range frame)
uint8_t ld2410InRangeState = 0;     // bit0 = moving in range, bit1 = still in range

// ==================== BUZZER MUTE STATE ====================
bool buzzerMuted = false, prevPresenceForMute = false;

unsigned long tDht = 0, tGas = 0, tHc = 0, tTof = 0, tMpu = 0, tDash = 0, tServo = 0;

// ==================== MOTOR CONTROL (UNCHANGED) ====================
void motorStop() {
  ledcWrite(L_RPWM, 0); ledcWrite(L_LPWM, 0); ledcWrite(R_RPWM, 0); ledcWrite(R_LPWM, 0);
  motorState = "STOPPED"; commandPWM = 0; leftPWM = 0; rightPWM = 0;
}
void applyMotors(int cmd) {
  commandPWM = cmd;
  leftPWM = constrain((int)(cmd * LEFT_MOTOR_SCALE), -MOTOR_MAX_PWM, MOTOR_MAX_PWM);
  rightPWM = constrain((int)(cmd * RIGHT_MOTOR_SCALE), -MOTOR_MAX_PWM, MOTOR_MAX_PWM);
  if (leftPWM >= 0) { ledcWrite(L_RPWM, leftPWM); ledcWrite(L_LPWM, 0); } else { ledcWrite(L_RPWM, 0); ledcWrite(L_LPWM, -leftPWM); }
  if (rightPWM >= 0) { ledcWrite(R_RPWM, rightPWM); ledcWrite(R_LPWM, 0); } else { ledcWrite(R_RPWM, 0); ledcWrite(R_LPWM, -rightPWM); }
}
void touchDriveWatchdog() { lastDriveCmdTime = millis(); }
void motorForward() { applyMotors(MOTOR_FORWARD_PWM); motorState = "FORWARD"; touchDriveWatchdog(); }
void motorBackward() { applyMotors(-MOTOR_BACKWARD_PWM); motorState = "BACKWARD"; touchDriveWatchdog(); }
void motorTurnLeftFn() {
  leftPWM = -MOTOR_TURN_PWM; rightPWM = MOTOR_TURN_PWM;
  ledcWrite(L_RPWM, 0); ledcWrite(L_LPWM, MOTOR_TURN_PWM); ledcWrite(R_RPWM, MOTOR_TURN_PWM); ledcWrite(R_LPWM, 0);
  motorState = "LEFT"; commandPWM = MOTOR_TURN_PWM; touchDriveWatchdog();
}
void motorTurnRightFn() {
  leftPWM = MOTOR_TURN_PWM; rightPWM = -MOTOR_TURN_PWM;
  ledcWrite(L_RPWM, MOTOR_TURN_PWM); ledcWrite(L_LPWM, 0); ledcWrite(R_RPWM, 0); ledcWrite(R_LPWM, MOTOR_TURN_PWM);
  motorState = "RIGHT"; commandPWM = MOTOR_TURN_PWM; touchDriveWatchdog();
}
void setupMotors() {
  pinMode(L_EN, OUTPUT); pinMode(R_EN, OUTPUT); digitalWrite(L_EN, HIGH); digitalWrite(R_EN, HIGH);
  ledcAttach(L_RPWM, 5000, 8); ledcAttach(L_LPWM, 5000, 8); ledcAttach(R_RPWM, 5000, 8); ledcAttach(R_LPWM, 5000, 8);
  motorStop();
}
void applySpeedMode(String mode) {
  mode.toUpperCase();
  if (mode == "LOW") { MOTOR_FORWARD_PWM = 110; MOTOR_BACKWARD_PWM = 110; MOTOR_TURN_PWM = 100; speedMode = "LOW"; }
  else if (mode == "HIGH") { MOTOR_FORWARD_PWM = 255; MOTOR_BACKWARD_PWM = 255; MOTOR_TURN_PWM = 220; speedMode = "HIGH"; }
  else { MOTOR_FORWARD_PWM = 180; MOTOR_BACKWARD_PWM = 180; MOTOR_TURN_PWM = 160; speedMode = "MEDIUM"; }
}
void applySpeedPWM(int pwm) {
  pwm = constrain(pwm, 40, MOTOR_MAX_PWM);
  MOTOR_FORWARD_PWM = pwm; MOTOR_BACKWARD_PWM = pwm; MOTOR_TURN_PWM = (int)(pwm * 0.9);
  speedMode = "CUSTOM(" + String(pwm) + ")";
}
void driveWatchdogCheck() {
  if (motorState != "STOPPED" && millis() - lastDriveCmdTime > DRIVE_WATCHDOG_MS) motorStop();
}

// ==================== SERVO (UNCHANGED) ====================
void setupServo() { ledcAttach(SERVO_PIN, 50, 16); }
void setServo(int angle) {
  angle = constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE); servoAngle = angle;
  // Map using the CONFIGURABLE servo limits, not hard-coded 0..180 (Requirement 9)
  int us = map(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE, SERVO_MIN_US, SERVO_MAX_US);
  us = constrain(us, SERVO_MIN_US, SERVO_MAX_US);
  ledcWrite(SERVO_PIN, (uint32_t)((us / 20000.0) * 65535));
}
void updateServoSweep() {
  unsigned long now = millis();
  if (now - tServo >= (unsigned long)SERVO_SCAN_INTERVAL_MS) {
    tServo = now;
    int next = servoAngle + servoDir * SERVO_STEP_DEG;
    if (next >= SERVO_MAX_ANGLE) { next = SERVO_MAX_ANGLE; servoDir = -1; }
    if (next <= SERVO_MIN_ANGLE) { next = SERVO_MIN_ANGLE; servoDir = 1; }
    setServo(next);
  }
}

// ==================== DHT22 (unchanged except checksum: compared as 8-bit, per datasheet) ====================
bool readDHT22() {
  uint8_t bits[5] = {0, 0, 0, 0, 0}, cnt = 7, idx = 0;
  pinMode(DHTPIN, OUTPUT); digitalWrite(DHTPIN, LOW); delay(2);
  noInterrupts();
  digitalWrite(DHTPIN, HIGH); delayMicroseconds(40);
  pinMode(DHTPIN, INPUT_PULLUP);
  unsigned long t0 = micros();
  while (digitalRead(DHTPIN) == HIGH) { if (micros() - t0 > 200) { interrupts(); return false; } }
  t0 = micros(); while (digitalRead(DHTPIN) == LOW) { if (micros() - t0 > 200) { interrupts(); return false; } }
  t0 = micros(); while (digitalRead(DHTPIN) == HIGH) { if (micros() - t0 > 200) { interrupts(); return false; } }
  for (int i = 0; i < 40; i++) {
    t0 = micros(); while (digitalRead(DHTPIN) == LOW) { if (micros() - t0 > 200) { interrupts(); return false; } }
    unsigned long t = micros(); t0 = micros();
    while (digitalRead(DHTPIN) == HIGH) { if (micros() - t0 > 200) break; }
    if ((micros() - t) > 40) bits[idx] |= (1 << cnt);
    if (cnt == 0) { cnt = 7; idx++; } else cnt--;
  }
  interrupts();
  for (int i = 0; i < 5; i++) dhtData[i] = bits[i];
  return (uint8_t)(dhtData[0] + dhtData[1] + dhtData[2] + dhtData[3]) == dhtData[4];
}
void updateDHT() {
  bool ok = readDHT22();
  if (ok) {
    dhtHum = ((dhtData[0] << 8) | dhtData[1]) * 0.1;
    dhtTemp = (((dhtData[2] & 0x7F) << 8) | dhtData[3]) * 0.1;
    if (dhtData[2] & 0x80) dhtTemp = -dhtTemp;
    lastValidTemp = dhtTemp; lastValidHum = dhtHum; dhtValidNow = true; dhtLastValidTime = millis();
  } else dhtValidNow = false;
}

// ==================== HC-SR04 (UNCHANGED) ====================
float readHC() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long d = pulseIn(ECHO_PIN, HIGH, 30000);
  if (d == 0) return -1;
  return d * 0.0343 / 2.0;
}
void updateHC() {
  hcDist = readHC();
  hcAngleAtReading = servoAngle; // tie reading to the angle it was taken at
  if (hcDist >= 0) { hcLastValid = hcDist; hcLastValidTime = millis(); hcFailCount = 0; } else hcFailCount++;
}

// ==================== MPU6050 (UNCHANGED) ====================
void readMPU() {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B); Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);
  if (Wire.available() < 14) { mpuReadOK = false; return; }
  ax = Wire.read() << 8 | Wire.read(); ay = Wire.read() << 8 | Wire.read(); az = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();
  gx = Wire.read() << 8 | Wire.read(); gy = Wire.read() << 8 | Wire.read(); gz = Wire.read() << 8 | Wire.read();
  float axg = ax / 16384.0, ayg = ay / 16384.0, azg = az / 16384.0;
  pitch = atan2(ayg, sqrt(axg * axg + azg * azg)) * 180.0 / PI;
  roll = atan2(-axg, azg) * 180.0 / PI;
  lastValidPitch = pitch; lastValidRoll = roll; mpuReadOK = true; mpuLastValidTime = millis();
}

// ==================== GAS ====================
int readAvg(int pin) { long s = 0; for (int i = 0; i < GAS_SAMPLE_COUNT; i++) { s += analogRead(pin); delayMicroseconds(200); } return s / GAS_SAMPLE_COUNT; }
// Original qualitative classifier - still used for MQ-4 and MQ-7 (unchanged behaviour)
const char* gasLevel(float v) { if (v < 0.5) return "NORMAL"; if (v < 1.5) return "ELEVATED"; return "HIGH"; }

const char* mq2StateString() {
  switch (mq2State) {
    case MQ2_WARMING: return "WARMING UP";
    case MQ2_NORMAL:  return "NORMAL";
    case MQ2_WARNING: return "WARNING";
    case MQ2_HIGH:    return "HIGH";
  }
  return "NORMAL";
}

// MQ-2 state machine. Runs every gas sample (300 ms). Non-blocking.
void updateMq2State() {
  unsigned long now = millis();
  if (!mq2FilterInit) { mq2Filtered = mq2V; mq2Baseline = mq2V; mq2FilterInit = true; }
  else mq2Filtered += MQ2_EMA_ALPHA * (mq2V - mq2Filtered);

  // ---- Warm-up: learn the clean-air baseline, never raise a gas warning ----
  if (now - bootTime < MQ2_WARMUP_MS) {
    mq2Baseline += MQ2_BASE_ALPHA_WARMUP * (mq2Filtered - mq2Baseline);
    mq2Delta = mq2Filtered - mq2Baseline;
    mq2State = MQ2_WARMING;
    mq2WarnCount = mq2HighCount = mq2ClearCount = 0;
    return;
  }
  if (mq2State == MQ2_WARMING) {
    if (mq2Baseline > MQ2_BASELINE_MAX_V) mq2Baseline = MQ2_BASELINE_MAX_V;
    if (mq2Baseline < 0) mq2Baseline = 0;
    mq2State = MQ2_NORMAL;
  }

  mq2Delta = mq2Filtered - mq2Baseline;
  bool highCond  = (mq2Filtered >= MQ2_HIGH_ABS_V) || (mq2Delta >= MQ2_HIGH_DELTA_V);
  bool warnCond  = (mq2Delta >= MQ2_WARN_DELTA_V);
  bool highClear = (mq2Filtered < MQ2_HIGH_CLEAR_ABS_V) && (mq2Delta < MQ2_HIGH_CLEAR_DELTA_V);
  bool warnClear = (mq2Delta < MQ2_WARN_CLEAR_DELTA_V);

  switch (mq2State) {
    case MQ2_NORMAL:
      mq2HighCount = highCond ? mq2HighCount + 1 : 0;
      mq2WarnCount = warnCond ? mq2WarnCount + 1 : 0;
      if (mq2HighCount >= MQ2_CONFIRM_SAMPLES) { mq2State = MQ2_HIGH; mq2HighCount = mq2WarnCount = mq2ClearCount = 0; }
      else if (mq2WarnCount >= MQ2_CONFIRM_SAMPLES) { mq2State = MQ2_WARNING; mq2HighCount = mq2WarnCount = mq2ClearCount = 0; }
      break;
    case MQ2_WARNING:
      mq2HighCount  = highCond ? mq2HighCount + 1 : 0;
      mq2ClearCount = warnClear ? mq2ClearCount + 1 : 0;
      if (mq2HighCount >= MQ2_CONFIRM_SAMPLES) { mq2State = MQ2_HIGH; mq2HighCount = mq2WarnCount = mq2ClearCount = 0; }
      else if (mq2ClearCount >= MQ2_CLEAR_SAMPLES) { mq2State = MQ2_NORMAL; mq2HighCount = mq2WarnCount = mq2ClearCount = 0; }
      break;
    case MQ2_HIGH:
      mq2ClearCount = highClear ? mq2ClearCount + 1 : 0;
      if (mq2ClearCount >= MQ2_CLEAR_SAMPLES) {
        mq2State = warnClear ? MQ2_NORMAL : MQ2_WARNING;
        mq2HighCount = mq2WarnCount = mq2ClearCount = 0;
      }
      break;
    default: break;
  }

  // Baseline tracks clean air only (never while a warning is active or rising).
  if (mq2State == MQ2_NORMAL && mq2Delta < MQ2_WARN_CLEAR_DELTA_V) {
    float a = (mq2Delta < 0) ? MQ2_BASE_ALPHA_DOWN : MQ2_BASE_ALPHA_UP;
    mq2Baseline += a * (mq2Filtered - mq2Baseline);
    if (mq2Baseline > MQ2_BASELINE_MAX_V) mq2Baseline = MQ2_BASELINE_MAX_V;
  }
}

unsigned long warmupLeftSec(unsigned long warmMs) {
  unsigned long el = millis() - bootTime;
  return el < warmMs ? (warmMs - el + 999) / 1000 : 0;
}

void updateGas() {
  mq2Raw = readAvg(MQ2_PIN); mq2V = mq2Raw * (3.3 / 4095.0);
  mq4Raw = readAvg(MQ4_PIN); mq4V = mq4Raw * (3.3 / 4095.0);
  mq7Raw = readAvg(MQ7_PIN); mq7V = mq7Raw * (3.3 / 4095.0);
  mq2ZeroCount = (mq2Raw == 0) ? mq2ZeroCount + 1 : 0;
  mq4ZeroCount = (mq4Raw == 0) ? mq4ZeroCount + 1 : 0;
  mq7ZeroCount = (mq7Raw == 0) ? mq7ZeroCount + 1 : 0;
  if (!mq7WarmedUp && millis() - bootTime > MQ7_WARMUP_MS) mq7WarmedUp = true; // non-blocking warm-up
  if (!mq4WarmedUp && millis() - bootTime > MQ4_WARMUP_MS) mq4WarmedUp = true; // non-blocking warm-up
  updateMq2State();
}
String mqErrorFlag(int zeroCount) { return (zeroCount > 200) ? "CHECK WIRING (stuck at 0 for 1+ min)" : "OK"; }

// ==================== I2C / VL53L0X ====================
bool i2cAddrPresent(byte addr) { Wire.beginTransmission(addr); return (Wire.endTransmission() == 0); }
void scanI2C() {
  Serial.println("[I2C SCAN]");
  mpuFound = i2cAddrPresent(MPU_ADDR); vl53DeviceFound = i2cAddrPresent(VL53_ADDR);
  Serial.print("  0x68 (MPU6050): "); Serial.println(mpuFound ? "FOUND" : "NOT FOUND");
  Serial.print("  0x29 (VL53L0X): "); Serial.println(vl53DeviceFound ? "FOUND" : "NOT FOUND");
}
void resetVL53() { pinMode(VL53L0X_XSHUT, OUTPUT); digitalWrite(VL53L0X_XSHUT, LOW); delay(20); digitalWrite(VL53L0X_XSHUT, HIGH); delay(20); }

// Recompute the VL53 state machine (Requirement 5). One temporary invalid range
// must NEVER be reported as INIT FAILED - only a genuine failed lox.begin() call
// (with no prior successful init) is INIT FAILED. Repeated post-init failures
// become RECOVERING, not INIT FAILED.
void recomputeVl53State() {
  if (!vl53DeviceFound) { vl53CurrentState = VL53_NOT_FOUND; return; }
  if (!vl53InitOK) {
    vl53CurrentState = vl53EverInitialized ? VL53_RECOVERING : VL53_INIT_FAILED;
    return;
  }
  if (vl53ConsecutiveFails == 0) { vl53CurrentState = VL53_VALID; return; }
  if (vl53ConsecutiveFails <= VL53_FAIL_LIMIT) { vl53CurrentState = VL53_STALE; return; }
  vl53CurrentState = VL53_RECOVERING;
}

void tryInitVL53() {
  resetVL53();
  Wire.begin(I2C_SDA, I2C_SCL); Wire.setClock(100000);
  vl53DeviceFound = i2cAddrPresent(VL53_ADDR);
  if (!vl53DeviceFound) { vl53InitOK = false; recomputeVl53State(); return; }
  // Address-only overload used deliberately: it is present in both old and new
  // Adafruit_VL53L0X releases.
  vl53InitOK = lox.begin(VL53_ADDR);
  if (vl53InitOK) { vl53EverInitialized = true; vl53ConsecutiveFails = 0; }
  recomputeVl53State();
}
void updateVL53() {
  if (!vl53InitOK) {
    if (millis() - tVl53Retry >= VL53_RETRY_INTERVAL) { tVl53Retry = millis(); tryInitVL53(); }
    recomputeVl53State();
    return;
  }
  VL53L0X_RangingMeasurementData_t m;
  memset(&m, 0, sizeof(m));                       // never evaluate an uninitialised struct
  VL53L0X_Error err = lox.rangingTest(&m, false);

  if (err != VL53L0X_ERROR_NONE) {
    // Genuine communication/ranging failure -> counts toward recovery (unchanged policy)
    vl53ConsecutiveFails++;
    if (vl53ConsecutiveFails > VL53_FAIL_LIMIT) { vl53InitOK = false; tVl53Retry = millis(); }
  } else if (m.RangeStatus == 4 || m.RangeMilliMeter > VL53_MAX_PLAUSIBLE_MM) {
    // Sensor answered correctly but no object inside its ~2 m range.
    // This is NOT a sensor fault (previously it forced reinit cycles in open space).
    vl53NoObject = true; vl53ConsecutiveFails = 0;
    vl53History[0] = vl53History[1] = vl53History[2] = -1; vl53HistIdx = 0;
  } else {
    vl53NoObject = false;
    vl53RawMM = m.RangeMilliMeter;
    vl53History[vl53HistIdx % 3] = vl53RawMM; vl53HistIdx++;
    int cnt = 0, sum = 0;
    for (int i = 0; i < 3; i++) if (vl53History[i] >= 0) { sum += vl53History[i]; cnt++; }
    vl53FilteredMM = cnt > 0 ? sum / cnt : vl53RawMM;
    vl53LastValidMM = vl53FilteredMM; vl53LastValidTime = millis(); vl53ConsecutiveFails = 0;
  }
  recomputeVl53State();
}
String vl53StatusString() {
  switch (vl53CurrentState) {
    case VL53_NOT_FOUND:   return "NOT FOUND";
    case VL53_INIT_FAILED: return "INIT FAILED";
    case VL53_VALID:       return vl53NoObject ? "NO OBJECT" : "VALID";
    case VL53_STALE:       return "STALE";
    case VL53_RECOVERING:  return "RECOVERING";
  }
  return "UNKNOWN";
}

// ==================== LD2410B CONFIGURATION ====================
// Command/ACK frames per HLK-LD2410B Serial Communication Protocol V1.07:
//   send: FD FC FB FA | len(2) | cmd(2) | value | 04 03 02 01
//   ACK : FD FC FB FA | len(2) | (cmd|0x0100)(2) | status(2, 0=ok) | data | 04 03 02 01
void sendLD2410Command(uint16_t cmd, uint8_t* data, uint8_t dataLen) {
  uint8_t header[4] = {0xFD, 0xFC, 0xFB, 0xFA};
  uint8_t footer[4] = {0x04, 0x03, 0x02, 0x01};
  uint16_t len = 2 + dataLen;
  LD2410Serial.write(header, 4);
  LD2410Serial.write((uint8_t)(len & 0xFF)); LD2410Serial.write((uint8_t)(len >> 8));
  LD2410Serial.write((uint8_t)(cmd & 0xFF)); LD2410Serial.write((uint8_t)(cmd >> 8));
  if (dataLen > 0) LD2410Serial.write(data, dataLen);
  LD2410Serial.write(footer, 4);
}
// Wait for the ACK of 'cmd'. Data frames (F4 F3 F2 F1...) are skipped. Setup-only.
bool waitLD2410Ack(uint16_t cmd, uint8_t* val, uint8_t valMax, unsigned long timeoutMs) {
  static const uint8_t H[4] = {0xFD, 0xFC, 0xFB, 0xFA};
  uint8_t buf[64]; uint16_t idx = 0, len = 0;
  unsigned long t0 = millis();
  while (millis() - t0 < timeoutMs) {
    if (!LD2410Serial.available()) { delay(1); continue; }
    uint8_t b = LD2410Serial.read();
    if (idx < 4) { if (b == H[idx]) buf[idx++] = b; else idx = (b == H[0]) ? 1 : 0; continue; }
    buf[idx++] = b;
    if (idx == 6) {
      len = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);
      if (len < 4 || len > 50) idx = 0;           // 6 + 50 + 4 = 60 < sizeof(buf)
      continue;
    }
    if (idx > 6 && idx == (uint16_t)(6 + len + 4)) {
      bool ftr = buf[idx - 4] == 0x04 && buf[idx - 3] == 0x03 && buf[idx - 2] == 0x02 && buf[idx - 1] == 0x01;
      uint16_t word = (uint16_t)buf[6] | ((uint16_t)buf[7] << 8);
      uint16_t status = (uint16_t)buf[8] | ((uint16_t)buf[9] << 8);
      if (ftr && word == (uint16_t)(cmd | 0x0100)) {
        for (uint8_t i = 0; i < valMax && (uint16_t)(10 + i) < (uint16_t)(6 + len); i++) val[i] = buf[10 + i];
        return status == 0;
      }
      idx = 0;
    }
  }
  return false;
}
bool ld2410Cmd(uint16_t cmd, uint8_t* data, uint8_t dataLen, uint8_t* val = nullptr, uint8_t valMax = 0) {
  while (LD2410Serial.available()) LD2410Serial.read();
  sendLD2410Command(cmd, data, dataLen);
  return waitLD2410Ack(cmd, val, valMax, 400);
}
int queryLD2410Resolution() {                     // 0x00AB -> 0x0000 = 0.75 m, 0x0001 = 0.20 m
  uint8_t v[2] = {0xFF, 0xFF};
  if (!ld2410Cmd(0x00AB, nullptr, 0, v, 2)) return 0;
  if (v[0] == 0x01 && v[1] == 0x00) return 20;
  if (v[0] == 0x00 && v[1] == 0x00) return 75;
  return 0;
}
// Runs once in setup(). Only restarts the module (and writes its flash) when the
// resolution is not already 0.20 m, so normal boots do not restart it.
void configureLD2410() {
  uint8_t en[2] = {0x01, 0x00};
  ld2410CfgAck = ld2410Cmd(0x00FF, en, 2);
  if (!ld2410CfgAck) {
    snprintf(ld2410CfgNote, sizeof(ld2410CfgNote), "no ACK from LD2410B - check ESP32 TX2 (GPIO17) -> radar RX wiring; software %.2f m limit still active", LD2410_MAX_RANGE_CM / 100.0);
    return;
  }
  ld2410ResCm = queryLD2410Resolution();
  if (ld2410ResCm != 20) {
    uint8_t r[2] = {0x01, 0x00};                  // 0x0001 = 0.20 m per gate
    if (ld2410Cmd(0x00AA, r, 2)) {
      ld2410Cmd(0x00A3, nullptr, 0);              // restart: resolution applies after reboot
      delay(2500);
      ld2410Cmd(0x00FF, en, 2);
      ld2410ResCm = queryLD2410Resolution();
    }
  }
  uint8_t gate = (ld2410ResCm == 20) ? LD2410_GATE_FINE : LD2410_GATE_COARSE;
  uint8_t p[18] = {
    0x00, 0x00, gate, 0, 0, 0,                     // max moving gate
    0x01, 0x00, gate, 0, 0, 0,                     // max still gate
    0x02, 0x00, (uint8_t)(LD2410_NOBODY_DURATION_S & 0xFF), (uint8_t)(LD2410_NOBODY_DURATION_S >> 8), 0, 0
  };
  ld2410GateOk = ld2410Cmd(0x0060, p, 18);
  if (ld2410GateOk) ld2410HwGate = gate;
  ld2410Cmd(0x0062, nullptr, 0);                  // engineering mode (as before)
  ld2410Cmd(0x00FE, nullptr, 0);                  // end configuration
  while (LD2410Serial.available()) LD2410Serial.read();
  float sw = LD2410_MAX_RANGE_CM / 100.0;
  if (ld2410GateOk && ld2410ResCm == 20)
    snprintf(ld2410CfgNote, sizeof(ld2410CfgNote), "hardware limit %.2f m (0.20 m x gate %d) + software %.2f m", gate * 0.20, gate, sw);
  else if (ld2410GateOk)
    snprintf(ld2410CfgNote, sizeof(ld2410CfgNote), "module firmware kept 0.75 m resolution: hardware limit 1.50 m, software limit %.2f m enforces the zone", sw);
  else
    snprintf(ld2410CfgNote, sizeof(ld2410CfgNote), "max-gate command not accepted; software %.2f m limit enforces the zone", sw);
}

// ---- HARD LIMIT (LD2410_MAX_RANGE_CM): evaluate the freshly parsed frame ----
// Target state bits: bit0 = moving target, bit1 = still target.
// A component only counts if its bit is set AND its reported distance is
// 1..LD2410_MAX_RANGE_CM. Anything beyond the limit is ignored for detection.
void evaluateLD2410Range() {
  bool movingBit = (ld2410TargetState & 0x01) != 0;
  bool stillBit  = (ld2410TargetState & 0x02) != 0;
  ld2410MovingInRange = movingBit && ld2410MovingDist > 0 && ld2410MovingDist <= LD2410_MAX_RANGE_CM;
  ld2410StillInRange  = stillBit  && ld2410StillDist  > 0 && ld2410StillDist  <= LD2410_MAX_RANGE_CM;
  ld2410FrameInRange  = ld2410MovingInRange || ld2410StillInRange;
  ld2410FrameOutOfRange = (ld2410TargetState != 0) && !ld2410FrameInRange;
  if (ld2410FrameInRange) {
    uint16_t d = 0xFFFF;
    if (ld2410MovingInRange) d = ld2410MovingDist;
    if (ld2410StillInRange && ld2410StillDist < d) d = ld2410StillDist;
    ld2410TargetDistCm = d;
    ld2410InRangeState = (ld2410MovingInRange ? 1 : 0) | (ld2410StillInRange ? 2 : 0);
  }
}

// ---- LD2410 data-frame parser ----
// Frame layout: F4 F3 F2 F1 | lenLo lenHi | type AA <payload...> | F8 F7 F6 F5
#define LD2410_BUF_SIZE 64
#define LD2410_MAX_FRAME_LEN 40 // sanity cap well above the largest expected engineering-mode payload

void readLD2410Frame() {
  static uint8_t buf[LD2410_BUF_SIZE];
  static uint16_t idx = 0;
  static const uint8_t HDR[4] = {0xF4, 0xF3, 0xF2, 0xF1};
  static const uint8_t FTR[4] = {0xF8, 0xF7, 0xF6, 0xF5};

  ld2410NewFrame = false;

  while (LD2410Serial.available()) {
    uint8_t b = LD2410Serial.read();

    if (idx < 4) {
      if (b == HDR[idx]) {
        buf[idx++] = b;
      } else {
        idx = (b == HDR[0]) ? 1 : 0;
        if (idx == 1) buf[0] = b;
      }
      continue;
    }

    if (idx >= LD2410_BUF_SIZE) { idx = 0; continue; }
    buf[idx++] = b;

    if (idx == 6) {
      uint16_t frameLen = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);
      if (frameLen == 0 || frameLen > LD2410_MAX_FRAME_LEN) {
        idx = 0;
        continue;
      }
    }

    if (idx >= 6) {
      uint16_t frameLen = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);
      uint16_t totalLen = (uint16_t)(4 + 2 + frameLen + 4);

      if (totalLen > LD2410_BUF_SIZE) { idx = 0; continue; }

      if (idx >= totalLen) {
        bool footerOK = (buf[totalLen - 4] == FTR[0] && buf[totalLen - 3] == FTR[1] &&
                          buf[totalLen - 2] == FTR[2] && buf[totalLen - 1] == FTR[3]);
        bool longEnough = (totalLen >= 21);
        // Data type 0x01 (engineering) / 0x02 (basic), followed by head byte 0xAA
        bool headOK = (buf[7] == 0xAA) && (buf[6] == 0x01 || buf[6] == 0x02);

        if (footerOK && longEnough && headOK) {
          ld2410TargetState  = buf[8];
          ld2410MovingDist   = buf[9]  | (buf[10] << 8);
          ld2410MovingEnergy = buf[11];
          ld2410StillDist    = buf[12] | (buf[13] << 8);
          ld2410StillEnergy  = buf[14];
          ld2410DetectDist   = buf[15] | (buf[16] << 8);
          ld2410LastFrameTime = millis();
          ld2410EverReceived = true;
          evaluateLD2410Range();   // apply the hard range limit to this frame
          ld2410NewFrame = true;
        }
        idx = 0;
        if (ld2410NewFrame) return; // one valid frame per call; stay non-blocking
      }
    }
  }
}

// ==================== PRESENCE DEBOUNCE + BUZZER ====================
// Debounce counters advance ONLY on a freshly validated frame, and ONLY an
// in-range (<= 1 m) target counts as "target". Stale UART clears presence.
void updatePresence() {
  bool stale = (millis() - ld2410LastFrameTime) >= LD2410_STALE_MS;

  if (ld2410NewFrame) {
    bool rawTarget = ld2410FrameInRange;   // HARD range gate - out-of-range target == no target
    if (rawTarget) {
      presenceOnCount++; presenceOffCount = 0;
      if (presenceOnCount >= PRESENCE_ON_FRAMES) presenceDebounced = true;
    } else {
      presenceOffCount++; presenceOnCount = 0;
      if (presenceOffCount >= PRESENCE_OFF_FRAMES) presenceDebounced = false;
    }
  }

  if (stale) {
    presenceDebounced = false;
    presenceOnCount = 0;
    presenceOffCount = 0;
    ld2410FrameInRange = false; ld2410FrameOutOfRange = false;
    ld2410MovingInRange = false; ld2410StillInRange = false;
  }

  presenceNow = presenceDebounced;
}
void updateBuzzer() {
  // Buzzer rule: ON only while a target is detected (<= 1 m) and not silenced
  // from the dashboard. AREA CLEAR / radar offline -> always OFF.
  digitalWrite(BUZZER_PIN, (presenceNow && !buzzerMuted) ? HIGH : LOW);
  if (!presenceNow) buzzerMuted = false; // silence only applies to the current target
  prevPresenceForMute = presenceNow;
}

// ==================== WEB DASHBOARD ====================
const char htmlPage[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1">
<meta name="theme-color" content="#05080d">
<title>Fire Rescue Rover</title>
<style>
:root{
  --bg:#05080d;--glass:rgba(16,26,38,.72);--glass2:rgba(24,36,52,.6);--line:rgba(120,170,220,.13);
  --text:#e6eef6;--sub:#7d91a4;--mono:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;
  --cyan:#22d3ee;--blue:#3b82f6;--ok:#34d399;--warn:#fbbf24;--orange:#fb923c;--err:#f43f5e;--gray:#64748b;--warm:#a78bfa;
  --c-dht:#22d3ee;--c-mq2:#fb923c;--c-mq4:#a78bfa;--c-mq7:#facc15;--c-hc:#4ade80;--c-vl:#e879f9;--c-mpu:#818cf8;--c-ld:#fb7185;--c-motor:#7fa8c9;
}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
html,body{margin:0}
body{background:var(--bg);color:var(--text);font-family:-apple-system,'Segoe UI',Roboto,Arial,sans-serif;font-size:14px;min-height:100vh}
body::before{content:"";position:fixed;inset:0;z-index:-1;pointer-events:none;
  background:radial-gradient(900px 500px at 0% -10%,rgba(34,211,238,.10),transparent 60%),
             radial-gradient(800px 480px at 110% 0%,rgba(244,63,94,.08),transparent 60%),
             linear-gradient(rgba(120,170,220,.035) 1px,transparent 1px) 0 0/100% 28px}
.wrap{max-width:1120px;margin:0 auto;padding:12px 12px 28px}
.glass{background:linear-gradient(160deg,var(--glass),var(--glass2));border:1px solid var(--line);border-radius:18px;box-shadow:0 10px 30px rgba(0,0,0,.35),inset 0 1px 0 rgba(255,255,255,.04)}
.sec{font-size:10.5px;letter-spacing:2px;text-transform:uppercase;color:var(--sub);margin:20px 4px 9px;display:flex;align-items:center;gap:10px}
.sec::after{content:"";flex:1;height:1px;background:linear-gradient(90deg,var(--line),transparent)}
.mono{font-family:var(--mono)}

/* header */
.top{display:flex;justify-content:space-between;align-items:center;gap:10px;padding:4px 2px 12px}
.brand{display:flex;align-items:center;gap:11px;min-width:0}
.logo{width:42px;height:42px;border-radius:13px;display:grid;place-items:center;font-size:22px;flex:none;
  background:radial-gradient(circle at 30% 25%,rgba(251,146,60,.45),rgba(244,63,94,.18) 60%,transparent);border:1px solid rgba(251,146,60,.35);box-shadow:0 0 18px rgba(251,146,60,.25)}
h1{margin:0;font-size:16px;letter-spacing:1.6px;font-weight:800;white-space:nowrap}
.tag{font-size:10.5px;color:var(--sub);letter-spacing:.4px;margin-top:2px}
.link{display:flex;align-items:center;gap:7px;font-size:10.5px;font-weight:700;letter-spacing:1.2px;padding:6px 10px;border-radius:20px;border:1px solid var(--line);background:rgba(10,16,24,.7);flex:none}
.ldot{width:8px;height:8px;border-radius:50%;background:var(--gray)}
.link[data-s=ok]{color:var(--ok)} .link[data-s=ok] .ldot{background:var(--ok)}
.ldot.on{animation:pulse .9s ease-out}
.link[data-s=lost]{color:var(--err);border-color:rgba(244,63,94,.5)} .link[data-s=lost] .ldot{background:var(--err);animation:blink .8s infinite}
@keyframes pulse{0%{box-shadow:0 0 0 0 rgba(52,211,153,.6)}70%{box-shadow:0 0 0 8px rgba(52,211,153,0)}100%{box-shadow:0 0 0 0 rgba(52,211,153,0)}}
@keyframes blink{50%{opacity:.25}}

/* status */
.statusPanel{display:grid;grid-template-columns:auto 1fr;gap:14px;padding:14px;align-items:center}
.ring{--p:0;--rc:var(--gray);width:92px;height:92px;border-radius:50%;display:grid;place-items:center;
  background:conic-gradient(var(--rc) calc(var(--p)*1%),rgba(255,255,255,.07) 0);transition:--p .6s}
.ring[data-s=ok]{--rc:var(--ok)} .ring[data-s=warn]{--rc:var(--warn)} .ring[data-s=crit]{--rc:var(--err)}
.ringIn{width:74px;height:74px;border-radius:50%;background:#0a121b;display:flex;flex-direction:column;align-items:center;justify-content:center;border:1px solid var(--line)}
.ringIn b{font-size:20px;font-family:var(--mono)} .ringIn small{font-size:7.5px;letter-spacing:.6px;color:var(--sub)}
.sysPill{display:inline-flex;align-items:center;gap:8px;padding:7px 13px;border-radius:12px;font-weight:800;font-size:13.5px;letter-spacing:.8px;border:1px solid var(--line);background:rgba(100,116,139,.12)}
.sysPill[data-s=ok]{color:var(--ok);background:rgba(52,211,153,.10);border-color:rgba(52,211,153,.35)}
.sysPill[data-s=warn]{color:var(--warn);background:rgba(251,191,36,.10);border-color:rgba(251,191,36,.4)}
.sysPill[data-s=crit]{color:var(--err);background:rgba(244,63,94,.12);border-color:rgba(244,63,94,.5);animation:critGlow 1.4s infinite}
@keyframes critGlow{50%{box-shadow:0 0 16px rgba(244,63,94,.35)}}
.counts{display:grid;grid-template-columns:repeat(4,1fr);gap:6px;margin-top:10px}
.counts div{background:rgba(255,255,255,.03);border:1px solid var(--line);border-radius:10px;padding:6px 4px;text-align:center}
.counts b{display:block;font-family:var(--mono);font-size:15px;color:var(--ok)}
.counts .cw b{color:var(--warn)} .counts .ce b{color:var(--err)} .counts .cm b{color:var(--warm)}
.counts span{font-size:8.5px;letter-spacing:1px;color:var(--sub)}
.meta{font-size:10.5px;color:var(--sub);margin-top:8px;letter-spacing:.6px} .meta b{color:var(--text);font-family:var(--mono)}
.reasons{grid-column:1/-1;list-style:none;margin:0;padding:0;display:flex;flex-direction:column;gap:5px}
.reasons li{font-size:12px;padding:6px 10px 6px 26px;border-radius:9px;position:relative;background:rgba(255,255,255,.025);border:1px solid var(--line)}
.reasons li::before{content:"";position:absolute;left:10px;top:50%;margin-top:-4px;width:8px;height:8px;border-radius:50%;background:var(--gray)}
.reasons .r-c{color:#fecdd3} .reasons .r-c::before{background:var(--err);box-shadow:0 0 8px var(--err)}
.reasons .r-w{color:#fde68a} .reasons .r-w::before{background:var(--warn)}
.reasons .r-i{color:#c7d2de} .reasons .r-i::before{background:var(--warm)}
.reasons .r-ok{color:#a7f3d0} .reasons .r-ok::before{background:var(--ok)}

#enableBtn{width:100%;margin:10px 0 0;padding:11px;border-radius:13px;border:1px solid var(--line);background:rgba(20,30,44,.7);color:var(--text);font-size:13px;font-weight:600}
#enableBtn.on{border-color:rgba(52,211,153,.5);color:var(--ok)}
/* wireless buzzer */
.buzzPanel{display:flex;align-items:center;justify-content:space-between;gap:14px;padding:14px 16px;transition:border-color .3s,box-shadow .3s}
.buzzPanel[data-s=manual]{border-color:rgba(251,146,60,.55);box-shadow:0 0 22px rgba(251,146,60,.18)}
.buzzPanel[data-s=alarm]{border-color:rgba(244,63,94,.6);box-shadow:0 0 22px rgba(244,63,94,.22)}
.bzInfo{min-width:0}
.bzState{display:flex;align-items:center;gap:8px;margin-top:8px;font-size:11.5px;color:#c3cfdb;flex-wrap:wrap}
.switch{position:relative;flex:none;width:104px;height:48px;border-radius:26px;border:1px solid rgba(148,163,184,.3);background:linear-gradient(180deg,rgba(51,65,85,.55),rgba(30,41,59,.7));box-shadow:inset 0 2px 6px rgba(0,0,0,.45);cursor:pointer;padding:0;transition:background .25s,border-color .25s,box-shadow .25s}
.switch .knob{position:absolute;top:5px;left:5px;width:36px;height:36px;border-radius:50%;background:linear-gradient(180deg,#cbd5e1,#94a3b8);box-shadow:0 3px 8px rgba(0,0,0,.45);transition:left .25s cubic-bezier(.3,1.4,.5,1),background .25s}
.switch .swl{position:absolute;right:14px;top:50%;transform:translateY(-50%);font-size:11px;font-weight:800;letter-spacing:1.2px;color:#94a3b8}
.switch.on{border-color:rgba(251,146,60,.8);background:linear-gradient(90deg,rgba(244,63,94,.55),rgba(251,146,60,.55));box-shadow:0 0 18px rgba(251,146,60,.45),inset 0 1px 0 rgba(255,255,255,.15)}
.switch.on .knob{left:63px;background:linear-gradient(180deg,#fff7ed,#fdba74);box-shadow:0 0 12px rgba(251,146,60,.9)}
.switch.on .swl{right:auto;left:16px;color:#fff}
.switch.pending{opacity:.6}
.switch:disabled{opacity:.4;cursor:not-allowed}

/* badges */
.badge{display:inline-flex;align-items:center;gap:6px;padding:3px 9px;border-radius:20px;font-size:10.5px;font-weight:800;letter-spacing:.5px;white-space:nowrap;background:rgba(100,116,139,.15);color:var(--gray)}
.badge i{width:7px;height:7px;border-radius:50%;background:currentColor}
.b-ok{background:rgba(52,211,153,.12);color:var(--ok)}
.b-warn{background:rgba(251,191,36,.12);color:var(--warn)}
.b-err{background:rgba(244,63,94,.14);color:var(--err)}
.b-recov{background:rgba(251,146,60,.13);color:var(--orange)}
.b-warm{background:rgba(167,139,250,.14);color:var(--warm)}
.b-cyan{background:rgba(34,211,238,.12);color:var(--cyan)}
.b-gray{background:rgba(100,116,139,.15);color:var(--gray)}
.b-err i,.b-warm i{animation:blink 1.2s infinite}

/* rescue */
.rescue{padding:15px;position:relative;overflow:hidden;transition:border-color .3s,background .3s}
.rescue.clear{border-color:rgba(52,211,153,.35)}
.rescue.verify{border-color:rgba(251,191,36,.5)}
.rescue.offline{border-color:rgba(100,116,139,.5)}
.rescue.alert{border-color:var(--err);background:linear-gradient(160deg,rgba(80,14,28,.75),rgba(30,12,20,.7));animation:alertRing 1.1s infinite}
@keyframes alertRing{0%{box-shadow:0 0 0 0 rgba(244,63,94,.45)}70%{box-shadow:0 0 0 12px rgba(244,63,94,0)}100%{box-shadow:0 0 0 0 rgba(244,63,94,0)}}
.rTitle{font-size:19px;font-weight:800;letter-spacing:.8px}
.rescue.alert .rTitle{color:#ff8fa3}.rescue.clear .rTitle{color:var(--ok)}.rescue.verify .rTitle{color:var(--warn)}.rescue.offline .rTitle{color:#cbd5e1}
.rSub{font-size:11.5px;color:var(--sub);margin-top:3px}
.rGrid{display:grid;grid-template-columns:1fr;gap:12px;margin-top:12px}
@media(min-width:640px){.rGrid{grid-template-columns:minmax(260px,1fr) 1.1fr;align-items:center}}
.radar{position:relative}
.radar svg{width:100%;height:auto;display:block}
.radar .zone{fill:rgba(34,211,238,.05);stroke:rgba(34,211,238,.45);stroke-width:.8}
.radar .rg{fill:none;stroke:rgba(34,211,238,.22);stroke-width:.6;stroke-dasharray:2 2}
.radar .sp{stroke:rgba(34,211,238,.12);stroke-width:.5}
.radar text{fill:var(--sub);font-size:6px;font-family:var(--mono)}
.radar .sweep{transform-box:view-box;transform-origin:100px 105px;animation:sweep 3.4s ease-in-out infinite alternate}
.radar .sweep path{fill:rgba(34,211,238,.18)} .radar .sweep line{stroke:rgba(34,211,238,.8);stroke-width:.8}
@keyframes sweep{from{transform:rotate(-90deg)}to{transform:rotate(75deg)}}
.radar[data-s=alert] .sweep{animation-duration:1.3s}
.radar[data-s=offline] .sweep{animation:none;opacity:0}
.radar[data-s=offline] .zone{stroke:rgba(100,116,139,.5);fill:rgba(100,116,139,.04)}
.radar[data-s=alert] .zone{stroke:rgba(244,63,94,.6);fill:rgba(244,63,94,.06)}
.radar[data-s=alert] .sweep path{fill:rgba(244,63,94,.16)} .radar[data-s=alert] .sweep line{stroke:rgba(244,63,94,.8)}
.radar .me{fill:var(--cyan)}
.blip{fill:var(--err)} .radar[data-s=verify] .blip{fill:var(--warn)}
.bRing{fill:none;stroke:var(--err);stroke-width:1;transform-box:fill-box;transform-origin:center;animation:ping 1.2s infinite}
.radar[data-s=verify] .bRing{stroke:var(--warn)}
@keyframes ping{from{transform:scale(1);opacity:.9}to{transform:scale(3.2);opacity:0}}
.far{fill:none;stroke:var(--gray);stroke-width:1;stroke-dasharray:1.5 1.5}
.rcap{font-size:9.5px;color:var(--sub);text-align:center;margin-top:3px;letter-spacing:.3px}
.row2{display:flex;justify-content:space-between;gap:10px;font-size:12.5px;padding:6px 0;border-top:1px dashed var(--line);color:#b9c6d2}
.row2:first-child{border-top:none}
.row2 b{color:var(--text);text-align:right}
.row2.pair span b{margin-left:6px}
.prox{margin-top:10px;font-size:11.5px;color:var(--sub);line-height:1.5}
.prox:empty{display:none}
.prox b{color:var(--text)}

/* drive */
.ctrlPanel{padding:14px}
.speedRow{display:flex;gap:6px;margin-bottom:10px}
.speedBtn{flex:1;padding:9px;border-radius:10px;border:1px solid var(--line);background:rgba(255,255,255,.03);color:var(--sub);font-size:11.5px;font-weight:800;letter-spacing:1px}
.speedBtn.active{color:var(--text);border-color:var(--c-motor);background:rgba(127,168,201,.16);box-shadow:0 0 12px rgba(127,168,201,.2)}
.padWrap{display:flex;flex-direction:column;align-items:center;gap:9px;margin:8px 0 12px}
.padRow{display:flex;gap:10px}
button.pad{width:80px;height:80px;font-size:22px;border-radius:18px;background:linear-gradient(180deg,#223142,#131c26);color:var(--text);font-weight:700;user-select:none;-webkit-user-select:none;touch-action:none;border:1px solid rgba(120,170,220,.2);box-shadow:0 6px 16px rgba(0,0,0,.4),inset 0 1px 0 rgba(255,255,255,.06)}
button.pad:active,button.pad.kbon{filter:brightness(1.35);box-shadow:0 0 18px rgba(34,211,238,.35);border-color:rgba(34,211,238,.6)}
.kbd{margin-top:8px;font-size:11px;color:var(--sub);text-align:center;line-height:1.9}
.kbd kbd{display:inline-block;min-width:22px;padding:1px 6px;margin:0 1px;border-radius:6px;border:1px solid rgba(120,170,220,.3);background:rgba(255,255,255,.05);color:var(--text);font-family:var(--mono);font-size:10.5px;box-shadow:0 2px 0 rgba(0,0,0,.4)}
@media (hover:none){.kbd{display:none}}
#stopBtn{background:linear-gradient(180deg,#4a1a22,#26090f);border-color:rgba(244,63,94,.55);color:#ff9aa9}
.driveStats{display:grid;grid-template-columns:repeat(2,1fr);gap:6px}
.driveStats div{background:rgba(255,255,255,.03);border:1px solid var(--line);border-radius:10px;padding:7px 9px;font-size:10px;letter-spacing:.8px;color:var(--sub)}
.driveStats b{display:block;color:var(--text);font-size:13px;font-family:var(--mono);letter-spacing:0;margin-top:2px}
@media(min-width:520px){.driveStats{grid-template-columns:repeat(4,1fr)}}
.dnote{font-size:10px;color:var(--sub);text-align:center;margin-top:8px}

/* cards */
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(265px,1fr));gap:10px}
.card{--acc:var(--gray);position:relative;padding:13px 14px 12px 17px;transition:border-color .3s}
.card::before{content:"";position:absolute;left:0;top:14px;bottom:14px;width:3px;border-radius:0 3px 3px 0;background:var(--acc);box-shadow:0 0 10px var(--acc)}
.card[data-s=ok]{--acc:var(--ok)} .card[data-s=warn]{--acc:var(--warn)} .card[data-s=err]{--acc:var(--err);border-color:rgba(244,63,94,.35)}
.card[data-s=warm]{--acc:var(--warm)} .card[data-s=info]{--acc:var(--cyan)} .card[data-s=idle]{--acc:var(--gray)}
.ch{display:flex;justify-content:space-between;align-items:flex-start;gap:8px}
.ct{font-size:12.5px;font-weight:800;letter-spacing:.5px;color:var(--c)}
.cs{font-size:10px;color:var(--sub);margin-top:2px}
.big{font-size:25px;font-weight:800;font-family:var(--mono);margin:8px 0 6px;letter-spacing:-.3px}
.big small{font-size:12px;color:var(--sub);margin-left:3px;font-weight:600}
.kv{display:flex;justify-content:space-between;font-size:11.5px;color:var(--sub);padding:3px 0}
.kv b{color:#d5dee7;font-family:var(--mono);font-weight:600}
.meter{position:relative;height:7px;border-radius:6px;background:rgba(255,255,255,.06);overflow:hidden;margin:5px 0}
.fill{height:100%;width:0;border-radius:6px;background:var(--cyan);transition:width .6s ease,background .3s}
.heat{background:linear-gradient(90deg,#22d3ee,#fbbf24,#f43f5e)} .hum{background:linear-gradient(90deg,#3b82f6,#22d3ee)}
.g-ok{background:var(--ok)} .g-warn{background:var(--warn)} .g-err{background:var(--err)} .g-warm{background:var(--warm)}
.gauge{position:relative;height:10px;border-radius:6px;background:rgba(255,255,255,.06);margin:6px 0 2px}
.gauge .fill{position:absolute;left:0;top:0}
.mk{position:absolute;top:-3px;bottom:-3px;width:2px;border-radius:2px;background:#cbd5e1;transition:left .6s}
.mk.base{background:var(--cyan)} .mk.thr{background:var(--warn)} .mk.hi{background:var(--err)}
.gl{display:flex;justify-content:space-between;font-size:9px;color:var(--sub);font-family:var(--mono);margin-bottom:4px}
.legend{display:flex;gap:10px;flex-wrap:wrap;font-size:9.5px;color:var(--sub);margin-bottom:4px}
.legend i{display:inline-block;width:8px;height:3px;border-radius:2px;margin-right:4px;vertical-align:middle}
.foot{display:flex;justify-content:space-between;align-items:center;font-size:10px;color:var(--sub);margin-top:6px}
.act{width:7px;height:7px;border-radius:50%;background:rgba(34,211,238,.25)}
.act.on{animation:act .7s ease-out}
@keyframes act{0%{background:var(--cyan);box-shadow:0 0 8px var(--cyan)}100%{background:rgba(34,211,238,.25);box-shadow:none}}
.problem{font-size:11px;color:var(--warn);margin-top:7px;padding-top:7px;border-top:1px dashed var(--line)}
.problem:empty{display:none}
.warmBox{font-size:10.5px;color:var(--warm);margin-top:4px}
.warmBox .fill{background:var(--warm)}
.scan{width:100%;height:auto;display:block;margin:4px 0}
.scan .rg{fill:none;stroke:rgba(74,222,128,.25);stroke-width:.6;stroke-dasharray:2 2}
.scan .base{stroke:rgba(74,222,128,.25);stroke-width:.6}
.scan .beam{stroke:rgba(74,222,128,.75);stroke-width:1}
.scan .me{fill:var(--c-hc)}
.scan text{fill:var(--sub);font-size:6px;font-family:var(--mono)}
.scan circle.pt{fill:var(--c-hc)}
.cbar{position:relative;height:8px;border-radius:6px;background:rgba(255,255,255,.06);margin:5px 0 8px}
.cbar .fill{position:absolute;top:0;background:var(--c-mpu)}
.cbar::before,.cbar::after{content:"";position:absolute;top:-2px;bottom:-2px;width:1px;background:rgba(251,191,36,.6)}
.cbar::before{left:25%}.cbar::after{left:75%}
.cbar .mid{position:absolute;left:50%;top:-3px;bottom:-3px;width:2px;margin-left:-1px;background:#cbd5e1}
.zoneMk{position:absolute;top:-2px;bottom:-2px;left:50%;width:2px;background:var(--err)}

/* events + diag */
.events{list-style:none;margin:0;padding:10px 12px;max-height:260px;overflow:auto}
.events li{display:flex;gap:10px;font-size:12px;padding:6px 0;border-top:1px dashed var(--line);color:#c3cfdb}
.events li:first-child{border-top:none}
.events time{font-family:var(--mono);color:var(--sub);font-size:10.5px;flex:none;padding-top:1px}
.events .e-c{color:#fecdd3} .events .e-w{color:#fde68a} .events .e-ok{color:#a7f3d0}
.events .empty{color:var(--sub)}
.diag{padding:12px 14px;font-size:11.5px;color:var(--sub);line-height:1.7}
.diag b{color:var(--text);font-weight:600}
body[data-link=lost] .grid,body[data-link=lost] .statusPanel{opacity:.55;filter:grayscale(.5)}
body[data-link=lost] .rescue::after{content:"LINK LOST — showing last known state";position:absolute;top:10px;right:12px;font-size:9.5px;font-weight:800;letter-spacing:1px;color:var(--err)}
footer{text-align:center;font-size:10px;color:var(--sub);margin-top:18px;letter-spacing:.6px}
@media(max-width:380px){button.pad{width:70px;height:70px}.ring{width:80px;height:80px}.ringIn{width:64px;height:64px}.rTitle{font-size:17px}}
@media(prefers-reduced-motion:reduce){*{animation:none!important;transition:none!important}}
</style>
</head>
<body data-link="init">
<div class="wrap">

<header class="top">
  <div class="brand">
    <div class="logo">🔥</div>
    <div><h1>FIRE RESCUE ROVER</h1><div class="tag">Emergency Robotics · Live Rescue Monitoring System</div></div>
  </div>
  <div class="link" id="link" data-s="init"><span class="ldot" id="ldot"></span><span id="linkTxt">CONNECTING</span></div>
</header>

<section class="glass statusPanel">
  <div class="ring" id="ring"><div class="ringIn"><b id="healthPct">--</b><small>SENSORS OK</small></div></div>
  <div>
    <div class="sysPill" id="sysPill"><span id="sysTxt">SYSTEM: CHECKING…</span></div>
    <div class="counts">
      <div><b id="okCount">-</b><span>OK</span></div>
      <div class="cw"><b id="warnCount">-</b><span>WARNING</span></div>
      <div class="ce"><b id="errCount">-</b><span>ERROR</span></div>
      <div class="cm"><b id="warmCount">-</b><span>WARMING UP</span></div>
    </div>
    <div class="meta">UPTIME <b id="uptime">--:--:--</b> &nbsp;·&nbsp; CLIENTS <b id="clients">-</b></div>
  </div>
  <ul class="reasons" id="reasons"></ul>
</section>

<button id="enableBtn" onclick="enableAlerts()">🔔 Enable Sound &amp; Notifications</button>

<div class="sec">Rescue Detection</div>
<section id="rescuePanel" class="glass rescue offline">
  <div class="rTitle" id="rescueTitle">CONNECTING…</div>
  <div class="rSub" id="rescueSub">Human-capable mmWave detection · LD2410B</div>
  <div class="rGrid">
    <div class="radar" id="radar" data-s="offline">
      <svg viewBox="0 0 200 112" aria-label="LD2410B detection zone">
        <path class="zone" d="M5 105 A95 95 0 0 1 195 105 Z"/>
        <path class="rg" d="M36.67 105 A63.33 63.33 0 0 1 163.33 105"/>
        <path class="rg" d="M68.33 105 A31.67 31.67 0 0 1 131.67 105"/>
        <line class="sp" x1="100" y1="105" x2="100" y2="10"/>
        <line class="sp" x1="100" y1="105" x2="32.8" y2="37.8"/>
        <line class="sp" x1="100" y1="105" x2="167.2" y2="37.8"/>
        <g class="sweep"><path d="M100 105 L100 10 A95 95 0 0 1 124.59 13.24 Z"/><line x1="100" y1="105" x2="100" y2="10"/></g>
        <text x="121" y="111" id="rl1">0.5 m</text><text x="153" y="111" id="rl2">1 m</text><text x="181" y="111" id="rl3">1.5 m</text>
        <g id="farG" transform="translate(100,4.5)" style="display:none"><circle class="far" r="3.2"/></g>
        <g id="blipG" transform="translate(100,60)" style="display:none"><circle class="bRing" r="3.5"/><circle class="blip" r="3.5"/></g>
        <circle class="me" cx="100" cy="105" r="3.2"/>
      </svg>
      <div class="rcap" id="rcap">1.5 m hard detection zone · range only (LD2410B reports distance, not bearing)</div>
    </div>
    <div id="rescueBody"></div>
  </div>
  <div class="prox" id="proximityBody"></div>
</section>

<div class="sec">Buzzer Control</div>
<section class="glass buzzPanel" id="buzzPanel" data-s="off">
  <div class="bzInfo">
    <div class="ct" style="--c:var(--orange)">Wireless Buzzer</div>
    <div class="cs">Auto ON when a target is detected · tap to silence · auto OFF when area is clear</div>
    <div class="bzState"><span class="badge b-gray" id="bBuzz"><i></i><span>--</span></span><span id="buzzTxt">Syncing with rover…</span></div>
  </div>
  <button class="switch" id="buzzSwitch" role="switch" aria-checked="false" aria-label="Manual buzzer" onclick="toggleBuzzer()" disabled><span class="knob"></span><span class="swl" id="buzzSwTxt">OFF</span></button>
</section>

<div class="sec">Driving</div>
<section class="glass ctrlPanel">
  <div class="speedRow">
    <button class="speedBtn" id="spLow" onclick="setSpeed('low')">LOW</button>
    <button class="speedBtn active" id="spMed" onclick="setSpeed('medium')">MEDIUM</button>
    <button class="speedBtn" id="spHigh" onclick="setSpeed('high')">HIGH</button>
  </div>
  <div class="padWrap">
    <div class="padRow"><button class="pad" data-cmd="forward">&#9650;</button></div>
    <div class="padRow">
      <button class="pad" data-cmd="left">&#9664;</button>
      <button class="pad" id="stopBtn" onclick="cmd('stop')">&#9632;</button>
      <button class="pad" data-cmd="right">&#9654;</button>
    </div>
    <div class="padRow"><button class="pad" data-cmd="backward">&#9660;</button></div>
  </div>
  <div class="driveStats">
    <div>STATE<b id="mState">STOPPED</b></div>
    <div>DRIVE OUTPUT<b id="mPWM">0</b></div>
    <div>L / R PWM<b id="mLR">0 / 0</b></div>
    <div>SPEED<b id="mSpeed">MEDIUM</b></div>
  </div>
  <div class="dnote">BTS7960 · hold to drive, release to stop · 600 ms dead-man watchdog · PWM level (no encoders)</div>
  <div class="kbd">⌨ Laptop: hold <kbd>W</kbd><kbd>A</kbd><kbd>S</kbd><kbd>D</kbd> or arrow keys to drive · <kbd>Space</kbd> STOP · <kbd>1</kbd><kbd>2</kbd><kbd>3</kbd> speed</div>
</section>

<div class="sec">Environment · Fire &amp; Gas</div>
<div class="grid">
  <div class="glass card" id="cDht" data-s="idle">
    <div class="ch"><div><div class="ct" style="--c:var(--c-dht)">DHT22</div><div class="cs">Temperature &amp; humidity</div></div><span class="badge" id="bDht"><i></i><span>--</span></span></div>
    <div class="big"><span id="tVal">--</span><small>°C</small></div>
    <div class="meter"><div class="fill heat" id="tBar"></div></div>
    <div class="kv"><span>Humidity</span><b id="hVal">-- %</b></div>
    <div class="meter"><div class="fill hum" id="hBar"></div></div>
    <div class="foot"><span id="dhtAge">--</span><span class="act" id="aDht"></span></div>
    <div class="problem" id="pDht"></div>
  </div>

  <div class="glass card" id="cMq2" data-s="idle">
    <div class="ch"><div><div class="ct" style="--c:var(--c-mq2)">MQ-2 Smoke / LPG</div><div class="cs">Baseline-tracked · qualitative (uncalibrated)</div></div><span class="badge" id="bMq2"><i></i><span>--</span></span></div>
    <div class="big" id="mq2Main">--</div>
    <div class="gauge"><div class="fill" id="mq2Bar"></div><i class="mk base" id="mq2Base"></i><i class="mk thr" id="mq2Thr"></i><i class="mk hi" id="mq2Hi"></i></div>
    <div class="gl"><span>0 V</span><span>3.3 V</span></div>
    <div class="legend"><span><i style="background:var(--cyan)"></i>baseline</span><span><i style="background:var(--warn)"></i>warn</span><span><i style="background:var(--err)"></i>high</span></div>
    <div class="kv"><span>Filtered voltage</span><b id="mq2V">--</b></div>
    <div class="kv"><span>Clean-air baseline</span><b id="mq2B">--</b></div>
    <div class="kv"><span>Rise above baseline</span><b id="mq2D">--</b></div>
    <div class="kv"><span>Raw ADC</span><b id="mq2R">--</b></div>
    <div class="warmBox" id="mq2Warm" style="display:none"><div class="meter"><div class="fill" id="mq2WarmBar"></div></div><span id="mq2WarmTxt"></span></div>
    <div class="problem" id="pMq2"></div>
  </div>

  <div class="glass card" id="cMq4" data-s="idle">
    <div class="ch"><div><div class="ct" style="--c:var(--c-mq4)">MQ-4 Methane</div><div class="cs">Qualitative · uncalibrated ADC voltage estimate</div></div><span class="badge" id="bMq4"><i></i><span>--</span></span></div>
    <div class="big" id="mq4Main">--</div>
    <div class="gauge"><div class="fill" id="mq4Bar"></div><i class="mk thr" style="left:15.2%"></i><i class="mk hi" style="left:45.5%"></i></div>
    <div class="gl"><span>0 V</span><span>3.3 V</span></div>
    <div class="kv"><span>ADC voltage estimate</span><b id="mq4V">--</b></div>
    <div class="kv"><span>Raw ADC</span><b id="mq4R">--</b></div>
    <div class="warmBox" id="mq4Warm" style="display:none"><span id="mq4WarmTxt"></span></div>
    <div class="problem" id="pMq4"></div>
  </div>

  <div class="glass card" id="cMq7" data-s="idle">
    <div class="ch"><div><div class="ct" style="--c:var(--c-mq7)">MQ-7 Carbon Monoxide</div><div class="cs">Qualitative · uncalibrated · needs heater cycling for accuracy</div></div><span class="badge" id="bMq7"><i></i><span>--</span></span></div>
    <div class="big" id="mq7Main">--</div>
    <div class="gauge"><div class="fill" id="mq7Bar"></div><i class="mk thr" style="left:15.2%"></i><i class="mk hi" style="left:45.5%"></i></div>
    <div class="gl"><span>0 V</span><span>3.3 V</span></div>
    <div class="kv"><span>ADC voltage estimate</span><b id="mq7V">--</b></div>
    <div class="kv"><span>Raw ADC</span><b id="mq7R">--</b></div>
    <div class="warmBox" id="mq7Warm" style="display:none"><span id="mq7WarmTxt"></span></div>
    <div class="problem" id="pMq7"></div>
  </div>
</div>

<div class="sec">Proximity · Motion · Radar Telemetry</div>
<div class="grid">
  <div class="glass card" id="cHc" data-s="idle">
    <div class="ch"><div><div class="ct" style="--c:var(--c-hc)">HC-SR04 Ultrasonic</div><div class="cs">Servo-mounted proximity / object sensor</div></div><span class="badge" id="bHc"><i></i><span>--</span></span></div>
    <div class="big"><span id="hcVal">--</span></div>
    <svg class="scan" viewBox="0 0 200 108">
      <path class="rg" d="M10 100 A90 90 0 0 1 190 100"/>
      <path class="rg" d="M55 100 A45 45 0 0 1 145 100"/>
      <line class="base" x1="4" y1="100" x2="196" y2="100"/>
      <g id="hcTrail"></g>
      <line id="hcBeam" class="beam" x1="100" y1="100" x2="100" y2="10"/>
      <circle class="me" cx="100" cy="100" r="3"/>
      <text x="138" y="107">2 m</text><text x="182" y="107">4 m</text>
    </svg>
    <div class="kv"><span>Scan angle (servo)</span><b id="hcAng">--</b></div>
    <div class="foot"><span id="hcAge">--</span><span class="act" id="aHc"></span></div>
    <div class="problem" id="pHc"></div>
  </div>

  <div class="glass card" id="cVl" data-s="idle">
    <div class="ch"><div><div class="ct" style="--c:var(--c-vl)">VL53L0X Time-of-Flight</div><div class="cs" id="vlSub">Proximity / object sensor · I2C 0x29</div></div><span class="badge" id="bVl"><i></i><span>--</span></span></div>
    <div class="big"><span id="vlVal">--</span></div>
    <div class="meter"><div class="fill" id="vlBar" style="background:var(--c-vl)"></div></div>
    <div class="gl"><span>0</span><span>2000 mm</span></div>
    <div class="kv"><span>Last valid</span><b id="vlLast">--</b></div>
    <div class="kv"><span>Consecutive fails</span><b id="vlFails">--</b></div>
    <div class="foot"><span id="vlAge">--</span><span class="act" id="aVl"></span></div>
  </div>

  <div class="glass card" id="cMpu" data-s="idle">
    <div class="ch"><div><div class="ct" style="--c:var(--c-mpu)">MPU6050 Motion</div><div class="cs">Chassis orientation · limit ±45°</div></div><span class="badge" id="bMpu"><i></i><span>--</span></span></div>
    <div class="big"><span id="mpuVal">--</span></div>
    <div class="kv"><span>Pitch</span><b id="pVal">--</b></div>
    <div class="cbar"><div class="fill" id="pBar"></div><i class="mid"></i></div>
    <div class="kv"><span>Roll</span><b id="rVal">--</b></div>
    <div class="cbar"><div class="fill" id="rBar"></div><i class="mid"></i></div>
    <div class="problem" id="pMpu"></div>
  </div>

  <div class="glass card" id="cLd" data-s="idle">
    <div class="ch"><div><div class="ct" style="--c:var(--c-ld)">LD2410B mmWave Radar</div><div class="cs" id="ldSub">Raw UART telemetry (before 1.5 m gate)</div></div><span class="badge" id="bLd"><i></i><span>--</span></span></div>
    <div class="big" style="font-size:19px"><span id="ldType">--</span></div>
    <div class="kv"><span>Moving</span><b id="ldMov">--</b></div>
    <div class="meter"><div class="fill" id="ldMovBar" style="background:var(--c-ld)"></div><i class="zoneMk"></i></div>
    <div class="kv"><span>Still</span><b id="ldStill">--</b></div>
    <div class="meter"><div class="fill" id="ldStillBar" style="background:var(--orange)"></div><i class="zoneMk"></i></div>
    <div class="gl"><span>0 m</span><span id="ldZl">1.5 m ▲</span><span id="ldZr">3 m</span></div>
    <div class="kv"><span>Detection zone</span><b id="ldZone">--</b></div>
    <div class="kv"><span>Module range setting</span><b id="ldHw">--</b></div>
    <div class="kv"><span>OUT pin (info only)</span><b id="ldOut">--</b></div>
    <div class="foot"><span id="ldAge">--</span><span class="act" id="aLd"></span></div>
  </div>
</div>

<div class="sec">Live Event Feed</div>
<section class="glass"><ul class="events" id="events"><li class="empty">Waiting for telemetry…</li></ul></section>

<div class="sec">System Diagnostics</div>
<section class="glass diag" id="diagPanel"></section>

<footer>FIRE RESCUE ROVER · ESP32 · AP 192.168.4.1</footer>
</div>

<script>
// ---------- helpers ----------
const $=id=>document.getElementById(id);
function T(id,v){const e=$(id);if(!e)return;v=String(v);if(e.textContent!==v)e.textContent=v;}
function H(id,v){const e=$(id);if(e&&e._h!==v){e.innerHTML=v;e._h=v;}}
function clamp(p){return Math.max(0,Math.min(100,p));}
function W(id,p){const e=$(id);if(e)e.style.width=clamp(p).toFixed(1)+'%';}
function Lf(id,p){const e=$(id);if(e)e.style.left=clamp(p).toFixed(1)+'%';}
function B(id,text,c){const e=$(id);if(!e)return;const k='badge '+c;if(e.className!==k)e.className=k;const s=e.lastElementChild;if(s.textContent!==text)s.textContent=text;}
function S(id,s){const e=$(id);if(e&&e.dataset.s!==s)e.dataset.s=s;}
function show(id,on){const e=$(id);if(e)e.style.display=on?'':'none';}
function ping(id){const e=$(id);if(!e)return;e.classList.remove('on');void e.offsetWidth;e.classList.add('on');}
function num(v){return v!=null&&!isNaN(v);}
function ago(sec){if(sec==null||sec<0)return 'never';if(sec<1)return 'just now';return sec.toFixed(1)+'s ago';}
function pad2(n){return String(n).padStart(2,'0');}
function fm(m){return (Math.round(m*100)/100)+' m';}

// ---------- alerts (unchanged behaviour) ----------
let audioCtx=null, alertsEnabled=false;
function enableAlerts(){
  audioCtx = new (window.AudioContext||window.webkitAudioContext)();
  if (window.Notification && Notification.permission!=="granted" && Notification.permission!=="denied") {
    Notification.requestPermission();
  }
  alertsEnabled=true;
  const b=$('enableBtn'); b.innerText="✓ Alerts Enabled"; b.classList.add('on');
}
function playBeep(){ if(!audioCtx) return; const o=audioCtx.createOscillator(); const g=audioCtx.createGain();
  o.type='square'; o.frequency.value=880; o.connect(g); g.connect(audioCtx.destination);
  g.gain.setValueAtTime(0.3,audioCtx.currentTime); o.start(); o.stop(audioCtx.currentTime+0.4); }
function notifyTarget(desc){
  if (window.Notification && Notification.permission==="granted") {
    try{ new Notification("🚨 Fire Rescue Rover",{body:"Possible human/target detected — "+desc}); }catch(e){}
  }
  playBeep();
}
// ---------- wireless buzzer ----------
// Buzzer = ON automatically while a target is detected; the switch can silence it.
// Switch always shows the REAL pin state (from /buzzer reply, then every /data poll).
// With the area clear the switch is disabled and the buzzer is off.
let buzzOn=false, buzzPending=false;
function applyBuzz(on,armed){
  buzzOn=on;
  const sw=$('buzzSwitch'); sw.classList.toggle('on',on); sw.setAttribute('aria-checked',on?'true':'false');
  sw.disabled=!armed; T('buzzSwTxt',on?'ON':'OFF');
}
function toggleBuzzer(){
  if(buzzPending) return;
  buzzPending=true; $('buzzSwitch').classList.add('pending');
  fetch('/buzzer?state='+(buzzOn?'off':'on'),{cache:'no-store'})
    .then(r=>r.text()).then(t=>applyBuzz(t.trim()==='ON',true))
    .catch(()=>logEvent('w','Buzzer command not delivered — check link'))
    .finally(()=>{ buzzPending=false; $('buzzSwitch').classList.remove('pending'); });
}
function renderBuzzer(d){
  const z=d.buzzer, rs=d.ld2410.rescueState;
  if(!buzzPending) applyBuzz(!!z.active,!!z.armed);
  let st,txt,ev;
  if(z.active){st='CRITICAL';txt='Sounding · target detected — tap to silence';ev='on';}
  else if(z.armed){st='WARNING';txt='Silenced from dashboard · target still detected — tap to re-sound';ev='silenced';}
  else if(rs==='OFFLINE'||rs==='NODATA'){st='OFFLINE';txt='Off · radar link error';ev='off';}
  else {st='OK';txt='Off · area clear — sounds automatically on detection';ev='off';}
  B('bBuzz',st,VC[st]); T('buzzTxt',txt);
  S('buzzPanel',z.active?'alarm':'off');
  track('buzz',ev,(v)=>logEvent(v==='on'?'c':(v==='silenced'?'w':'ok'),
    v==='on'?'Buzzer ON — target detected':(v==='silenced'?'Buzzer silenced from dashboard':'Buzzer OFF — area clear')));
}
function setSpeed(mode){
  fetch('/speed?mode='+mode);
  ['spLow','spMed','spHigh'].forEach(id=>$(id).classList.remove('active'));
  const map={low:'spLow',medium:'spMed',high:'spHigh'};
  $(map[mode]).classList.add('active');
}

// ---------- driving: press+hold = drive, release = stop, heartbeat (unchanged) ----------
let driveInterval=null, activeCmd=null;
function cmd(c){ fetch('/'+c); }
function startDrive(c){
  if(activeCmd===c) return;
  activeCmd=c; cmd(c);
  if(driveInterval) clearInterval(driveInterval);
  driveInterval=setInterval(()=>cmd(c),250); // heartbeat keeps watchdog satisfied
}
function stopDrive(){
  if(driveInterval){ clearInterval(driveInterval); driveInterval=null; }
  activeCmd=null; cmd('stop');
}
document.querySelectorAll('.pad[data-cmd]').forEach(btn=>{
  const c=btn.getAttribute('data-cmd');
  btn.addEventListener('pointerdown', e=>{ e.preventDefault(); btn.setPointerCapture(e.pointerId); startDrive(c); });
  btn.addEventListener('pointerup', stopDrive);
  btn.addEventListener('pointercancel', stopDrive);
  btn.addEventListener('pointerleave', stopDrive);
});
function emergencyStop(){ kbHeld=[]; padHi(null); stopDrive(); }

// ---------- laptop keyboard driving (W A S D / arrows) ----------
// Same path as the on-screen pad: hold = drive (250 ms heartbeat), release = stop.
// Auto-repeat is ignored, so after any emergency stop the key must be pressed again.
const KEYMAP={KeyW:'forward',ArrowUp:'forward',KeyS:'backward',ArrowDown:'backward',KeyA:'left',ArrowLeft:'left',KeyD:'right',ArrowRight:'right'};
let kbHeld=[];
function padHi(c){ document.querySelectorAll('.pad[data-cmd]').forEach(b=>b.classList.toggle('kbon',b.getAttribute('data-cmd')===c)); }
window.addEventListener('keydown',e=>{
  if(e.ctrlKey||e.metaKey||e.altKey) return;
  const tg=e.target&&e.target.tagName; if(tg==='INPUT'||tg==='TEXTAREA') return;
  if(e.code==='Space'){ e.preventDefault(); emergencyStop(); return; }
  if(!e.repeat&&(e.code==='Digit1'||e.code==='Digit2'||e.code==='Digit3')){ setSpeed({Digit1:'low',Digit2:'medium',Digit3:'high'}[e.code]); return; }
  const c=KEYMAP[e.code]; if(!c) return;
  e.preventDefault(); if(e.repeat) return;
  kbHeld=kbHeld.filter(k=>k!==e.code); kbHeld.push(e.code);
  startDrive(c); padHi(c);
});
window.addEventListener('keyup',e=>{
  if(e.code==='Space'){ e.preventDefault(); return; }   // stops a focused button being "clicked" by Space
  const c=KEYMAP[e.code]; if(!c) return;
  e.preventDefault();
  if(!kbHeld.includes(e.code)) return;
  kbHeld=kbHeld.filter(k=>k!==e.code);
  if(kbHeld.length){ const n=KEYMAP[kbHeld[kbHeld.length-1]]; startDrive(n); padHi(n); }
  else { padHi(null); stopDrive(); }
});
window.addEventListener('pointerup', emergencyStop);
window.addEventListener('pointercancel', emergencyStop);
window.addEventListener('blur', emergencyStop);
document.addEventListener('visibilitychange', () => { if (document.hidden) emergencyStop(); });

// ---------- event feed ----------
const events=[];
function logEvent(lv,msg){
  const t=new Date(), ts=pad2(t.getHours())+':'+pad2(t.getMinutes())+':'+pad2(t.getSeconds());
  events.unshift({lv:lv,msg:msg,ts:ts}); if(events.length>15) events.pop();
  H('events',events.map(e=>'<li class="e-'+e.lv+'"><time>'+e.ts+'</time><span>'+e.msg+'</span></li>').join(''));
}
const prev={};
function track(key,val,fn){ if(prev[key]!==undefined && prev[key]!==val) fn(val,prev[key]); prev[key]=val; }

// ---------- one status vocabulary for every card ----------
// OK · WARNING · ERROR · OFFLINE · WARMING UP · SCANNING  (+ CRITICAL for hazards, TARGET for radar)
const VC={'OK':'b-ok','WARNING':'b-warn','ERROR':'b-err','OFFLINE':'b-err','WARMING UP':'b-warm','SCANNING':'b-cyan','CRITICAL':'b-err','TARGET':'b-err'};
const VS={'OK':'ok','WARNING':'warn','ERROR':'err','OFFLINE':'err','WARMING UP':'warm','SCANNING':'info','CRITICAL':'err','TARGET':'err'};
function stat(badgeId,cardId,st){B(badgeId,st,VC[st]||'b-gray');S(cardId,VS[st]||'idle');}
function gasVocab(l){return l==='HIGH'?'CRITICAL':(l==='WARNING'||l==='ELEVATED')?'WARNING':(l==='WARMING UP')?'WARMING UP':'OK';}
function lvlFill(l){return 'fill g-'+({CRITICAL:'err',WARNING:'warn','WARMING UP':'warm'}[gasVocab(l)]||'ok');}

// ---------- renderers ----------
function renderHealth(d){
  const tot=d.totalCount||1;
  T('healthPct',d.okCount+'/'+tot);                      // real sensor count, not an invented score
  const lv=d.health==='NORMAL'?'ok':(d.health==='WARNING'?'warn':'crit');
  $('ring').style.setProperty('--p',Math.round(d.okCount/tot*100)); S('ring',lv); S('sysPill',lv);
  T('sysTxt',(lv==='ok'?'🟢':lv==='warn'?'🟡':'🔴')+' SYSTEM: '+d.health);
  T('okCount',d.okCount); T('warnCount',d.warnCount); T('errCount',d.errCount); T('warmCount',d.warmCount);
  const s=d.uptime; T('uptime',pad2(Math.floor(s/3600))+':'+pad2(Math.floor((s%3600)/60))+':'+pad2(s%60));
  T('clients',d.wifiClients);
  let h='';
  if(!d.reasons||!d.reasons.length) h='<li class="r-ok">All monitored systems nominal</li>';
  else d.reasons.forEach(r=>{h+='<li class="r-'+r.l+'">'+r.m+'</li>';});
  H('reasons',h);
  track('health',d.health,(v)=>logEvent(v==='NORMAL'?'ok':(v==='WARNING'?'w':'c'),'System status → '+v));
}

function nearestRaw(L){
  let best=null;
  if((L.targetState&1)&&L.movingDist>0) best=L.movingDist;
  if((L.targetState&2)&&L.stillDist>0&&(best==null||L.stillDist<best)) best=L.stillDist;
  return best;
}

let prevRescue=null;
function renderRescue(d){
  const L=d.ld2410, rs=L.rescueState;
  const map={TARGET:'alert',VERIFYING:'verify',CLEAR:'clear',OUT_OF_RANGE:'clear',OFFLINE:'offline',NODATA:'offline'};
  const mode=map[rs]||'offline';
  const p=$('rescuePanel'), c='glass rescue '+mode; if(p.className!==c) p.className=c;
  const zoneM=fm(L.maxRangeCm/100);
  T('rl1',fm(L.maxRangeCm/300)); T('rl2',fm(L.maxRangeCm/150)); T('rl3',zoneM);
  T('rcap',zoneM+' hard detection zone · range only (LD2410B reports distance, not bearing)');
  const dist=L.targetDist>0?(L.targetDist/100).toFixed(2)+' m':'--';
  const row=(k,v)=>'<div class="row2"><span>'+k+'</span><b>'+v+'</b></div>';
  const pair=(k1,v1,k2,v2)=>'<div class="row2 pair"><span>'+k1+'<b>'+v1+'</b></span><span>'+k2+'<b>'+v2+'</b></span></div>';
  let title,sub,rows;
  if(rs==='TARGET'){
    title='🚨 TARGET DETECTED'; sub='Possible human / target detected · human-capable mmWave detection';
    rows=row('Source','LD2410B mmWave Radar (Human-capable)')+row('Target','Possible Human / '+L.zoneType)
        +row('Distance',dist)+pair('UART','OK','Range','Within '+zoneM)
        +row('Corroboration','VL53L0X / HC-SR04')+row('OUT pin',L.outPin+' (info only)');
  } else if(rs==='VERIFYING'){
    title='⚠️ VERIFYING TARGET'; sub='Target reported within '+zoneM+' — confirming over consecutive radar frames';
    rows=row('Source','LD2410B mmWave Radar (Human-capable)')+row('Candidate',L.zoneType+' · '+dist)+pair('UART','OK','Range','Within '+zoneM);
  } else if(rs==='OUT_OF_RANGE'){
    const r=nearestRaw(L);
    title='🟢 AREA CLEAR'; sub='Target outside configured '+zoneM+' detection zone';
    rows=pair('LD2410B','ONLINE','Radar','SCANNING')+row('Reported target (ignored)',r!=null?(r/100).toFixed(2)+' m':'beyond zone')
        +row('Detection zone','0 – '+(L.maxRangeCm/100).toFixed(2)+' m')+row('OUT pin',L.outPin+' (info only)');
  } else if(rs==='CLEAR'){
    title='🟢 AREA CLEAR'; sub='No valid target detected within '+zoneM;
    rows=pair('LD2410B','ONLINE','Radar','SCANNING')+row('Detection zone','0 – '+(L.maxRangeCm/100).toFixed(2)+' m')+row('OUT pin',L.outPin+' (info only)');
  } else {
    title='⚠️ RADAR LINK ERROR';
    sub=rs==='NODATA'?'UART communication lost — no LD2410B frames received since boot (check wiring / power)'
                     :'UART communication lost — detection unavailable (this is NOT “area clear”)';
    rows=row('UART',rs==='NODATA'?'OFFLINE (no data)':'OFFLINE (last frame '+ago(L.age)+')')+row('Detection','UNAVAILABLE')+row('OUT pin',L.outPin+' (never used alone)');
  }
  T('rescueTitle',title); T('rescueSub',sub); H('rescueBody',rows);

  S('radar',mode);
  if((rs==='TARGET'||rs==='VERIFYING')&&L.targetDist>0){
    const y=105-Math.min(1,L.targetDist/L.maxRangeCm)*95;
    $('blipG').setAttribute('transform','translate(100,'+y.toFixed(1)+')'); show('blipG',true);
  } else show('blipG',false);
  show('farG',rs==='OUT_OF_RANGE');

  if(rs==='TARGET'){
    H('proximityBody','<b>Proximity corroboration</b> (object distance only — not human identification)<br>VL53L0X: '
      +(d.vl53.status==='VALID'&&num(d.vl53.filtered)?d.vl53.filtered+' mm':(d.vl53.status==='NO OBJECT'?'no object < 2 m':'n/a'))
      +' &nbsp;·&nbsp; HC-SR04: '+(d.hc.valid&&num(d.hc.dist)?d.hc.dist.toFixed(1)+' cm @ '+d.hc.angle+'°':'n/a'));
  } else H('proximityBody','');


  if(rs==='TARGET'&&prevRescue!=='TARGET'&&alertsEnabled) notifyTarget(L.zoneType+' at '+dist);
  if(prevRescue!==null&&rs!==prevRescue){
    const msg={TARGET:'🚨 Target acquired — possible human at '+dist,VERIFYING:'Radar candidate within '+zoneM+' — verifying',
      CLEAR:(prevRescue==='TARGET'?'Target lost — ':'')+'Area clear — no valid target within '+zoneM,
      OUT_OF_RANGE:'Target outside '+zoneM+' zone — ignored',
      OFFLINE:'Radar link error — UART communication lost',NODATA:'Radar link error — no UART data'}[rs];
    logEvent(rs==='TARGET'||rs==='OFFLINE'||rs==='NODATA'?'c':(rs==='VERIFYING'?'w':'ok'),msg);
  }
  prevRescue=rs;
}

function renderDrive(d){
  T('mState',d.motor.state); T('mPWM',d.motor.cmdPWM); T('mLR',d.motor.leftPWM+' / '+d.motor.rightPWM); T('mSpeed',d.motor.speedMode);
  const map={LOW:'spLow',MEDIUM:'spMed',HIGH:'spHigh'}, id=map[d.motor.speedMode];
  if(id) ['spLow','spMed','spHigh'].forEach(x=>$(x).classList.toggle('active',x===id));
}

function gasWarm(prefix,warmedUp,left){ show(prefix+'Warm',!warmedUp); if(!warmedUp) T(prefix+'WarmTxt','Warming up · '+left+' s (no warnings raised yet)'); }
function renderEnv(d){
  // DHT22
  const h=d.dht;
  T('tVal',h.everValid?h.temp.toFixed(1):'--'); T('hVal',h.everValid?h.hum.toFixed(1)+' %':'-- %');
  W('tBar',h.temp/80*100); W('hBar',h.hum);
  const dSt=!h.valid?(h.stale?'OFFLINE':'WARNING'):(h.temp>=d.tempCrit?'CRITICAL':(h.temp>=d.tempWarn?'WARNING':'OK'));
  stat('bDht','cDht',dSt);
  T('dhtAge',(h.valid?'Live · ':'Last valid · ')+'updated '+ago(h.everValid?h.age:-1)); if(h.valid&&h.age<2.1) ping('aDht');
  T('pDht',h.valid?'':'⚠ '+(h.stale?'No valid reading for 30+ s — check sensor':h.problem));
  track('dht',dSt,(v)=>logEvent(v==='OK'?'ok':(v==='CRITICAL'?'c':'w'),'DHT22 → '+v));

  // MQ-2 (baseline-tracked)
  const m=d.mq2, s2=m.problem!=='OK'?'ERROR':gasVocab(m.lvl);
  stat('bMq2','cMq2',s2);
  T('mq2Main',{NORMAL:'NORMAL · SAFE',WARNING:'GAS WARNING',HIGH:'GAS DETECTED','WARMING UP':'CALIBRATING'}[m.lvl]||m.lvl);
  $('mq2Bar').className=lvlFill(m.lvl); W('mq2Bar',m.filt/3.3*100);
  Lf('mq2Base',m.base/3.3*100); Lf('mq2Thr',(m.base+m.warnDelta)/3.3*100); Lf('mq2Hi',m.highAbs/3.3*100);
  T('mq2V',m.filt.toFixed(2)+' V'); T('mq2B',m.base.toFixed(2)+' V'); T('mq2D',(m.delta>=0?'+':'')+m.delta.toFixed(2)+' V'); T('mq2R',m.raw);
  show('mq2Warm',m.lvl==='WARMING UP');
  if(m.lvl==='WARMING UP'){ W('mq2WarmBar',100-m.warmLeft/m.warmTotal*100); T('mq2WarmTxt','Heater stabilising · calibrating clean-air baseline · '+m.warmLeft+' s'); }
  T('pMq2',m.problem!=='OK'?'⚠ '+m.problem:'');
  track('mq2',m.lvl,(v,o)=>logEvent(v==='HIGH'?'c':(v==='WARNING'?'w':'ok'),
    v==='NORMAL'&&(o==='WARNING'||o==='HIGH')?'MQ-2 gas warning cleared':(v==='NORMAL'?'MQ-2 calibrated — clean air':'MQ-2 → '+(v==='HIGH'?'GAS DETECTED':v))));

  // MQ-4 (classification unchanged; warm-up display only)
  const q=d.mq4, l4=q.warmedUp?q.lvl:'WARMING UP', s4=q.problem!=='OK'?'ERROR':gasVocab(l4);
  stat('bMq4','cMq4',s4); T('mq4Main',l4);
  $('mq4Bar').className=lvlFill(l4); W('mq4Bar',q.v/3.3*100);
  T('mq4V',q.v.toFixed(2)+' V'); T('mq4R',q.raw); T('pMq4',q.problem!=='OK'?'⚠ '+q.problem:'');
  gasWarm('mq4',q.warmedUp,q.warmLeft);
  track('mq4',l4,(v)=>logEvent(v==='HIGH'?'w':'ok','MQ-4 methane → '+v));

  // MQ-7 (unchanged classification)
  const k=d.mq7, l7=k.warmedUp?k.lvl:'WARMING UP', s7=k.problem!=='OK'?'ERROR':gasVocab(l7);
  stat('bMq7','cMq7',s7); T('mq7Main',l7);
  $('mq7Bar').className=lvlFill(l7); W('mq7Bar',k.v/3.3*100);
  T('mq7V',k.v.toFixed(2)+' V'); T('mq7R',k.raw); T('pMq7',k.problem!=='OK'?'⚠ '+k.problem:'');
  gasWarm('mq7',k.warmedUp,k.warmLeft);
  track('mq7',l7,(v)=>logEvent(v==='HIGH'?'w':'ok','MQ-7 CO → '+v));
}

const trail=[];
function scanXY(angle,distCm){const r=Math.min(400,distCm)/400*90, a=angle*Math.PI/180;return [100+r*Math.cos(a),100-r*Math.sin(a)];}
function renderProx(d){
  // HC-SR04
  const c=d.hc, sh=c.valid?'OK':(c.longNoEcho?'WARNING':'SCANNING');
  stat('bHc','cHc',sh);
  T('hcVal',c.valid&&num(c.dist)?c.dist.toFixed(1)+' cm':'NO ECHO');
  T('hcAng',d.servoAngle+'°'); T('hcAge','Last echo '+ago(num(c.lastValid)?c.age:-1));
  const tip=scanXY(d.servoAngle,400), be=$('hcBeam'); be.setAttribute('x2',tip[0].toFixed(1)); be.setAttribute('y2',tip[1].toFixed(1));
  if(c.valid&&num(c.dist)){ trail.push(scanXY(c.angle,c.dist)); if(trail.length>14) trail.shift(); ping('aHc'); }
  let s='';
  trail.forEach((pt,i)=>{ s+='<circle class="pt" cx="'+pt[0].toFixed(1)+'" cy="'+pt[1].toFixed(1)+'" r="2.4" opacity="'+((i+1)/trail.length).toFixed(2)+'"/>'; });
  H('hcTrail',s);
  T('pHc',c.longNoEcho?'No echo for 30+ s — open space beyond ~4 m, or check TRIG/ECHO wiring & divider':'');

  // VL53L0X
  const v=d.vl53, vs=v.status;
  const vOk=vs==='VALID', vNo=vs==='NO OBJECT';
  const sv={VALID:'OK','NO OBJECT':'SCANNING',STALE:'WARNING',RECOVERING:'WARNING','NOT FOUND':'OFFLINE','INIT FAILED':'ERROR'}[vs]||'ERROR';
  stat('bVl','cVl',sv);
  T('vlVal',vOk&&num(v.filtered)?v.filtered+' mm':(vNo?'> 2 m':(num(v.lastValid)?v.lastValid+' mm*':'—')));
  W('vlBar',vOk&&num(v.filtered)?v.filtered/20:(vNo?100:0));
  T('vlLast',num(v.lastValid)?v.lastValid+' mm':'n/a'); T('vlFails',v.fails);
  T('vlAge','Last distance '+ago(num(v.lastValid)?v.age:-1)); if(vOk) ping('aVl');
  T('vlSub',vNo?'Sensor OK · no object within ~2 m':(vs==='RECOVERING'?'Re-initialising sensor (recovering)':(vs==='INIT FAILED'?'Sensor found but init failed':'Proximity / object sensor · I2C '+v.addr)));
  track('vl53',sv,(x)=>logEvent((x==='OK'||x==='SCANNING')?'ok':'w','VL53L0X → '+x+(vs==='RECOVERING'?' (recovering)':'')));

  // MPU6050
  const u=d.mpu;
  const su=!u.found?'OFFLINE':(!u.valid?'ERROR':(u.tiltSafe?'OK':'CRITICAL'));
  stat('bMpu','cMpu',su);
  T('mpuVal',u.found&&u.valid&&num(u.pitch)?u.pitch.toFixed(1)+'° / '+u.roll.toFixed(1)+'°':(u.found?'READ ERROR':'NOT AVAILABLE'));
  T('pVal',num(u.pitch)?u.pitch.toFixed(1)+'°':'--'); T('rVal',num(u.roll)?u.roll.toFixed(1)+'°':'--');
  cbar('pBar',num(u.pitch)?u.pitch:0); cbar('rBar',num(u.roll)?u.roll:0);
  T('pMpu',!u.found?'⚠ MPU6050 not found on I2C — tilt safety unavailable':(!u.tiltSafe&&u.valid?'⚠ Chassis tilt beyond ±45°':''));
  track('mpu',su,(x)=>logEvent(x==='OK'?'ok':'c','MPU6050 → '+(x==='CRITICAL'?'UNSAFE TILT':x)));

  // LD2410B raw telemetry
  const L=d.ld2410, sl=!L.uartFresh?'OFFLINE':(L.presence?'TARGET':'SCANNING');
  stat('bLd','cLd',sl);
  T('ldType',L.uartFresh?L.targetType:'—');
  T('ldMov',L.movingDist+' cm · E '+L.movingEnergy); T('ldStill',L.stillDist+' cm · E '+L.stillEnergy);
  const zm=L.maxRangeCm/100, scale=L.maxRangeCm*2;          // bar = 0 .. 2x zone, red mark = zone edge
  W('ldMovBar',(L.targetState&1)?L.movingDist/scale*100:0); W('ldStillBar',(L.targetState&2)?L.stillDist/scale*100:0);
  T('ldZl',fm(zm)+' ▲'); T('ldZr',fm(zm*2)); T('ldSub','Raw UART telemetry (before '+fm(zm)+' gate)');
  T('ldZone',!L.uartFresh?'n/a':(L.inZone?'Target inside '+fm(zm):(L.outOfRange?'Outside '+fm(zm)+' (ignored)':'No target')));
  T('ldHw',!L.cfgAck?'not confirmed (no ACK)':(!L.gateOk?'gate not set':(L.resCm?(L.resCm/100).toFixed(2)+' m/gate · gate '+L.hwGate+' = '+(L.resCm*L.hwGate/100).toFixed(2)+' m'
    :'gate '+L.hwGate+' (≈'+(0.75*L.hwGate).toFixed(2)+' m, 0.75 m assumed)')));
  T('ldOut',L.outPin); T('ldAge','UART frame '+ago(L.age)); if(L.uartFresh) ping('aLd');
}
function cbar(id,v){const e=$(id);if(!e)return;const w=Math.min(50,Math.abs(v)/90*50);e.style.width=w.toFixed(1)+'%';e.style.left=(v>=0?50:50-w).toFixed(1)+'%';e.style.background=Math.abs(v)>45?'var(--err)':'var(--c-mpu)';}

function renderDiag(d){
  H('diagPanel',
    '<div>I2C — MPU6050 0x68: <b>'+(d.mpu.found?'FOUND':'NOT FOUND')+'</b> · VL53L0X 0x29: <b>'+(d.vl53.addrFound?'FOUND':'NOT FOUND')+'</b></div>'+
    '<div>Detection sources: <b>'+d.detectionSources+'</b></div>'+
    '<div>Radar zone: <b>0 – '+(d.ld2410.maxRangeCm/100).toFixed(2)+' m</b> · debounce <b>'+d.ld2410.onFrames+' frames</b></div>'+
    '<div>LD2410B config: <b>'+d.ld2410.cfgNote+'</b></div>'+
    '<div>MQ-2 thresholds: warn <b>+'+d.mq2.warnDelta.toFixed(2)+' V</b> above baseline · high <b>≥ '+d.mq2.highAbs.toFixed(2)+' V</b> or <b>+'+d.mq2.highDelta.toFixed(2)+' V</b></div>'+
    '<div>Wi-Fi clients: <b>'+d.wifiClients+'</b> · Buzzer: <b>'+(d.buzzer.active?'ON (target)':(d.buzzer.armed?'SILENCED':'OFF'))+'</b></div>');
}

// ---------- polling loop with link supervision ----------
let lastOk=0, inflight=false, linkState='init';
function setLink(s){
  if(s===linkState) return;
  S('link',s); T('linkTxt',s==='ok'?'LIVE':(s==='lost'?'LINK LOST':'CONNECTING'));
  document.body.dataset.link=s;
  if(linkState==='ok'&&s==='lost') logEvent('c','Connection to rover lost');
  if(linkState==='lost'&&s==='ok') logEvent('ok','Connection restored');
  if(linkState==='init'&&s==='ok') logEvent('ok','Connected to rover');
  linkState=s;
}
function refresh(){
  if(inflight){ if(Date.now()-lastOk>3500) setLink('lost'); return; }
  inflight=true;
  const ctl=window.AbortController?new AbortController():null;
  const to=setTimeout(()=>{ if(ctl) ctl.abort(); },2500);
  fetch('/data',ctl?{signal:ctl.signal,cache:'no-store'}:{cache:'no-store'}).then(r=>r.json()).then(d=>{
    lastOk=Date.now(); setLink('ok'); ping('ldot');
    try{ renderHealth(d); renderRescue(d); renderBuzzer(d); renderDrive(d); renderEnv(d); renderProx(d); renderDiag(d); }
    catch(e){ console.error(e); }
  }).catch(()=>{ if(Date.now()-lastOk>3500) setLink('lost'); })
    .finally(()=>{ clearTimeout(to); inflight=false; });
}
setInterval(refresh,1000);
refresh();
</script>
</body>
</html>
)rawliteral";

// ==================== WEB ROUTES (UNCHANGED) ====================
void handleRoot() { server.send_P(200, "text/html", htmlPage); } // served straight from flash, no heap copy
void handleForward() { motorForward(); server.send(200, "text/plain", "OK"); }
void handleBackward() { motorBackward(); server.send(200, "text/plain", "OK"); }
void handleLeft() { motorTurnLeftFn(); server.send(200, "text/plain", "OK"); }
void handleRight() { motorTurnRightFn(); server.send(200, "text/plain", "OK"); }
void handleStop() { motorStop(); server.send(200, "text/plain", "OK"); }
void handleMuteBuzzer() { if (presenceNow) buzzerMuted = !buzzerMuted; server.send(200, "text/plain", buzzerMuted ? "MUTED" : "UNMUTED"); }
// Wireless buzzer control.  /buzzer?state=off -> silence the current target alarm
// /buzzer?state=on -> re-arm (sounds again only while a target is detected)
// /buzzer          -> toggle.   Replies with the REAL pin state: "ON" / "OFF".
// Only acts while a target is detected; with the area clear the buzzer stays off.
void handleBuzzerToggle() {
  if (presenceNow) {
    if (server.hasArg("state")) {
      String st = server.arg("state"); st.toLowerCase();
      buzzerMuted = !(st == "on" || st == "1" || st == "true");
    } else {
      buzzerMuted = !buzzerMuted;
    }
  }
  server.send(200, "text/plain", (presenceNow && !buzzerMuted) ? "ON" : "OFF");
}
void handleSpeed() {
  if (server.hasArg("mode")) applySpeedMode(server.arg("mode"));
  else if (server.hasArg("pwm")) applySpeedPWM(server.arg("pwm").toInt());
  server.send(200, "text/plain", speedMode);
}

const char* rescueStateString(bool fresh) {
  if (!fresh) return ld2410EverReceived ? "OFFLINE" : "NODATA"; // never claim "clear" without radar data
  if (presenceNow) return "TARGET";
  if (ld2410FrameInRange) return "VERIFYING";                   // in-range, debounce not yet satisfied
  if (ld2410FrameOutOfRange) return "OUT_OF_RANGE";             // target reported beyond the zone -> clear
  return "CLEAR";
}

void handleData() {
  unsigned long now = millis();
  bool ld2410Fresh = ld2410EverReceived && (now - ld2410LastFrameTime) < LD2410_STALE_MS;
  float ld2410Age = ld2410EverReceived ? (now - ld2410LastFrameTime) / 1000.0 : -1.0;

  const char* targetType;
  switch (ld2410TargetState) {
    case 0: targetType = "No target"; break;
    case 1: targetType = "Moving only"; break;
    case 2: targetType = "Still only"; break;
    case 3: targetType = "Moving + Still"; break;
    default: targetType = "Unknown"; break;
  }
  const char* zoneType = (ld2410InRangeState == 3) ? "Moving + Still" : (ld2410InRangeState == 1 ? "Moving" : (ld2410InRangeState == 2 ? "Still" : "None"));

  String vlSt = vl53StatusString();
  float vlAge = (now - vl53LastValidTime) / 1000.0;
  float dhtAge = (now - dhtLastValidTime) / 1000.0;
  float hcAge = (now - hcLastValidTime) / 1000.0;

  // ---------------- System health ----------------
  // FAULT on a mission-critical item (radar) or a real hazard -> CRITICAL.
  // FAULT on a non-critical sensor -> WARNING. Short blips -> DEGRADED only.
  int okCount = 0, warnCount = 0, errCount = 0, warmCount = 0; const int totalCount = 8;
  bool crit = false, warn = false;
  String reasons = "["; bool firstR = true;
  auto addReason = [&](const char* l, const String& m) {
    if (!firstR) reasons += ",";
    reasons += "{\"l\":\""; reasons += l; reasons += "\",\"m\":\""; reasons += m; reasons += "\"}";
    firstR = false;
  };

  // 1) LD2410B - primary rescue sensor
  if (ld2410Fresh) okCount++;
  else { errCount++; crit = true; addReason("c", ld2410EverReceived ? "LD2410B radar UART link lost - rescue detection unavailable" : "LD2410B radar: no UART data received - check wiring/power"); }

  // 2) DHT22
  bool dhtStaleLong = (dhtLastValidTime == 0) || (now - dhtLastValidTime > 30000);
  if (dhtValidNow) okCount++;
  else if (dhtStaleLong) { errCount++; warn = true; addReason("w", "DHT22 not responding - temperature unavailable"); }
  else warnCount++;
  bool tempRecent = (dhtLastValidTime != 0) && (now - dhtLastValidTime <= 30000);
  if (tempRecent && lastValidTemp >= TEMP_CRIT_C) { crit = true; addReason("c", "High temperature " + String(lastValidTemp, 1) + " °C"); }
  else if (tempRecent && lastValidTemp >= TEMP_WARN_C) { warn = true; addReason("w", "Elevated temperature " + String(lastValidTemp, 1) + " °C"); }

  // 3) MQ-2
  if (mq2ZeroCount > 200) { errCount++; warn = true; addReason("w", "MQ-2 stuck at 0 - check wiring"); }
  else if (mq2State == MQ2_WARMING) { warmCount++; addReason("i", "MQ-2 warming up / learning clean-air baseline (" + String(warmupLeftSec(MQ2_WARMUP_MS)) + " s)"); }
  else okCount++;
  if (mq2State == MQ2_HIGH) { crit = true; addReason("c", "MQ-2 smoke/gas HIGH (confirmed)"); }
  else if (mq2State == MQ2_WARNING) { warn = true; addReason("w", "MQ-2 smoke/gas rising above clean-air baseline"); }

  // 4) MQ-4 (classification unchanged)
  const char* mq4Lvl = gasLevel(mq4V);
  if (mq4ZeroCount > 200) { errCount++; warn = true; addReason("w", "MQ-4 stuck at 0 - check wiring"); }
  else if (!mq4WarmedUp) { warmCount++; addReason("i", "MQ-4 warming up (" + String(warmupLeftSec(MQ4_WARMUP_MS)) + " s)"); }
  else okCount++;
  if (mq4WarmedUp && strcmp(mq4Lvl, "HIGH") == 0) { warn = true; addReason("w", "MQ-4 methane reading HIGH (uncalibrated)"); }

  // 5) MQ-7 (classification unchanged)
  const char* mq7Lvl = gasLevel(mq7V);
  if (mq7ZeroCount > 200) { errCount++; warn = true; addReason("w", "MQ-7 stuck at 0 - check wiring"); }
  else if (!mq7WarmedUp) { warmCount++; addReason("i", "MQ-7 warming up (" + String(warmupLeftSec(MQ7_WARMUP_MS)) + " s)"); }
  else okCount++;
  if (mq7WarmedUp && strcmp(mq7Lvl, "HIGH") == 0) { warn = true; addReason("w", "MQ-7 CO reading HIGH (uncalibrated)"); }

  // 6) HC-SR04 - "no echo" is normal in open space, only flag if it lasts
  bool hcLongNoEcho = (hcDist < 0) && ((hcLastValidTime == 0) ? (now - bootTime > 30000) : (now - hcLastValidTime > 30000));
  if (!hcLongNoEcho) okCount++;
  else { warnCount++; addReason("i", "HC-SR04: no echo for 30+ s (open space or wiring)"); }

  // 7) VL53L0X
  if (vl53CurrentState == VL53_VALID) okCount++;
  else if (vl53CurrentState == VL53_STALE || vl53CurrentState == VL53_RECOVERING) { warnCount++; addReason("i", "VL53L0X " + vlSt); }
  else { errCount++; warn = true; addReason("w", "VL53L0X " + vlSt); }

  // 8) MPU6050
  bool mpuStaleLong = (mpuLastValidTime == 0) || (now - mpuLastValidTime > 30000);
  if (mpuFound && mpuReadOK) okCount++;
  else if (!mpuFound || mpuStaleLong) { errCount++; warn = true; addReason("w", mpuFound ? "MPU6050 read errors" : "MPU6050 not found - tilt safety unavailable"); }
  else warnCount++;
  bool unsafeTilt = mpuFound && mpuReadOK && (abs(pitch) > TILT_LIMIT_DEG || abs(roll) > TILT_LIMIT_DEG);
  if (unsafeTilt) { crit = true; addReason("c", "Unsafe chassis tilt (beyond ±45°)"); }

  reasons += "]";
  const char* health = crit ? "CRITICAL" : (warn ? "WARNING" : "NORMAL");

  // ---------------- JSON ----------------
  String json; json.reserve(3200);
  json = "{";
  json += "\"uptime\":" + String((now - bootTime) / 1000) + ",";
  json += "\"health\":\"" + String(health) + "\",\"okCount\":" + String(okCount) + ",\"warnCount\":" + String(warnCount) + ",\"errCount\":" + String(errCount) + ",\"warmCount\":" + String(warmCount) + ",\"totalCount\":" + String(totalCount) + ",";
  json += "\"reasons\":" + reasons + ",";
  json += "\"tempWarn\":" + String(TEMP_WARN_C, 1) + ",\"tempCrit\":" + String(TEMP_CRIT_C, 1) + ",";
  json += "\"servoAngle\":" + String(servoAngle) + ",";
  json += "\"wifiClients\":" + String(WiFi.softAPgetStationNum()) + ",";
  json += "\"detectionSources\":\"Human-capable radar: 1/1 \\u2014 LD2410B (" + String(LD2410_MAX_RANGE_CM / 100.0, 2) + " m zone)\",";

  json += "\"motor\":{\"state\":\"" + motorState + "\",\"cmdPWM\":" + String(commandPWM) + ",\"leftPWM\":" + String(leftPWM) + ",\"rightPWM\":" + String(rightPWM) + ",\"speedMode\":\"" + speedMode + "\"},";

  json += "\"dht\":{\"temp\":" + String(dhtValidNow ? dhtTemp : lastValidTemp, 1) + ",\"hum\":" + String(dhtValidNow ? dhtHum : lastValidHum, 1) + ",\"valid\":" + String(dhtValidNow ? "true" : "false") + ",\"everValid\":" + String(dhtLastValidTime != 0 ? "true" : "false") + ",\"stale\":" + String(dhtStaleLong ? "true" : "false") + ",\"age\":" + String(dhtAge, 1) + ",\"problem\":\"" + (dhtValidNow ? "" : "READ ERROR - showing last valid") + "\"},";

  json += "\"mq2\":{\"raw\":" + String(mq2Raw) + ",\"v\":" + String(mq2V, 2) + ",\"filt\":" + String(mq2Filtered, 3) + ",\"base\":" + String(mq2Baseline, 3) + ",\"delta\":" + String(mq2Delta, 3)
        + ",\"lvl\":\"" + mq2StateString() + "\",\"warmLeft\":" + String(warmupLeftSec(MQ2_WARMUP_MS)) + ",\"warmTotal\":" + String(MQ2_WARMUP_MS / 1000)
        + ",\"warnDelta\":" + String(MQ2_WARN_DELTA_V, 2) + ",\"highAbs\":" + String(MQ2_HIGH_ABS_V, 2) + ",\"highDelta\":" + String(MQ2_HIGH_DELTA_V, 2)
        + ",\"problem\":\"" + mqErrorFlag(mq2ZeroCount) + "\"},";
  json += "\"mq4\":{\"raw\":" + String(mq4Raw) + ",\"v\":" + String(mq4V, 2) + ",\"lvl\":\"" + mq4Lvl + "\",\"warmedUp\":" + String(mq4WarmedUp ? "true" : "false") + ",\"warmLeft\":" + String(warmupLeftSec(MQ4_WARMUP_MS)) + ",\"problem\":\"" + mqErrorFlag(mq4ZeroCount) + "\"},";
  json += "\"mq7\":{\"raw\":" + String(mq7Raw) + ",\"v\":" + String(mq7V, 2) + ",\"lvl\":\"" + mq7Lvl + "\",\"warmedUp\":" + String(mq7WarmedUp ? "true" : "false") + ",\"warmLeft\":" + String(warmupLeftSec(MQ7_WARMUP_MS)) + ",\"problem\":\"" + mqErrorFlag(mq7ZeroCount) + "\"},";

  json += "\"hc\":{\"dist\":" + (hcDist < 0 ? String("null") : String(hcDist, 1)) + ",\"valid\":" + String(hcDist >= 0 ? "true" : "false") + ",\"angle\":" + String(hcAngleAtReading) + ",\"lastValid\":" + (hcLastValid < 0 ? String("null") : String(hcLastValid, 1)) + ",\"age\":" + String(hcAge, 1) + ",\"longNoEcho\":" + String(hcLongNoEcho ? "true" : "false") + ",\"problem\":\"" + (hcDist < 0 ? "NO ECHO" : "") + "\"},";

  json += "\"vl53\":{\"raw\":" + (vl53RawMM < 0 ? String("null") : String(vl53RawMM)) + ",\"filtered\":" + (vl53FilteredMM < 0 ? String("null") : String(vl53FilteredMM)) + ",\"lastValid\":" + (vl53LastValidMM < 0 ? String("null") : String(vl53LastValidMM)) + ",\"age\":" + String(vlAge, 1) + ",\"status\":\"" + vlSt + "\",\"noObject\":" + String(vl53NoObject ? "true" : "false") + ",\"addr\":\"0x29\",\"addrFound\":" + String(vl53DeviceFound ? "true" : "false") + ",\"fails\":" + String(vl53ConsecutiveFails) + "},";

  json += "\"mpu\":{\"pitch\":" + (mpuReadOK ? String(pitch, 1) : String("null")) + ",\"roll\":" + (mpuReadOK ? String(roll, 1) : String("null")) + ",\"valid\":" + String(mpuReadOK ? "true" : "false") + ",\"found\":" + String(mpuFound ? "true" : "false") + ",\"tiltSafe\":" + String((mpuFound && mpuReadOK && abs(pitch) <= TILT_LIMIT_DEG && abs(roll) <= TILT_LIMIT_DEG) ? "true" : "false") + "},";

  json += "\"ld2410\":{\"targetState\":" + String(ld2410TargetState) + ",\"targetType\":\"" + targetType + "\",\"movingDist\":" + String(ld2410MovingDist) + ",\"movingEnergy\":" + String(ld2410MovingEnergy) + ",\"stillDist\":" + String(ld2410StillDist) + ",\"stillEnergy\":" + String(ld2410StillEnergy) + ",\"detectDist\":" + String(ld2410DetectDist)
        + ",\"uartFresh\":" + String(ld2410Fresh ? "true" : "false") + ",\"everRx\":" + String(ld2410EverReceived ? "true" : "false") + ",\"age\":" + String(ld2410Age, 1) + ",\"outPin\":\"" + (ld2410OutState == HIGH ? "HIGH" : "LOW") + "\""
        + ",\"presence\":" + String(presenceNow ? "true" : "false") + ",\"rescueState\":\"" + rescueStateString(ld2410Fresh) + "\""
        + ",\"inZone\":" + String((ld2410Fresh && ld2410FrameInRange) ? "true" : "false") + ",\"outOfRange\":" + String((ld2410Fresh && ld2410FrameOutOfRange) ? "true" : "false")
        + ",\"targetDist\":" + String(ld2410TargetDistCm) + ",\"zoneType\":\"" + zoneType + "\",\"maxRangeCm\":" + String(LD2410_MAX_RANGE_CM) + ",\"onFrames\":" + String(PRESENCE_ON_FRAMES)
        + ",\"cfgAck\":" + String(ld2410CfgAck ? "true" : "false") + ",\"gateOk\":" + String(ld2410GateOk ? "true" : "false")
        + ",\"resCm\":" + String(ld2410ResCm) + ",\"hwGate\":" + String(ld2410HwGate) + ",\"cfgNote\":\"" + ld2410CfgNote + "\"},";

  json += "\"buzzer\":{\"muted\":" + String(buzzerMuted ? "true" : "false") + ",\"active\":" + String((presenceNow && !buzzerMuted) ? "true" : "false")
        + ",\"armed\":" + String(presenceNow ? "true" : "false") + "}";
  json += "}";
  server.send(200, "application/json", json);
}

// ==================== SETUP (UNCHANGED apart from log text) ====================
void setup() {
  Serial.begin(115200); delay(1500); bootTime = millis();
  Serial.println("+========================================================+");
  Serial.println("|              FIRE RESCUE ROVER - BOOT                 |");
  Serial.println("+========================================================+");

  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);
  pinMode(LD2410_OUT, INPUT_PULLDOWN); pinMode(MPU_INT, INPUT);
  pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, LOW);

  setupMotors(); setupServo(); setServo(0);
  lastDriveCmdTime = millis();

  LD2410Serial.begin(256000, SERIAL_8N1, LD2410_RX2, LD2410_TX2);
  configureLD2410();
  Serial.print("[INIT] LD2410B config ACK: "); Serial.print(ld2410CfgAck ? "YES" : "NO");
  Serial.print("  resolution: "); Serial.print(ld2410ResCm); Serial.print(" cm/gate  max gate: "); Serial.println(ld2410HwGate);
  Serial.print("[INIT] LD2410B: "); Serial.println(ld2410CfgNote);
  Serial.print("[INIT] LD2410B hard detection limit: "); Serial.print(LD2410_MAX_RANGE_CM); Serial.println(" cm");

  Wire.begin(I2C_SDA, I2C_SCL); Wire.setClock(100000);
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0); Wire.endTransmission(true);

  scanI2C();
  Serial.print("[INIT] MPU6050 : "); Serial.println(mpuFound ? "OK" : "NOT FOUND");
  tryInitVL53();
  Serial.print("[INIT] VL53L0X : "); Serial.println(vl53StatusString());

  WiFi.softAP(ssid, password);
  Serial.print("[WiFi] IP: http://"); Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/forward", handleForward); server.on("/backward", handleBackward);
  server.on("/left", handleLeft); server.on("/right", handleRight); server.on("/stop", handleStop);
  server.on("/data", handleData); server.on("/mutebuzzer", handleMuteBuzzer);
  server.on("/buzzer", handleBuzzerToggle);
  server.on("/speed", handleSpeed);
  server.begin();

  Serial.println("Ready. MQ-7 warm-up ~20 s, MQ-2 warm-up/baseline ~60 s (both non-blocking).\n");
}

// ==================== MAIN LOOP (UNCHANGED) ====================
void loop() {
  server.handleClient();
  unsigned long now = millis();

  ld2410OutState = digitalRead(LD2410_OUT); // informational only - never overrides validated UART detection

  readLD2410Frame();
  updatePresence();
  updateBuzzer();
  driveWatchdogCheck();

  if (now - tDht >= 2000) { tDht = now; updateDHT(); }
  if (now - tGas >= 300) { tGas = now; updateGas(); }
  if (now - tHc >= 150) { tHc = now; updateHC(); }
  updateServoSweep();
  if (now - tTof >= 150) { tTof = now; updateVL53(); }
  if (now - tMpu >= 75) { tMpu = now; if (mpuFound) readMPU(); }
  if (now - tDash >= 1000) { tDash = now; printDashboard(); }
}

// ==================== SERIAL DASHBOARD ====================
void printDashboard() {
  Serial.println("+========================================================+");
  Serial.print("| Uptime: "); Serial.print((millis() - bootTime) / 1000);
  Serial.print("s  Motor: "); Serial.println(motorState);
  Serial.println("+========================================================+");
  Serial.print("[DHT22] "); Serial.println(dhtValidNow ? (String(dhtTemp, 1) + "C " + String(dhtHum, 1) + "%") : "ERROR - using last valid");
  Serial.print("[MQ2] raw:"); Serial.print(mq2Raw); Serial.print(" filt:"); Serial.print(mq2Filtered, 2);
  Serial.print("V base:"); Serial.print(mq2Baseline, 2); Serial.print("V d:"); Serial.print(mq2Delta, 2);
  Serial.print("V "); Serial.print(mq2StateString()); Serial.print(" "); Serial.println(mqErrorFlag(mq2ZeroCount));
  Serial.print("[MQ4] "); Serial.print(mq4WarmedUp ? String(mq4Raw) : String("WARMING UP")); Serial.print(" "); Serial.println(mqErrorFlag(mq4ZeroCount));
  Serial.print("[MQ7] "); Serial.println(mq7WarmedUp ? String(mq7Raw) : "WARMING UP");
  Serial.print("[HC-SR04] "); Serial.println(hcDist < 0 ? "NO ECHO" : String(hcDist, 1) + " cm @ " + String(hcAngleAtReading) + "deg");
  Serial.print("[VL53L0X] "); Serial.println(vl53StatusString());
  Serial.print("[MPU6050] ");
  if (!mpuFound) Serial.println("NOT AVAILABLE");
  else if (!mpuReadOK) Serial.println("READ ERROR");
  else { Serial.print("Pitch:"); Serial.print(pitch, 1); Serial.print(" Roll:"); Serial.println(roll, 1); }
  Serial.print("[LD2410B] TargetState:"); Serial.print(ld2410TargetState);
  Serial.print(" MovingDist:"); Serial.print(ld2410MovingDist);
  Serial.print("cm StillDist:"); Serial.print(ld2410StillDist);
  Serial.print("cm Zone(<="); Serial.print(LD2410_MAX_RANGE_CM); Serial.print("cm):"); Serial.print(ld2410FrameInRange ? "IN" : (ld2410FrameOutOfRange ? "OUTSIDE" : "-"));
  Serial.print(" OUT:"); Serial.print(ld2410OutState == HIGH ? "HIGH" : "LOW");
  Serial.print("  Buzzer:"); Serial.print((presenceNow && !buzzerMuted) ? "ON" : (buzzerMuted ? "SILENCED" : "OFF"));
  Serial.print("  -> "); Serial.println(presenceNow ? ">>> TARGET DETECTED (in zone) <<<" : "AREA CLEAR");
  Serial.println("+========================================================+\n");
}
