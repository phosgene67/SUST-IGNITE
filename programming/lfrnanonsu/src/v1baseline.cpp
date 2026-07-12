// =============================================================
//  LINE FOLLOWER - VERSION 1: BASELINE (16-channel MUX sensor)
// =============================================================
//  Sensor hardware changed: 16 IR sensors -> one 16-channel analog
//  mux (e.g. CD74HC4067) -> single analog pin. Wiring given:
//    VCC -> 5V     GND -> GND     EN -> GND (mux always enabled)
//    SIG -> A0     S0 -> A4   S1 -> A3   S2 -> A2   S3 -> A1
//
//  IMPORTANT hardware-correctness note: with 8 independent ADC
//  channels you never needed a "settling delay" because each
//  channel was already electrically stable on its own pin. Now
//  ALL 16 sensors share ONE analog trace through a CMOS mux. When
//  you switch the mux address, the SIG line has to physically
//  charge/discharge to the new sensor's voltage before the ADC
//  samples it. Skip this delay and you'll read a blend of the
//  previous and current sensor - this is the #1 cause of "ghost"
//  or noisy readings on DIY mux sensor arrays. MUX_SETTLE_US
//  below is a safe starting point; you can try lowering it once
//  the robot is tuned, but don't remove it.
// =============================================================

#include <Arduino.h>

// ===== PIN DEFINITIONS =====
#define PWMA 3
#define AI1 7
#define AI2 6
#define STBY 8
#define PWMB 10
#define BI1 11
#define BI2 12

#define BTN_START 5
#define BTN_STOP 2
#define BTN_CALIBRATE 9

// ---- Mux sensor pins ----
#define MUX_SIG A0
#define MUX_S0 A4
#define MUX_S1 A3
#define MUX_S2 A2
#define MUX_S3 A1
#define MUX_SETTLE_US 10   // time for SIG to settle after switching channel

#define NUM_SENSORS 16
#define MAX_SPEED 200

// ===== GLOBAL STATE =====
uint16_t sensorMin[NUM_SENSORS];
uint16_t sensorMax[NUM_SENSORS];
uint16_t threshold[NUM_SENSORS];
int16_t lastPosition = 0;

enum RobotState { STATE_IDLE, STATE_CALIBRATING, STATE_RUNNING, STATE_STOPPED };
RobotState currentState = STATE_IDLE;

enum TDirection { T_LEFT, T_RIGHT, T_STRAIGHT };
TDirection tJunctionDirection = T_LEFT;

bool allBlackDetected = false;
uint32_t overshootStartTime = 0;
#define OVERSHOOT_TIME 150

bool deadEndArmed = false;
uint32_t deadEndArmTime = 0;
#define DEADEND_OVERSHOOT_TIME 200

float kp = 0.40;
float ki = 0.0;
float kd = 0.60;
float integral = 0;
int16_t lastError = 0;

// 16 weights, symmetric, step of 100, no sensor sits exactly on 0
// (normal for an even sensor count - the "center" is the gap
// between sensor 7 and sensor 8).
const float weights[NUM_SENSORS] = {
  -750, -650, -550, -450, -350, -250, -150, -50,
    50,  150,  250,  350,  450,  550,  650, 750
};

// Which 4 sensors count as the "dead-end center stripe" marker.
// NOTE: this was tuned for the old 8-sensor spacing (center 4 of 8).
// With 16 sensors likely packed into the same physical array width,
// the same physical stripe may now cover MORE than 4 sensors. Test
// this on your real track and widen the range below if needed.
#define DEADEND_LOW 6
#define DEADEND_HIGH 9   // inclusive; sensors 6,7,8,9

// ===== MUX SELECT + SENSOR READ (naive, readable) =====
void SelectMuxChannel(uint8_t ch) {
  digitalWrite(MUX_S0, (ch >> 0) & 0x01);
  digitalWrite(MUX_S1, (ch >> 1) & 0x01);
  digitalWrite(MUX_S2, (ch >> 2) & 0x01);
  digitalWrite(MUX_S3, (ch >> 3) & 0x01);
}

void ReadSensors(uint16_t *values) {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    SelectMuxChannel(i);
    delayMicroseconds(MUX_SETTLE_US);
    values[i] = analogRead(MUX_SIG);
  }
}

// ===== LINE DETECTION =====
int16_t DetectLine(uint16_t *raw) {
  bool onLine[NUM_SENSORS];
  uint8_t blackCount = 0;

  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    onLine[i] = (raw[i] > threshold[i]);
    if (onLine[i]) blackCount++;
  }

  if (blackCount == NUM_SENSORS) return 9999;
  if (blackCount == 0) return -9999;

  // Exactly the center 4 on, nothing else - blackCount check makes
  // this equivalent to (and simpler than) checking all 16 flags.
  if (blackCount == 4 &&
      onLine[DEADEND_LOW] && onLine[DEADEND_LOW + 1] &&
      onLine[DEADEND_LOW + 2] && onLine[DEADEND_HIGH]) {
    return 8888;
  }

  float weightedSum = 0;
  float sumValues = 0;
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    if (onLine[i]) {
      float value = raw[i] - threshold[i];
      weightedSum += weights[i] * value;
      sumValues += value;
    }
  }

  int16_t position = (int16_t)(weightedSum / sumValues);
  lastPosition = position;
  return position;
}

// ===== MOTOR CONTROL =====
void SetMotors(int16_t rightSpeed, int16_t leftSpeed) {
  rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);
  leftSpeed  = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);

  digitalWrite(AI1, rightSpeed >= 0 ? HIGH : LOW);
  digitalWrite(AI2, rightSpeed >= 0 ? LOW : HIGH);
  analogWrite(PWMA, abs(rightSpeed));

  digitalWrite(BI1, leftSpeed >= 0 ? HIGH : LOW);
  digitalWrite(BI2, leftSpeed >= 0 ? LOW : HIGH);
  analogWrite(PWMB, abs(leftSpeed));
}

void StopMotors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  digitalWrite(AI1, LOW);
  digitalWrite(AI2, LOW);
  digitalWrite(BI1, LOW);
  digitalWrite(BI2, LOW);
}

void EnableDriver()  { digitalWrite(STBY, HIGH); }
void DisableDriver() { digitalWrite(STBY, LOW); }

// ===== CALIBRATION =====
void Calibrate() {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    sensorMin[i] = 1023;
    sensorMax[i] = 0;
  }

  uint16_t raw[NUM_SENSORS];
  EnableDriver();
  SetMotors(70, -70);

  for (uint16_t sample = 0; sample < 1500; sample++) {
    ReadSensors(raw);
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
      if (raw[i] < sensorMin[i]) sensorMin[i] = raw[i];
      if (raw[i] > sensorMax[i]) sensorMax[i] = raw[i];
    }
    delay(5);
  }

  StopMotors();
  DisableDriver();

  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    threshold[i] = (sensorMin[i] + sensorMax[i]) / 2;
  }
}

// ===== T-JUNCTION / DEAD-END HANDLERS (blocking) =====
void HandleTJunction() {
  StopMotors();
  delay(50);

  if (tJunctionDirection == T_LEFT) {
    SetMotors(-150, 150);
    delay(300);
  } else if (tJunctionDirection == T_RIGHT) {
    SetMotors(150, -150);
    delay(300);
  } else {
    SetMotors(MAX_SPEED, MAX_SPEED);
    delay(200);
  }

  StopMotors();
  delay(50);
  allBlackDetected = false;
  overshootStartTime = 0;
}

void HandleDeadEnd() {
  StopMotors();
  delay(50);

  if (tJunctionDirection == T_RIGHT) {
    SetMotors(-180, 180);
  } else {
    SetMotors(180, -180);
  }
  delay(600);

  StopMotors();
  delay(50);
  deadEndArmed = false;
}

// ===== PID CONTROL =====
void RunPIDControl() {
  uint16_t raw[NUM_SENSORS];
  ReadSensors(raw);
  int16_t position = DetectLine(raw);
  uint32_t now = millis();

  if (position == 9999 && !allBlackDetected && !deadEndArmed) {
    allBlackDetected = true;
    overshootStartTime = now;
  }
  if (allBlackDetected) {
    if (position == -9999 && (now - overshootStartTime) >= OVERSHOOT_TIME) {
      HandleTJunction();
      return;
    }
    if ((now - overshootStartTime) > 1000) allBlackDetected = false;
  }

  if (position == 8888 && !deadEndArmed && !allBlackDetected) {
    deadEndArmed = true;
    deadEndArmTime = now;
  }
  if (deadEndArmed) {
    if (position == -9999 && (now - deadEndArmTime) >= DEADEND_OVERSHOOT_TIME) {
      HandleDeadEnd();
      return;
    }
    if ((now - deadEndArmTime) > 1000) deadEndArmed = false;
  }

  if (position == 9999 || position == -9999 || position == 8888) {
    position = lastError;
  }

  int16_t error = position;
  integral += error;
  integral = constrain(integral, -5000, 5000);
  int16_t derivative = error - lastError;

  float correction = kp * error + ki * integral + kd * derivative;

  int16_t rightSpeed = MAX_SPEED - (int16_t)correction;
  int16_t leftSpeed  = MAX_SPEED + (int16_t)correction;

  SetMotors(rightSpeed, leftSpeed);
  lastError = error;
}

// ===== BUTTON HANDLING (edge detect + debounce) =====
uint8_t lastBtnState[3] = {HIGH, HIGH, HIGH};
uint32_t lastBtnChangeTime[3] = {0, 0, 0};
#define DEBOUNCE_MS 25

bool ButtonPressed(uint8_t pin, uint8_t index) {
  uint8_t current = digitalRead(pin);
  uint32_t now = millis();
  bool pressed = false;
  if (current != lastBtnState[index] && (now - lastBtnChangeTime[index]) > DEBOUNCE_MS) {
    lastBtnChangeTime[index] = now;
    lastBtnState[index] = current;
    pressed = (current == LOW);
  }
  return pressed;
}

// ===== SETUP =====
void setup() {
  pinMode(PWMA, OUTPUT);
  pinMode(AI1, OUTPUT);
  pinMode(AI2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BI1, OUTPUT);
  pinMode(BI2, OUTPUT);
  pinMode(STBY, OUTPUT);

  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_S3, OUTPUT);
  // MUX_SIG (A0) stays an input by default - that's the analogRead pin.
  // EN is wired straight to GND, so no pin/code needed for it.

  pinMode(BTN_CALIBRATE, INPUT_PULLUP);
  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(BTN_STOP, INPUT_PULLUP);

  DisableDriver();
  StopMotors();
  currentState = STATE_IDLE;
}

// ===== MAIN LOOP =====
void loop() {
  bool calibratePressed = ButtonPressed(BTN_CALIBRATE, 0);
  bool startPressed = ButtonPressed(BTN_START, 1);
  bool stopPressed = ButtonPressed(BTN_STOP, 2);

  if (calibratePressed && currentState == STATE_IDLE) {
    currentState = STATE_CALIBRATING;
    Calibrate();
    currentState = STATE_STOPPED;
  }

  if (startPressed && (currentState == STATE_STOPPED || currentState == STATE_IDLE)) {
    currentState = STATE_RUNNING;
    EnableDriver();
    integral = 0;
    lastError = 0;
  }

  if (stopPressed && currentState == STATE_RUNNING) {
    currentState = STATE_STOPPED;
    StopMotors();
    DisableDriver();
  }

  if (currentState == STATE_RUNNING) {
    RunPIDControl();
  }
}
