
#include<Arduino.h>


// ===== PIN DEFINITIONS (matches your wiring diagram exactly) =====
// Motor Driver Pins (TB6612FNG)
#define PWMA 3      // Right motor PWM
#define AI1 7       // Right motor direction 1 (AIN1)
#define AI2 6       // Right motor direction 2 (AIN2)
#define STBY 8      // Driver standby
#define PWMB 10     // Left motor PWM
#define BI1 11      // Left motor direction 1 (BIN1)
#define BI2 12      // Left motor direction 2 (BIN2)

// Sensors: A0-A7 = Sensor1-Sensor8, direct ADC access
// IR array enable: hardwired to 3.3V, always ON - no pin needed

// Button Pins (momentary, active LOW, internal pull-up)
#define BTN_START 5     // Start/Resume
#define BTN_STOP 2       // Stop (keeps calibration)
#define BTN_CALIBRATE 9  // Calibrate

// ===== GLOBAL CONSTANTS =====
#define NUM_SENSORS 8
#define MAX_SPEED 200

// ===== GLOBAL VARIABLES =====
uint16_t sensorMin[NUM_SENSORS];
uint16_t sensorMax[NUM_SENSORS];
uint16_t threshold[NUM_SENSORS];
int16_t lastPosition = 0;

enum RobotState { STATE_IDLE, STATE_CALIBRATING, STATE_RUNNING, STATE_STOPPED };
volatile RobotState currentState = STATE_IDLE;

enum TDirection { T_LEFT, T_RIGHT, T_STRAIGHT };
TDirection tJunctionDirection = T_LEFT;  // <-- set your T-junction turn preference

bool allBlackDetected = false;
uint32_t overshootStartTime = 0;
#define OVERSHOOT_TIME 150

bool deadEndArmed = false;
uint32_t deadEndArmTime = 0;
#define DEADEND_OVERSHOOT_TIME 200

// PID (scaled integers, no floats)
int16_t kp = 40;
int16_t ki = 0;
int16_t kd = 60;
int16_t integral = 0;
int16_t lastError = 0;

// ===== FAST DIRECT ADC READ (sensor reads only - this is the hot path) =====
inline void ReadSensors(uint16_t* values) {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    ADMUX = (1 << REFS0) | i;   // AVcc reference, channel i (A0-A7)
    ADCSRA |= (1 << ADSC);      // start conversion
    while (ADCSRA & (1 << ADSC)); // wait
    values[i] = ADC;
  }
}

// ===== LINE DETECTION =====
// Returns weighted position -700..+700
// 9999 = all black, -9999 = all white, 8888 = straight dead-end pattern
int16_t DetectLine(uint16_t* raw) {
  uint8_t blackCount = 0;
  bool onLine[NUM_SENSORS];

  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    onLine[i] = (raw[i] > threshold[i]);
    if (onLine[i]) blackCount++;
  }

  if (blackCount == NUM_SENSORS) return 9999;
  if (blackCount == 0) return -9999;

  if (onLine[2] && onLine[3] && onLine[4] && onLine[5] &&
      !onLine[0] && !onLine[1] && !onLine[6] && !onLine[7]) {
    return 8888;
  }

  static const int16_t weights[8] = {-700, -500, -300, -100, 100, 300, 500, 700};
  int32_t weightedSum = 0;
  int16_t sumValues = 0;

  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    if (onLine[i]) {
      int16_t value = raw[i] - threshold[i];
      weightedSum += (int32_t)weights[i] * value;
      sumValues += value;
    }
  }

  if (sumValues == 0) return lastPosition;

  int16_t position = (int16_t)(weightedSum / sumValues);
  lastPosition = position;
  return position;
}

// ===== MOTOR CONTROL (plain digitalWrite - safe, no pin/port mismatches) =====
inline void SetMotors(int16_t rightSpeed, int16_t leftSpeed) {
  if (rightSpeed > MAX_SPEED) rightSpeed = MAX_SPEED;
  if (rightSpeed < -MAX_SPEED) rightSpeed = -MAX_SPEED;
  if (leftSpeed > MAX_SPEED) leftSpeed = MAX_SPEED;
  if (leftSpeed < -MAX_SPEED) leftSpeed = -MAX_SPEED;

  if (rightSpeed >= 0) {
    digitalWrite(AI1, HIGH);
    digitalWrite(AI2, LOW);
  } else {
    digitalWrite(AI1, LOW);
    digitalWrite(AI2, HIGH);
    rightSpeed = -rightSpeed;
  }
  analogWrite(PWMA, rightSpeed);

  if (leftSpeed >= 0) {
    digitalWrite(BI1, HIGH);
    digitalWrite(BI2, LOW);
  } else {
    digitalWrite(BI1, LOW);
    digitalWrite(BI2, HIGH);
    leftSpeed = -leftSpeed;
  }
  analogWrite(PWMB, leftSpeed);
}

inline void StopMotors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  digitalWrite(AI1, LOW);
  digitalWrite(AI2, LOW);
  digitalWrite(BI1, LOW);
  digitalWrite(BI2, LOW);
}

inline void EnableDriver() { digitalWrite(STBY, HIGH); }
inline void DisableDriver() { digitalWrite(STBY, LOW); }

// ===== CALIBRATION =====
void Calibrate() {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    sensorMin[i] = 1023;
    sensorMax[i] = 0;
  }

  uint16_t raw[NUM_SENSORS];
  EnableDriver();
  SetMotors(70, -70); // right fwd, left rev -> rotate

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
    threshold[i] = (sensorMin[i] + sensorMax[i]) >> 1;
  }
}

// ===== T-JUNCTION HANDLER =====
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

// ===== DEAD-END 180 TURN HANDLER =====
void HandleDeadEnd() {
  StopMotors();
  delay(50);

  if (tJunctionDirection == T_RIGHT) {
    SetMotors(-180, 180); // CW
  } else {
    SetMotors(180, -180); // CCW
  }
  delay(600); // tune for exact 180 degrees

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

  // T-junction detection
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

  // Dead-end detection
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

  // Normal PID (skip special codes)
  if (position == 9999 || position == -9999 || position == 8888) {
    position = lastError;
  }

  int16_t error = position;
  integral += error;
  if (integral > 5000) integral = 5000;
  if (integral < -5000) integral = -5000;
  int16_t derivative = error - lastError;

  int32_t correction = ((int32_t)kp * error + (int32_t)ki * integral + (int32_t)kd * derivative) / 100;

  int16_t rightSpeed = MAX_SPEED - correction;
  int16_t leftSpeed  = MAX_SPEED + correction;

  SetMotors(rightSpeed, leftSpeed);
  lastError = error;
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

  pinMode(BTN_CALIBRATE, INPUT_PULLUP);
  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(BTN_STOP, INPUT_PULLUP);

  ADCSRA = (1 << ADEN) | (1 << ADPS2); // enable ADC, prescaler 16

  DisableDriver();
  StopMotors();
  currentState = STATE_IDLE;

  // Serial.begin(115200); // uncomment to debug button states
}

// ===== BUTTON HANDLING (edge detection) =====
uint8_t lastBtnState[3] = {HIGH, HIGH, HIGH}; // 0=calibrate,1=start,2=stop

inline bool ButtonPressed(uint8_t pin, uint8_t index) {
  uint8_t current = digitalRead(pin);
  bool pressed = (current == LOW && lastBtnState[index] == HIGH);
  lastBtnState[index] = current;
  return pressed;
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
    // calibration data (threshold[]) is untouched here on purpose
  }

  if (currentState == STATE_RUNNING) {
    RunPIDControl();
  }
}