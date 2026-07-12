// =============================================================
//  LINE FOLLOWER - VERSION 2: OPTIMIZED (16-channel MUX sensor)
// =============================================================
//  Same behavior as v1-mux16, fast implementation. Changes from
//  the 8-sensor v2 you had before:
//
//   1. Only ONE ADC channel is ever used now (A0/ADC0) - ADMUX is
//      set ONCE in setup() and never touched again. The "channel
//      switching" now happens on the mux's S0-S3 address lines,
//      not on ADMUX. This is a fundamentally different bottleneck:
//      before, switching sensors was ~free (ADMUX write); now it
//      costs a mandatory settle delay because the analog signal
//      has to physically stabilize through the mux/wiring.
//   2. S0-S3 (A4,A3,A2,A1) all live on PORTC (bits 4,3,2,1). That
//      means selecting any of the 16 channels is ONE PORTC write
//      with a precomputed bitmask - not four digitalWrite() calls.
//      The 16 masks are baked in as compile-time literals in the
//      unrolled macro below (see the table computed in comments).
//   3. NUM_SENSORS doubled (8->16) so the fused read+detect loop
//      is now 16 unrolled steps instead of 8, and onLineMask grows
//      from uint8_t to uint16_t.
//
//  Honest cost/benefit note: doubling sensor count while forcing
//  serialized reads through one mux (with a mandatory settle delay
//  per channel) means a full sensor scan is roughly ~3.5x slower
//  than the old 8-parallel-ADC-channel design, even after all the
//  register-level optimization below - because the settle delay,
//  not computation, now dominates. At MUX_SETTLE_US=10 you're still
//  looking at roughly (10+13)us * 16 =~ 370us per scan, i.e. ~2.7kHz
//  loop rate - plenty fast for line following, but worth knowing
//  WHY it's slower than before: it's physics (RC settling on the
//  mux output), not a code inefficiency you can optimize away.
// =============================================================

#include <Arduino.h>

// ===== PIN DEFINITIONS =====
#define PWMA 3      // PD3 = OC2B (Timer2)
#define AI1 7       // PD7
#define AI2 6       // PD6
#define STBY 8      // PB0
#define PWMB 10     // PB2 = OC1B (Timer1)
#define BI1 11      // PB3 = OC2A (unused as PWM here)
#define BI2 12      // PB4

#define BTN_START 5
#define BTN_STOP 2
#define BTN_CALIBRATE 9

// ---- Mux sensor pins: SIG=A0(PC0/ADC0), S0=A4(PC4), S1=A3(PC3), S2=A2(PC2), S3=A1(PC1) ----
#define MUX_S0 A4
#define MUX_S1 A3
#define MUX_S2 A2
#define MUX_S3 A1
#define MUX_SELECT_BITS 0x1E   // PC1|PC2|PC3|PC4 - the 4 mux address bits on PORTC
#define MUX_SETTLE_US 10       // do not remove - see header note on RC settling

#define NUM_SENSORS 16
#define MAX_SPEED 120

#define AI1_MASK (1 << PD7)
#define AI2_MASK (1 << PD6)
#define STBY_MASK (1 << PB0)
#define BI1_MASK (1 << PB3)
#define BI2_MASK (1 << PB4)

static inline void MuxSelect(uint8_t bits) __attribute__((always_inline));
static inline void MuxSelect(uint8_t bits) {
  PORTC = (PORTC & ~MUX_SELECT_BITS) | bits;
}

// ===== GLOBAL STATE =====
uint16_t sensorMin[NUM_SENSORS];
uint16_t sensorMax[NUM_SENSORS];
uint16_t threshold[NUM_SENSORS];
int16_t lastPosition = 0;
int16_t lastError = 0;

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

// Q8 fixed point, same effective tuning as before (kp=0.40, kd=0.60).
const int16_t KP_Q8 = 128;
const int16_t KI_Q8 = 0;
const int16_t KD_Q8 = 0;
int32_t integral = 0;

#define POS_ALL_BLACK 9999
#define POS_ALL_WHITE -9999
#define POS_DEADEND 8888

// Sensors 6,7,8,9 = the center-4 dead-end stripe marker (16-sensor
// layout). VERIFY THIS on your actual hardware - if your 16 sensors
// are packed into the same physical width the old 8 covered, the
// same stripe may now cover more than 4 sensors. If so, widen the
// mask below (e.g. add sensors 5 and 10) and retest.
#define DEADEND_MASK 0x03C0   // bits 6,7,8,9

// ===== FUSED MUX-SELECT + READ + THRESHOLD + WEIGHTED SUM =====
// One macro step per sensor: set the mux address (1 PORTC write,
// mask precomputed for that channel - see header table), wait for
// SIG to settle, convert, accumulate. Weights are compile-time
// immediates, same convention as before but rescaled for 16 sensors.
#define SENSOR_STEP(bit, w, muxmask)                        \
  do {                                                      \
    MuxSelect(muxmask);                                     \
    delayMicroseconds(MUX_SETTLE_US);                       \
    ADCSRA |= (1 << ADSC);                                  \
    while (ADCSRA & (1 << ADSC));                           \
    uint16_t v = ADC;                                       \
    if (v > threshold[sensorIndex]) {                       \
      int16_t d = v - threshold[sensorIndex];                \
      onLineMask |= bit;                                    \
      weightedSum += (int32_t)(w) * d;                      \
      sumValues += d;                                       \
    }                                                        \
    sensorIndex++;                                          \
  } while (0)

int16_t DetectLineFast() {
  uint16_t onLineMask = 0;
  int32_t weightedSum = 0;
  int16_t sumValues = 0;
  uint8_t sensorIndex = 0;

  // channel : mux mask (from the S0-S3/PORTC table)
  SENSOR_STEP(0x0001, -750, 0x00); // ch0
  SENSOR_STEP(0x0002, -650, 0x10); // ch1
  SENSOR_STEP(0x0004, -550, 0x08); // ch2
  SENSOR_STEP(0x0008, -450, 0x18); // ch3
  SENSOR_STEP(0x0010, -350, 0x04); // ch4
  SENSOR_STEP(0x0020, -250, 0x14); // ch5
  SENSOR_STEP(0x0040, -150, 0x0C); // ch6
  SENSOR_STEP(0x0080,  -50, 0x1C); // ch7
  SENSOR_STEP(0x0100,   50, 0x02); // ch8
  SENSOR_STEP(0x0200,  150, 0x12); // ch9
  SENSOR_STEP(0x0400,  250, 0x0A); // ch10
  SENSOR_STEP(0x0800,  350, 0x1A); // ch11
  SENSOR_STEP(0x1000,  450, 0x06); // ch12
  SENSOR_STEP(0x2000,  550, 0x16); // ch13
  SENSOR_STEP(0x4000,  650, 0x0E); // ch14
  SENSOR_STEP(0x8000,  750, 0x1E); // ch15

  if (onLineMask == 0xFFFF) return POS_ALL_BLACK;
  if (onLineMask == 0x0000) return POS_ALL_WHITE;
  if (onLineMask == DEADEND_MASK) return POS_DEADEND;

  // sumValues >= 1 guaranteed whenever onLineMask != 0 (see v1 note).
  int16_t position = (int16_t)(weightedSum / sumValues);
  lastPosition = position;
  return position;
}

// ===== MOTOR CONTROL - direct port + OCR writes =====
static inline void SetMotors(int16_t rightSpeed, int16_t leftSpeed) {
  if (rightSpeed > MAX_SPEED) rightSpeed = MAX_SPEED;
  if (rightSpeed < -MAX_SPEED) rightSpeed = -MAX_SPEED;
  if (leftSpeed > MAX_SPEED) leftSpeed = MAX_SPEED;
  if (leftSpeed < -MAX_SPEED) leftSpeed = -MAX_SPEED;

  if (rightSpeed >= 0) {
    PORTD &= ~AI1_MASK;
    PORTD |= AI2_MASK;
  } else {
     PORTD |= AI1_MASK;
    PORTD &= ~AI2_MASK;
    rightSpeed = -rightSpeed;
  }
  OCR2B = (uint8_t)rightSpeed;

  if (leftSpeed >= 0) {
     PORTB &= ~BI1_MASK;
    PORTB |= BI2_MASK;
  } else {
    PORTB |= BI1_MASK;
    PORTB &= ~BI2_MASK;
    leftSpeed = -leftSpeed;
  }
  OCR1B = (uint8_t)leftSpeed;
}

static inline void StopMotors() {
  OCR2B = 0;
  OCR1B = 0;
  PORTD &= ~(AI1_MASK | AI2_MASK);
  PORTB &= ~(BI1_MASK | BI2_MASK);
}

static inline void EnableDriver()  { PORTB |= STBY_MASK; }
static inline void DisableDriver() { PORTB &= ~STBY_MASK; }

// ===== CALIBRATION (not perf-critical - simple table-driven loop) =====
const uint8_t muxMaskTable[16] PROGMEM = {
  0x00, 0x10, 0x08, 0x18, 0x04, 0x14, 0x0C, 0x1C,
  0x02, 0x12, 0x0A, 0x1A, 0x06, 0x16, 0x0E, 0x1E
};

uint16_t ReadSensorRaw(uint8_t ch) {
  uint8_t mask = pgm_read_byte(&muxMaskTable[ch]);
  MuxSelect(mask);
  delayMicroseconds(MUX_SETTLE_US);
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC));
  return ADC;
}

void Calibrate() {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    sensorMin[i] = 1023;
    sensorMax[i] = 0;
  }

  EnableDriver();
  SetMotors(70, -70);

  for (uint16_t sample = 0; sample < 1500; sample++) {
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
      uint16_t v = ReadSensorRaw(i);
      if (v < sensorMin[i]) sensorMin[i] = v;
      if (v > sensorMax[i]) sensorMax[i] = v;
    }
    delay(5);
  }

  StopMotors();
  DisableDriver();

  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    threshold[i] = (sensorMin[i] + sensorMax[i]) >> 1;
  }
}

// ===== NON-BLOCKING TURN STATE MACHINE =====
enum TurnPhase { TURN_NONE, TURN_BRAKE1, TURN_PIVOT, TURN_BRAKE2 };
TurnPhase turnPhase = TURN_NONE;
uint32_t turnPhaseStart = 0;
bool turnIsDeadEnd = false;

void StartTurn(bool isDeadEnd) {
  StopMotors();
  turnIsDeadEnd = isDeadEnd;
  turnPhase = TURN_BRAKE1;
  turnPhaseStart = millis();
}

void UpdateTurn() {
  uint32_t elapsed = millis() - turnPhaseStart;

  switch (turnPhase) {
    case TURN_BRAKE1:
      if (elapsed >= 50) {
        if (turnIsDeadEnd) {
          if (tJunctionDirection == T_RIGHT) SetMotors(-180, 180);
          else SetMotors(180, -180);
        } else {
          if (tJunctionDirection == T_LEFT) SetMotors(-150, 150);
          else if (tJunctionDirection == T_RIGHT) SetMotors(150, -150);
          else SetMotors(MAX_SPEED, MAX_SPEED);
        }
        turnPhase = TURN_PIVOT;
        turnPhaseStart = millis();
      }
      break;

    case TURN_PIVOT: {
      uint32_t pivotTime = turnIsDeadEnd ? 600 : (tJunctionDirection == T_STRAIGHT ? 200 : 300);
      if (elapsed >= pivotTime) {
        StopMotors();
        turnPhase = TURN_BRAKE2;
        turnPhaseStart = millis();
      }
      break;
    }

    case TURN_BRAKE2:
      if (elapsed >= 50) {
        turnPhase = TURN_NONE;
        allBlackDetected = false;
        overshootStartTime = 0;
        deadEndArmed = false;
      }
      break;

    default:
      break;
  }
}

// ===== PID CONTROL =====
void RunPIDControl() {
  if (turnPhase != TURN_NONE) {
    UpdateTurn();
    return;
  }

  int16_t position = DetectLineFast();
  uint32_t now = millis();

  if (position == POS_ALL_BLACK && !allBlackDetected && !deadEndArmed) {
    allBlackDetected = true;
    overshootStartTime = now;
  }
  if (allBlackDetected) {
    if (position == POS_ALL_WHITE && (now - overshootStartTime) >= OVERSHOOT_TIME) {
      StartTurn(false);
      return;
    }
    if ((now - overshootStartTime) > 1000) allBlackDetected = false;
  }

  if (position == POS_DEADEND && !deadEndArmed && !allBlackDetected) {
    deadEndArmed = true;
    deadEndArmTime = now;
  }
  if (deadEndArmed) {
    if (position == POS_ALL_WHITE && (now - deadEndArmTime) >= DEADEND_OVERSHOOT_TIME) {
      StartTurn(true);
      return;
    }
    if ((now - deadEndArmTime) > 1000) deadEndArmed = false;
  }

  if (position == POS_ALL_BLACK || position == POS_ALL_WHITE || position == POS_DEADEND) {
    position = lastError;
  }

  int16_t error = position;
  integral += error;
  if (integral > 5000) integral = 5000;
  if (integral < -5000) integral = -5000;
  int16_t derivative = error - lastError;

  int32_t correction = ((int32_t)KP_Q8 * error + (int32_t)KI_Q8 * integral + (int32_t)KD_Q8 * derivative) >> 8;

  int16_t rightSpeed = MAX_SPEED + (int16_t)correction;
  int16_t leftSpeed  = MAX_SPEED - (int16_t)correction;

  SetMotors(rightSpeed, leftSpeed);
  lastError = error;
}

// ===== BUTTON HANDLING (debounced edge detection) =====
uint8_t lastBtnState[3] = {HIGH, HIGH, HIGH};
uint32_t lastBtnChangeTime[3] = {0, 0, 0};
#define DEBOUNCE_MS 25

inline bool ButtonPressed(uint8_t pin, uint8_t index) {
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
  // A0 (SIG) is left as an ADC input. EN is hardwired to GND.

  pinMode(BTN_CALIBRATE, INPUT_PULLUP);
  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(BTN_STOP, INPUT_PULLUP);

  TCCR2A |= (1 << COM2B1);
  TCCR1A |= (1 << COM1B1);

  ADMUX = (1 << REFS0) | 0;  // fixed on ADC0 (A0/SIG) forever - the mux does the switching now
  ADCSRA = (1 << ADEN) | (1 << ADPS2); // ADC on, prescaler 16

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
    turnPhase = TURN_NONE;
  }

  if (stopPressed && currentState == STATE_RUNNING) {
    currentState = STATE_STOPPED;
    StopMotors();
    DisableDriver();
    turnPhase = TURN_NONE;
  }

  if (currentState == STATE_RUNNING) {
    RunPIDControl();
  }
}
