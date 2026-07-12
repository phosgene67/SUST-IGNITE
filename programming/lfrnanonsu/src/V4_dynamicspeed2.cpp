// =============================================================
//  LINE FOLLOWER - VERSION 3: 8-SENSOR NARROW WINDOW (from 16-ch MUX)
// =============================================================
//  Derived from the 16-channel mux version. Only 8 of the 16
//  physical mux channels are read now: the middle-dense window
//  OLD channels 4..11 (out of 0..15), which sits symmetrically
//  around the array's true center (between old ch7 and ch8).
//
//  Old-channel -> New-index map (for wiring/debug reference):
//    old ch4  -> new 0   (weight -350)
//    old ch5  -> new 1   (weight -250)
//    old ch6  -> new 2   (weight -150)
//    old ch7  -> new 3   (weight  -50)
//    old ch8  -> new 4   (weight  +50)
//    old ch9  -> new 5   (weight +150)
//    old ch10 -> new 6   (weight +250)
//    old ch11 -> new 7   (weight +350)
//
//  Everything else (mux wiring, PORTC mask trick, motor driver,
//  turn state machine, PID) is unchanged from the 16-ch version.
//
//  NEW IN THIS VERSION: dynamic speed control. Instead of driving
//  both wheels around a fixed base speed regardless of how sharp
//  the correction is, the base speed itself now scales down with
//  abs(error): straight/near-centered -> BASE_SPEED_MAX (full speed),
//  sharp correction -> down to BASE_SPEED_MIN. This should reduce
//  overshoot/oscillation on corners without sacrificing straight-
//  line speed.
//
//  Perf note: a full scan is 8 mux settle+convert steps instead
//  of 16, so scan time is roughly HALVED (~185us vs ~370us at
//  MUX_SETTLE_US=10), i.e. roughly ~5.4kHz loop rate instead of ~2.7kHz.
//
//  IMPORTANT HARDWARE CHECK: the DEADEND_MASK below assumes your
//  physical dead-end stripe still falls under old channels 6-9,
//  which are now new indices 2-5. If your 8 physically-read sensors
//  don't line up with where the dead-end marker actually is on the
//  track, you'll need to re-verify/re-derive this mask by testing.
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

#define NUM_SENSORS 8
#define MAX_SPEED 255   // hard safety ceiling for OCR2B/OCR1B (8-bit PWM, 0-255)

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

const int16_t KP_Q8 = 154;
const int16_t KI_Q8 = 0;
const int16_t KD_Q8 = 64;
int32_t integral = 0;

#define POS_ALL_BLACK 9999
#define POS_ALL_WHITE -9999
#define POS_DEADEND 8888

// ===== DYNAMIC SPEED CONTROL =====
// Two speed behaviors combined:
//  1. CORNERING (error above STRAIGHT_ERROR_THRESHOLD): speed drops
//     immediately (no ramp - this is a safety response) between
//     BASE_SPEED (at low-moderate error) down to MIN_SPEED (at the
//     sharpest possible error, ERROR_FULL_SCALE).
//  2. SMOOTH TRACK (error at/below STRAIGHT_ERROR_THRESHOLD): speed
//     does NOT jump straight to top speed. It climbs gradually,
//     +RAMP_STEP every RAMP_INTERVAL_MS, from BASE_SPEED up toward
//     CRUISE_MAX_SPEED, for as long as the track stays smooth. Any
//     corner resets this progress back down via case 1.
#define BASE_SPEED 180           // normal/default speed and corner-recovery ceiling
#define MIN_SPEED 150            // slowest speed, at the sharpest correction
#define CRUISE_MAX_SPEED 230     // top speed, only reached after sustained smooth track
#define ERROR_FULL_SCALE 350     // matches max |weight| in DetectLineFast
#define STRAIGHT_ERROR_THRESHOLD 40  // |error| at/below this counts as "smooth, no error"
#define RAMP_INTERVAL_MS 20      // how often cruiseSpeed is allowed to step up while smooth
#define RAMP_STEP 1              // speed units added per RAMP_INTERVAL_MS (keeps it gradual)

// Persists between control-loop calls so the ramp accumulates over
// time rather than resetting every scan.
int16_t cruiseSpeed = BASE_SPEED;
uint32_t lastRampTime = 0;

// Corner-recovery target: linear from BASE_SPEED (low error) down to
// MIN_SPEED (error at or beyond ERROR_FULL_SCALE). Applied immediately
// (not gradually) since slowing down for a sharp turn is a safety
// response, not something you want to ease into.
static inline int16_t ComputeCornerSpeed(int16_t error) __attribute__((always_inline));
static inline int16_t ComputeCornerSpeed(int16_t error) {
  int16_t absErr = (error < 0) ? (int16_t)(-error) : error;
  if (absErr > ERROR_FULL_SCALE) absErr = ERROR_FULL_SCALE;
  int32_t speed = BASE_SPEED -
      (((int32_t)(BASE_SPEED - MIN_SPEED) * absErr) / ERROR_FULL_SCALE);
  return (int16_t)speed;
}

// Dead-end stripe: OLD channels 6,7,8,9 -> NEW indices 2,3,4,5.
// VERIFY THIS on your actual hardware now that only 8 of the 16
// physical channels are being read - if the stripe doesn't line
// up with new indices 2-5, widen/shift this mask and retest.
#define DEADEND_MASK 0x3C   // bits 2,3,4,5

// ===== FUSED MUX-SELECT + READ + THRESHOLD + WEIGHTED SUM =====
// One macro step per sensor: set the mux address (1 PORTC write,
// mask precomputed for that channel), wait for SIG to settle,
// convert, accumulate.
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
  uint8_t onLineMask = 0;
  int32_t weightedSum = 0;
  int16_t sumValues = 0;
  uint8_t sensorIndex = 0;

  // new_index : (old_channel) : mux mask
  SENSOR_STEP(0x01, -350, 0x04); // new0 (old ch4)
  SENSOR_STEP(0x02, -250, 0x14); // new1 (old ch5)
  SENSOR_STEP(0x04, -150, 0x0C); // new2 (old ch6)
  SENSOR_STEP(0x08,  -50, 0x1C); // new3 (old ch7)
  SENSOR_STEP(0x10,   50, 0x02); // new4 (old ch8)
  SENSOR_STEP(0x20,  150, 0x12); // new5 (old ch9)
  SENSOR_STEP(0x40,  250, 0x0A); // new6 (old ch10)
  SENSOR_STEP(0x80,  350, 0x1A); // new7 (old ch11)

  if (onLineMask == 0xFF) return POS_ALL_BLACK;
  if (onLineMask == 0x00) return POS_ALL_WHITE;
  if (onLineMask == DEADEND_MASK) return POS_DEADEND;

  // sumValues >= 1 guaranteed whenever onLineMask != 0.
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
// Mux masks for new indices 0..7, mapped to old channels 4..11.
const uint8_t muxMaskTable[NUM_SENSORS] PROGMEM = {
  0x04, 0x14, 0x0C, 0x1C, 0x02, 0x12, 0x0A, 0x1A
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

  // Speed logic: smooth track -> gradually climb toward 255; any
  // real error -> drop back down immediately (no easing into slowdowns).
  int16_t absErr = (error < 0) ? (int16_t)(-error) : error;
  if (absErr <= STRAIGHT_ERROR_THRESHOLD) {
    if (now - lastRampTime >= RAMP_INTERVAL_MS) {
      lastRampTime = now;
      if (cruiseSpeed < CRUISE_MAX_SPEED) cruiseSpeed += RAMP_STEP;
    }
  } else {
    lastRampTime = now;
    cruiseSpeed = ComputeCornerSpeed(error);
  }

  int16_t rightSpeed = cruiseSpeed + (int16_t)correction;
  int16_t leftSpeed  = cruiseSpeed - (int16_t)correction;

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
    cruiseSpeed = BASE_SPEED;
    lastRampTime = millis();
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
