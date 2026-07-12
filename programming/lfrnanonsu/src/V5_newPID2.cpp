// =============================================================
//  ARDUINO NANO - 8-SENSOR PID LINE FOLLOWER (SPEED + ACCURACY)
//  16-CHANNEL MUX BOARD, READING 8 OF ITS 16 CHANNELS
// =============================================================
//  Hardware assumed (per your reference pin-map):
//    - Motor driver: TB6612FNG-style (PWMA/AI1/AI2/STBY/PWMB/BI1/BI2)
//    - 16-channel analog mux (e.g. CD74HC4067): SIG -> A0 (single
//      shared ADC input), S0-S3 -> 4 Nano pins used as address lines.
//      Only 8 of the 16 physical channels are populated with
//      sensors; MuxSelectChannel() + a short RC-settle delay + one
//      ADC conversion reads each in turn - same technique, and same
//      channel numbering, as the companion sensor-test sketch.
//    - 3 push buttons: CALIBRATE / START / STOP.
//
//  DESIGN NOTES / ASSUMPTIONS (read before tuning):
//
//  1. SENSOR ORIENTATION: index 0 = physical LEFT-most sensor,
//     index 7 = physical RIGHT-most sensor. Weights run -350..+350.
//     If the robot steers AWAY from the line instead of toward it,
//     your wiring is mirrored relative to this assumption - flip
//     the sign of `correction` in RunPIDControl() (one line, marked
//     below) rather than rewiring anything.
//
//  2. TRACK EVENTS are classified purely from the 8-bit sensor mask
//     and timing, with no dependency on knowing the maze layout:
//       - Normal line (1-4 sensors on, not all 8) -> weighted PID.
//       - ALL 8 ON (0xFF): a full-width bar under the array. This is
//         a straight/T/cross intersection OR a finish/stop bar. We
//         can't tell which from a single instant, so we time it:
//           * clears again quickly              -> intersection,
//             the robot simply drives straight through on stored
//             error/motor state and resumes normal PID once a
//             sensor sees line again.
//           * stays solid black past STOP_BAR_HOLD_MS, OR a second
//             all-black hit begins within DOUBLE_BAR_WINDOW_MS of
//             the first ending (a common "double stripe" finish
//             marker) -> declared STOP, robot decelerates and halts.
//       - ALL 8 OFF (0x00): line gap or the robot has driven past a
//         sharp turn / dead end so the line is no longer under any
//         sensor. We first COAST on the last known motor output for
//         GAP_COAST_MS (bridges small physical gaps/dashed line
//         without any steering surprise), then, if still lost, PIVOT
//         SEARCH in place toward whichever side the line was last
//         seen on (this is what recovers 90-degree corners, T-turns
//         taken as a turn, and dead ends). If the line has never
//         been tracked yet this run (e.g. started on white, slightly
//         off the line), searching continues indefinitely - it's
//         only treated as a genuine derailment, and stopped via
//         SEARCH_TIMEOUT_MS, once the line has actually been
//         followed successfully at least once this run.
//     45-degree / gentle corners need no special case: they show up
//     as ordinary partial bitmasks with growing |error|, handled by
//     the same proportional term plus the cornering speed curve.
//     If your rules define a T-junction that must be TURNED at
//     (maze solving) rather than driven straight through, that needs
//     external knowledge (which branch to take) - hook it in where
//     marked "MAZE HOOK" below.
//
//  3. FIXED-POINT EVERYWHERE ON THE HOT PATH: no floats in
//     DetectLineFast()/RunPIDControl(). AVR328P has no FPU, so this
//     is the single biggest speed win available at the algorithm
//     level. PID gains are Q8 fixed point (real_gain * 256).
//
//  4. HYSTERESIS (Schmitt trigger) per sensor kills flicker for
//     readings that sit right on the calibrated threshold, which
//     would otherwise inject noise straight into the error term.
//
//  5. DYNAMIC CRUISE SPEED: gradual ramp toward CRUISE_MAX_SPEED on
//     straights (so you're not permanently capped at BASE_SPEED),
//     immediate drop toward MIN_SPEED as |error| grows (safety
//     response to corners is not something to ease into).
// =============================================================

#include <Arduino.h>

// ================= PIN DEFINITIONS (motor driver) ==========
#define PWMA 3      // PD3 = OC2B (Timer2)  - RIGHT motor PWM
#define AI1  7      // PD7                   - RIGHT motor dir
#define AI2  6      // PD6                   - RIGHT motor dir
#define STBY 8      // PB0                   - driver standby/enable
#define PWMB 10     // PB2 = OC1B (Timer1)  - LEFT motor PWM
#define BI1  11     // PB3                   - LEFT motor dir
#define BI2  12     // PB4                   - LEFT motor dir

#define AI1_MASK (1 << PD7)
#define AI2_MASK (1 << PD6)
#define STBY_MASK (1 << PB0)
#define BI1_MASK (1 << PB3)
#define BI2_MASK (1 << PB4)

// ================= PIN DEFINITIONS (buttons) ================
#define BTN_START     5
#define BTN_STOP      2
#define BTN_CALIBRATE 9

// ================= PIN DEFINITIONS (16-ch mux) ================
// SIG -> A0 (PC0/ADC0, fixed - the mux does the channel switching,
// the ADC never has to). S0-S3 -> 4 address pins on PORTC. This
// matches your sensor-test sketch exactly: same pins, same channel
// numbering (S0 = bit0 ... S3 = bit3 of the channel number 0-15).
// Whatever channel index reads a clean sensor swing in that test
// sketch's Serial output is the same number you put in
// sensorMuxChannel[] below - no hex decoding needed.
#define MUX_SIG A0
#define MUX_S0  A4   // PC4 - address bit 0
#define MUX_S1  A3   // PC3 - address bit 1
#define MUX_S2  A2   // PC2 - address bit 2
#define MUX_S3  A1   // PC1 - address bit 3
#define MUX_SELECT_BITS 0x1E   // PC1|PC2|PC3|PC4 - the 4 mux address bits on PORTC
#define MUX_SETTLE_US 10       // RC settle time after switching address - do not remove

// Same channel encoding as SelectMuxChannel() in the test sketch
// (ch bit0->S0, bit1->S1, bit2->S2, bit3->S3), just built as one
// PORTC write instead of four digitalWrite() calls - digitalWrite
// is far too slow (~a few us + pin lookup) to call 4x per sensor,
// 8x per scan, on the PID hot path.
static inline void MuxSelectChannel(uint8_t ch) __attribute__((always_inline));
static inline void MuxSelectChannel(uint8_t ch) {
  uint8_t bits = 0;
  if (ch & 0x01) bits |= (1 << PC4);   // S0
  if (ch & 0x02) bits |= (1 << PC3);   // S1
  if (ch & 0x04) bits |= (1 << PC2);   // S2
  if (ch & 0x08) bits |= (1 << PC1);   // S3
  PORTC = (PORTC & ~MUX_SELECT_BITS) | bits;
}

// ================= SENSOR CONFIG =============================
#define NUM_SENSORS 8
#define MAX_SPEED 255            // hard PWM ceiling (8-bit timer)

// Physical mux channel number (0-15) for each of the 8 populated
// sensors, in left-to-right array order. Index 0 is the physical
// LEFT-most sensor - if your loom runs the other way, reverse this
// table rather than touching any other code.
//
// Confirmed by you via the sensor-test sketch: sensors are wired
// with a gap of one channel each (0,2,4,6,8,10,12,14), not to
// consecutive channels.
const uint8_t sensorMuxChannel[NUM_SENSORS] = {0, 2, 4, 6, 8, 10, 12, 14};

// Weighted position contribution per sensor, centered on 0.
// Evenly spaced +-350 exactly like the 8-sensor reference scheme.
const int16_t sensorWeight[NUM_SENSORS] = {-350, -250, -150, -50,
                                             50,  150,  250, 350};

// Calibration arrays, pre-seeded with YOUR measured readings so the
// robot has usable thresholds even if CALIBRATE is skipped. Pressing
// CALIBRATE still overwrites these with a fresh live sweep and
// SHOULD be done before any real run - ambient light, battery
// voltage, and track surface all drift these numbers over time; the
// values below are only a same-session snapshot, not a substitute.
//
// Measured white-surface averages (9 samples, all sensors on white):
//   CH0:282  CH2:77  CH4:72  CH6:158  CH8:158  CH10:158  CH12:159  CH14:269
// Measured black-surface averages (8 samples, all sensors on black):
//   CH0:928  CH2:897  CH4:865  CH6:861  CH8:878  CH10:876  CH12:881  CH14:917
// Note: CH0/CH14 (the two edge sensors) show noticeably less
// black/white contrast (~640-650 swing) and more sample-to-sample
// noise (CH0 white jittered 244-354) than the six inner sensors
// (~790 swing, jitter under 25 counts). That's still a wide enough
// margin around each threshold below to be harmless, but if you see
// flicker specifically from the outer two sensors on the real track,
// this is why - check their mounting height/angle first.
uint16_t sensorMin[NUM_SENSORS] = {282, 77, 72, 158, 158, 158, 159, 269};   // white
uint16_t sensorMax[NUM_SENSORS] = {928, 897, 865, 861, 878, 876, 881, 917}; // black
uint16_t threshold[NUM_SENSORS];   // computed by RecomputeThresholds()

// Derives threshold[] as the midpoint of sensorMin[]/sensorMax[] for
// every sensor. Shared by setup() (so the pre-seeded defaults above
// are usable immediately) and by Calibrate() (after a fresh sweep),
// so there's exactly one place this math lives.
void RecomputeThresholds() {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    threshold[i] = (sensorMin[i] + sensorMax[i]) >> 1;
  }
}

// Schmitt-trigger memory: bit i = 1 if sensor i was "on line" last scan.
uint8_t sensorOnState = 0;
#define HYSTERESIS_COUNTS 15   // ADC counts of dead-band around threshold

#define POS_ALL_BLACK  9999
#define POS_ALL_WHITE -9999

// ================= PID (Q8 FIXED POINT) ======================
// Start conservative, then push KP up until you see a *small*
// oscillation on straights, then back off ~10-15%. KD damps that
// oscillation and helps at corner entry/exit. KI stays 0 unless you
// observe steady-state drift on long straights (rare on a line
// follower - the line itself is the feedback, so KI mostly just
// adds lag risk).
const int16_t KP_Q8 = 180;   // ~0.70 real gain
const int16_t KI_Q8 = 0;
const int16_t KD_Q8 = 90;    // ~0.35 real gain
int32_t integral = 0;
#define INTEGRAL_CLAMP 4000
int16_t lastError = 0;

// ================= DYNAMIC SPEED CONTROL ======================
#define BASE_SPEED        150   // default / corner-recovery ceiling
#define MIN_SPEED         110   // slowest, at max error (sharpest correction)
#define CRUISE_MAX_SPEED  230   // top speed after sustained straight running
#define ERROR_FULL_SCALE  350   // matches max |weight|
#define STRAIGHT_ERROR_THRESHOLD 35   // |error| at/below this = "straight"
#define RAMP_INTERVAL_MS  15
#define RAMP_STEP         2

int16_t cruiseSpeed = BASE_SPEED;
uint32_t lastRampTime = 0;

// Corner target speed: linear BASE_SPEED -> MIN_SPEED as |error|
// grows to ERROR_FULL_SCALE. Applied immediately, never ramped -
// slowing for a sharp turn is a safety response, not a preference.
static inline int16_t ComputeCornerSpeed(int16_t error) __attribute__((always_inline));
static inline int16_t ComputeCornerSpeed(int16_t error) {
  int16_t absErr = (error < 0) ? (int16_t)(-error) : error;
  if (absErr > ERROR_FULL_SCALE) absErr = ERROR_FULL_SCALE;
  int32_t speed = BASE_SPEED -
      (((int32_t)(BASE_SPEED - MIN_SPEED) * absErr) / ERROR_FULL_SCALE);
  return (int16_t)speed;
}

// ================= JUNCTION / STOP-BAR HANDLING ===============
#define STOP_BAR_HOLD_MS      350   // continuous all-black this long = finish bar
#define DOUBLE_BAR_WINDOW_MS  400   // 2nd all-black within this of the 1st ending = finish gate
#define CROSS_MAX_MS          800   // safety: give up "waiting it out" after this long regardless

bool allBlackActive = false;
uint32_t allBlackStartTime = 0;
uint32_t lastAllBlackEndTime = 0;
bool sawFirstAllBlackBar = false;

// ================= LINE-LOST / SEARCH HANDLING ================
#define GAP_COAST_MS       80    // bridge small gaps on stored motor output
#define SEARCH_SPEED       120   // in-place pivot speed while hunting for the line
#define SEARCH_TIMEOUT_MS  1200  // give up and stop if not reacquired by then

bool lineLostActive = false;
uint32_t lineLostStartTime = 0;
int8_t lastDirection = 0;   // -1 = line last seen to the left, +1 = right, 0 = unknown
int16_t lastRightSpeed = 0, lastLeftSpeed = 0;

// True once the line has been tracked normally at least once this
// run. SEARCH_TIMEOUT_MS's give-up-and-stop behavior only applies
// after this is true - a cold start on a white surface (robot
// placed slightly off the line, or waiting at a start box) is not a
// derailment and should keep searching indefinitely instead of
// disabling the driver.
bool hasTrackedLine = false;

// ================= STATE MACHINE ==============================
enum RobotState { STATE_IDLE, STATE_CALIBRATING, STATE_RUNNING, STATE_STOPPED };
RobotState currentState = STATE_IDLE;

// ================= FUSED MUX-SELECT+READ+THRESHOLD+SUM ==========
// One step per sensor: set the mux address (single PORTC write,
// mask precomputed per channel), wait for the mux's RC settle,
// convert on the fixed ADC0/SIG input, Schmitt-trigger compare,
// accumulate into the weighted sum. This bypasses analogRead()'s
// per-call overhead (pin validation lookup) since the ADC is always
// reading ADC0 - only the mux address changes.
#define SENSOR_STEP(i)                                                       \
  do {                                                                       \
    MuxSelectChannel(sensorMuxChannel[i]);                                    \
    delayMicroseconds(MUX_SETTLE_US);                                        \
    ADCSRA |= (1 << ADSC);                                                   \
    while (ADCSRA & (1 << ADSC));                                            \
    int16_t val = (int16_t)ADC;                                              \
    int16_t thr = (int16_t)threshold[i];                                     \
    bool wasOn = (sensorOnState & (1 << i)) != 0;                            \
    int16_t edge = wasOn ? (int16_t)(thr - HYSTERESIS_COUNTS)                \
                          : (int16_t)(thr + HYSTERESIS_COUNTS);              \
    bool onLine = val > edge;   /* dark line => higher IR return => higher ADC */ \
    if (onLine) {                                                            \
      onLineMask |= (1 << i);                                                \
      int16_t d = val - thr;                                                 \
      if (d < 1) d = 1;                                                      \
      weightedSum += (int32_t)sensorWeight[i] * d;                           \
      sumValues += d;                                                        \
    }                                                                        \
  } while (0)

int16_t DetectLineFast(uint8_t *outMask) {
  uint8_t onLineMask = 0;
  int32_t weightedSum = 0;
  int16_t sumValues = 0;

  SENSOR_STEP(0);
  SENSOR_STEP(1);
  SENSOR_STEP(2);
  SENSOR_STEP(3);
  SENSOR_STEP(4);
  SENSOR_STEP(5);
  SENSOR_STEP(6);
  SENSOR_STEP(7);

  sensorOnState = onLineMask;
  *outMask = onLineMask;

  if (onLineMask == 0xFF) return POS_ALL_BLACK;
  if (onLineMask == 0x00) return POS_ALL_WHITE;

  // sumValues >= 1 guaranteed whenever onLineMask != 0.
  return (int16_t)(weightedSum / sumValues);
}

// ================= MOTOR CONTROL (direct port + OCR writes) ====
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

  lastRightSpeed = rightSpeed;
  lastLeftSpeed  = leftSpeed;
}

static inline void StopMotors() {
  OCR2B = 0;
  OCR1B = 0;
  PORTD &= ~(AI1_MASK | AI2_MASK);
  PORTB &= ~(BI1_MASK | BI2_MASK);
  lastRightSpeed = 0;
  lastLeftSpeed = 0;
}

static inline void EnableDriver()  { PORTB |= STBY_MASK; }
static inline void DisableDriver() { PORTB &= ~STBY_MASK; }

// ================= CALIBRATION ================================
// Rotate in place over the line so every sensor sweeps across both
// line and background, capturing true min/max per channel.
void Calibrate() {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    sensorMin[i] = 1023;
    sensorMax[i] = 0;
  }

  EnableDriver();
  SetMotors(90, -90);

  for (uint16_t sample = 0; sample < 1800; sample++) {
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
      MuxSelectChannel(sensorMuxChannel[i]);
      delayMicroseconds(MUX_SETTLE_US);
      ADCSRA |= (1 << ADSC);
      while (ADCSRA & (1 << ADSC));
      uint16_t v = ADC;
      if (v < sensorMin[i]) sensorMin[i] = v;
      if (v > sensorMax[i]) sensorMax[i] = v;
    }
    delay(3);
  }

  StopMotors();
  DisableDriver();

  RecomputeThresholds();

  sensorOnState = 0;  // stale hysteresis memory is meaningless post-recalibration
}

// ================= EVENT RESET (call when (re)entering RUNNING) ==
void ResetRunState() {
  integral = 0;
  lastError = 0;
  cruiseSpeed = BASE_SPEED;
  lastRampTime = millis();
  allBlackActive = false;
  sawFirstAllBlackBar = false;
  lastAllBlackEndTime = 0;
  lineLostActive = false;
  lastDirection = 0;
  hasTrackedLine = false;
}

// ================= MAIN CONTROL LOOP ===========================
void RunPIDControl() {
  uint8_t mask = 0;
  int16_t position = DetectLineFast(&mask);
  uint32_t now = millis();

  // ---------- Case 1: full-width black bar (junction or stop) ----------
  if (position == POS_ALL_BLACK) {
    lineLostActive = false; // definitely not a gap right now

    if (!allBlackActive) {
      allBlackActive = true;
      allBlackStartTime = now;

      // Double-bar finish check: did a previous bar end recently?
      if (sawFirstAllBlackBar &&
          (now - lastAllBlackEndTime) < DOUBLE_BAR_WINDOW_MS) {
        StopMotors();
        DisableDriver();
        currentState = STATE_STOPPED;
        return;
      }
    }

    // Held solid long enough on its own -> treat as the finish bar too.
    if ((now - allBlackStartTime) >= STOP_BAR_HOLD_MS) {
      StopMotors();
      DisableDriver();
      currentState = STATE_STOPPED;
      return;
    }

    // MAZE HOOK: if your rules require turning at this intersection
    // instead of driving straight through, decide the turn here using
    // externally-known layout info and call a turn routine instead of
    // falling through. Default behavior below = drive straight through
    // holding the last commanded motor output (keeps geometry-agnostic).
    SetMotors(lastRightSpeed, lastLeftSpeed);
    return;
  }

  // We are not currently under a black bar - if we just left one,
  // remember when, for the double-bar finish check above.
  if (allBlackActive) {
    allBlackActive = false;
    sawFirstAllBlackBar = true;
    lastAllBlackEndTime = now;
  }

  // ---------- Case 2: line lost entirely (gap, sharp turn, dead end) ----------
  if (position == POS_ALL_WHITE) {
    if (!lineLostActive) {
      lineLostActive = true;
      lineLostStartTime = now;
    }

    uint32_t lostFor = now - lineLostStartTime;

    if (lostFor < GAP_COAST_MS) {
      // Bridge small physical gaps (dashed line) with zero steering
      // surprise: just keep the last commanded output briefly.
      SetMotors(lastRightSpeed, lastLeftSpeed);
      return;
    }

    if (lostFor < SEARCH_TIMEOUT_MS || !hasTrackedLine) {
      // Pivot search toward whichever side the line was last seen on.
      // This is what recovers 90-degree turns and dead ends: the line
      // vanished from under the array because it turned away to one
      // side, so rotating that way reacquires it.
      //
      // The `|| !hasTrackedLine` keeps this branch active indefinitely
      // if the line has never been found yet this run (e.g. started
      // on a white surface, slightly off the line, or in a start box)
      // - that is not a derailment, so it must not fall through to the
      // give-up stop below no matter how long it takes.
      if (lastDirection > 0) {
        SetMotors(SEARCH_SPEED, -SEARCH_SPEED);   // pivot right
      } else if (lastDirection < 0) {
        SetMotors(-SEARCH_SPEED, SEARCH_SPEED);   // pivot left
      } else {
        SetMotors(SEARCH_SPEED, SEARCH_SPEED);    // no history: creep forward
      }
      return;
    }

    // Truly lost past the timeout, AND the line had been tracked
    // successfully at some point this run - this is a genuine
    // mid-run derailment (drove off the track entirely), not a cold
    // start. Fail-safe stop rather than wandering off blind.
    StopMotors();
    DisableDriver();
    currentState = STATE_STOPPED;
    return;
  }

  // ---------- Case 3: normal line tracking (1-7 sensors, weighted PID) ----------
  lineLostActive = false;
  hasTrackedLine = true;
  lastDirection = (position > 0) ? 1 : (position < 0 ? -1 : lastDirection);

  int16_t error = position;
  integral += error;
  if (integral > INTEGRAL_CLAMP) integral = INTEGRAL_CLAMP;
  if (integral < -INTEGRAL_CLAMP) integral = -INTEGRAL_CLAMP;
  int16_t derivative = error - lastError;

  int32_t correction =
      ((int32_t)KP_Q8 * error + (int32_t)KI_Q8 * integral +
       (int32_t)KD_Q8 * derivative) >> 8;

  // Speed: ramp toward CRUISE_MAX_SPEED while straight, snap down
  // toward MIN_SPEED immediately once error grows (corner / 45-deg
  // bend - no special-case code needed, this curve covers it).
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

  // NOTE: if the robot steers AWAY from the line, swap the +/- signs
  // on the next two lines (this is the single line referenced in the
  // orientation note at the top of the file).
  int16_t rightSpeed = cruiseSpeed + (int16_t)correction;
  int16_t leftSpeed  = cruiseSpeed - (int16_t)correction;

  SetMotors(rightSpeed, leftSpeed);
  lastError = error;
}

// ================= BUTTON HANDLING (debounced edge detection) ===
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

// ================= SETUP =======================================
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

  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_S3, OUTPUT);
  // MUX_SIG (A0) is left as a plain ADC input; the mux's EN pin is
  // assumed hardwired to GND (always enabled).

  // Enable OC2B/OC1B PWM outputs on pins 3 and 10. Arduino core has
  // already put Timer1/Timer2 into a PWM mode at startup; we just
  // need the compare outputs connected so OCR2B/OCR1B drive them.
  TCCR2A |= (1 << COM2B1);
  TCCR1A |= (1 << COM1B1);

  ADMUX = (1 << REFS0) | 0;             // fixed on ADC0 (A0/SIG) forever - the mux switches channels, not us
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1); // ADC on, prescaler 64 (16MHz/64=250kHz, safe+fast)

  DisableDriver();
  StopMotors();
  RecomputeThresholds();   // usable immediately from the pre-seeded defaults; CALIBRATE overwrites with a live sweep
  currentState = STATE_IDLE;
}

// ================= MAIN LOOP ====================================
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
    ResetRunState();
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
