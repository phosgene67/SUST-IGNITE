#include <Arduino.h>
#define PWMA 3      // PD3 = OC2B (Timer2)  - RIGHT motor PWM
#define AI1  7      // PD7                   - RIGHT motor dir
#define AI2  6      // PD6                   - RIGHT motor dir
#define STBY 8      // PB0                   - driver standby/enable
#define PWMB 10     // PB2 = OC1B (Timer1)  - LEFT motor PWM
#define BI1  11     // PB3                   - LEFT motor dir
#define BI2  12     // PB4                   - LEFT motor dir

#define BTN_START     5
#define BTN_STOP      2
#define BTN_CALIBRATE 9

#define NUM_SENSORS 6

const uint8_t sensorPins[NUM_SENSORS] = { A5, A4, A3, A2, A1, A0 };
const int sensorWeight[NUM_SENSORS] = { -250, -150, -50,
                                           50,  150, 250 };

#define POS_ALL_WHITE (-9999)
#define POS_ALL_BLACK  9999
#define BASE_SPEED               180
#define MIN_SPEED                110
#define CRUISE_MAX_SPEED         150
#define SEARCH_SPEED             120
#define ERROR_FULL_SCALE         250
#define STRAIGHT_ERROR_THRESHOLD 25
#define RAMP_INTERVAL_MS         15UL
#define RAMP_STEP                2
#define STOP_BOX_HOLD_MS 100UL   // full-black held this long = filled finish box, not a thin junction bar
#define GAP_COAST_MS      100UL     // bridges small dashed-line gaps
#define SEARCH_TIMEOUT_MS 2000UL   // give up & stop past this, if line was ever tracked
#define CROSS_HOLD_MS     10UL    // full-black held this long = probable T, not a quick crossing

#define TURN_45_MIN_ERROR 100
#define TURN_90_MIN_ERROR 200

#define LEFT_EXTREME_MASK  0b000001   // sensor 0 only
#define RIGHT_EXTREME_MASK 0b100000   // sensor 5 only
#define LEFT_HALF_MASK     0b000111   // sensors 0-2
#define RIGHT_HALF_MASK    0b111000   // sensors 3-5
#define ALL_SENSORS_MASK   0b111111   // all 6 sensors on-line

#define CALIBRATION_DURATION_MS 5000UL
#define DEBOUNCE_MS 25UL

class MotorDriver {
public:
    void begin();

    void drive(int leftSpeed, int rightSpeed);
    void forward(int speed);
    void backward(int speed);
    void turnLeft(int speed);
    void turnRight(int speed);
    void brake();
    void stop();

    void setLeftMotor(int speed);
    void setRightMotor(int speed);

    void testForward();
    void testBackward();
    void testLeft();
    void testRight();

    int getLastLeftSpeed()  const { return _lastLeftSpeed; }
    int getLastRightSpeed() const { return _lastRightSpeed; }

private:
    static const int MAX_SPEED  = 255;
    static const int TEST_SPEED = 150;
    static const unsigned long TEST_DURATION_MS = 1000;

    int _lastLeftSpeed  = 0;
    int _lastRightSpeed = 0;

    int _clampSpeed(int speed) const;
};


class SensorArray {
public:
    void begin();

    void readAnalog();     // populates raw sensorValues[] - call once per loop
    void readDigital();    // thresholds the values already in sensorValues[]

    void calibrate();      // call repeatedly WHILE the robot sweeps over the line
    void autoThreshold();  // call once after calibrate() loop completes

    int getPosition();
    int getError();
    bool isCentered();
    bool lineDetected();
    uint8_t getMask() const { return onLineMask; }

    void printAnalog();
    void printDigital();
    void printThreshold();
    void printPosition();

private:
    int sensorValues[NUM_SENSORS];
    uint16_t sensorMin[NUM_SENSORS];
    uint16_t sensorMax[NUM_SENSORS];
    uint16_t threshold[NUM_SENSORS];
    uint8_t onLineMask = 0;
    uint8_t sensorOnState = 0;   // Schmitt-trigger memory per sensor

    static const int HYSTERESIS_COUNTS = 15;
    static const int CENTERED_ERROR_THRESHOLD = 20;
};


class PIDController {
public:
    void begin();
    int compute(int error);
    void reset();

    // Gains are Q8 fixed-point: pass (real_gain * 256) as an integer.
    // e.g. a real gain of 0.70 -> kpQ8 = 179. AVR328P has no FPU, so
    // fixed-point integer math is the single biggest speed win
    // available on this hot path - no floats anywhere in here.
    void setTunings(int16_t kpQ8, int16_t kiQ8, int16_t kdQ8);
    void setOutputLimits(int minimum, int maximum);

private:
    int16_t _kpQ8 = 167;   // ~0.70 real gain
    int16_t _kiQ8 = 0;
    int16_t _kdQ8 = 77;    // ~0.35 real gain

    int32_t _integral = 0;
    int _lastError = 0;

    int _outMin = -255;
    int _outMax = 255;

    static const int32_t INTEGRAL_CLAMP = 4000;
};


class JunctionDetector {
public:
    void begin();
    void update();   // call once per loop, right after sensor.readDigital()

    bool detect45Left();
    bool detect45Right();
    bool detect90Left();
    bool detect90Right();
    bool detectTJunction();
    bool detectCross();
    bool detectGap();
    bool detectWhiteLine();
    bool detectLostLine();
    bool detectStopBox();
    unsigned long whiteDuration() const;  // ms since all-white began, 0 if not currently all-white
    unsigned long blackDuration() const;  // ms since all-black began, 0 if not currently all-black

private:
    uint8_t currentMask = 0;
    uint8_t previousMask = 0;

    bool allBlackActive = false;
    unsigned long allBlackStartTime = 0;

    bool allWhiteActive = false;
    unsigned long allWhiteStartTime = 0;
};


enum RobotState {
    STATE_IDLE,
    STATE_CALIBRATING,
    STATE_RUNNING,
    STATE_STOPPED
};

RobotState currentState = STATE_IDLE;


class Robot {
public:
    void begin();
    void update();

private:
    void followLine();

    void updateState();
    void executeState();

    void handle45Left();
    void handle45Right();
    void handle90Left();
    void handle90Right();
    void handleTJunction();
    void handleCross();
    void handleStopBox();
    void handleGap();
    void handleWhiteLine();
    void searchLine();

    bool readButtonPressed(uint8_t pin, uint8_t index);

    uint8_t _lastBtnState[3] = { HIGH, HIGH, HIGH };
    unsigned long _lastBtnChangeTime[3] = { 0, 0, 0 };

    int _cruiseSpeed = BASE_SPEED;
    unsigned long _lastRampTime = 0;

    bool _hasTrackedLine = false;
    int8_t _lastDirection = 0;   // -1 = line last seen left, +1 = right, 0 = unknown

    unsigned long _calibrationStartTime = 0;
};

// Global instances - declared here so every ::method() body below can
// reference them directly.
MotorDriver motor;
SensorArray sensor;
PIDController pid;
JunctionDetector junction;
Robot robot;

int MotorDriver::_clampSpeed(int speed) const {
    if (speed > MAX_SPEED) return MAX_SPEED;
    if (speed < -MAX_SPEED) return -MAX_SPEED;
    return speed;
}

void MotorDriver::begin() {
    pinMode(PWMA, OUTPUT);
    pinMode(AI1, OUTPUT);
    pinMode(AI2, OUTPUT);
    pinMode(STBY, OUTPUT);
    pinMode(PWMB, OUTPUT);
    pinMode(BI1, OUTPUT);
    pinMode(BI2, OUTPUT);

    digitalWrite(STBY, LOW);   // stay in standby until a real command wakes it
    setLeftMotor(0);
    setRightMotor(0);
}

void MotorDriver::setRightMotor(int speed) {
    speed = _clampSpeed(speed);
    _lastRightSpeed = speed;
    digitalWrite(STBY, HIGH);

    // Direction pins swapped vs. the original mapping - this motor was
    // spinning backward on a positive (forward) command.
    int magnitude = speed;
    if (magnitude >= 0) {
        digitalWrite(AI1, LOW);
        digitalWrite(AI2, HIGH);
    } else {
        digitalWrite(AI1, HIGH);
        digitalWrite(AI2, LOW);
        magnitude = -magnitude;
    }
    analogWrite(PWMA, magnitude);
}

void MotorDriver::setLeftMotor(int speed) {
    speed = _clampSpeed(speed);
    _lastLeftSpeed = speed;
    digitalWrite(STBY, HIGH);

    // Direction pins swapped vs. the original mapping - this motor was
    // spinning backward on a positive (forward) command.
    int magnitude = speed;
    if (magnitude >= 0) {
        digitalWrite(BI1, LOW);
        digitalWrite(BI2, HIGH);
    } else {
        digitalWrite(BI1, HIGH);
        digitalWrite(BI2, LOW);
        magnitude = -magnitude;
    }
    analogWrite(PWMB, magnitude);
}

void MotorDriver::drive(int leftSpeed, int rightSpeed) {
    setLeftMotor(leftSpeed);
    setRightMotor(rightSpeed);
}

void MotorDriver::forward(int speed) {
    speed = _clampSpeed(speed);
    if (speed < 0) speed = -speed;
    drive(speed, speed);
}

void MotorDriver::backward(int speed) {
    speed = _clampSpeed(speed);
    if (speed < 0) speed = -speed;
    drive(-speed, -speed);
}

void MotorDriver::turnLeft(int speed) {
    speed = _clampSpeed(speed);
    if (speed < 0) speed = -speed;
    drive(-speed, speed);   // pivot: left backward, right forward
}

void MotorDriver::turnRight(int speed) {
    speed = _clampSpeed(speed);
    if (speed < 0) speed = -speed;
    drive(speed, -speed);   // pivot: left forward, right backward
}

void MotorDriver::brake() {
    digitalWrite(STBY, HIGH);
    // TB6612 short-brake: both direction pins HIGH shorts the windings -
    // stops much faster than just zeroing PWM.
    digitalWrite(AI1, HIGH);
    digitalWrite(AI2, HIGH);
    analogWrite(PWMA, MAX_SPEED);

    digitalWrite(BI1, HIGH);
    digitalWrite(BI2, HIGH);
    analogWrite(PWMB, MAX_SPEED);

    _lastLeftSpeed = 0;
    _lastRightSpeed = 0;
}

void MotorDriver::stop() {
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);
    digitalWrite(AI1, LOW);
    digitalWrite(AI2, LOW);
    digitalWrite(BI1, LOW);
    digitalWrite(BI2, LOW);
    digitalWrite(STBY, LOW);
    _lastLeftSpeed = 0;
    _lastRightSpeed = 0;
}

void MotorDriver::testForward()  { forward(TEST_SPEED);  delay(TEST_DURATION_MS); stop(); }
void MotorDriver::testBackward() { backward(TEST_SPEED); delay(TEST_DURATION_MS); stop(); }
void MotorDriver::testLeft()     { turnLeft(TEST_SPEED); delay(TEST_DURATION_MS); stop(); }
void MotorDriver::testRight()    { turnRight(TEST_SPEED);delay(TEST_DURATION_MS); stop(); }

// =============================================================
//  SENSORARRAY IMPLEMENTATION
// =============================================================
void SensorArray::begin() {
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        pinMode(sensorPins[i], INPUT);
        sensorMin[i] = 1023;   // pre-seeded so calibrate() starts from the full range
        sensorMax[i] = 0;
        threshold[i] = 512;    // usable midpoint default before first real calibration
        sensorValues[i] = 0;
    }
    onLineMask = 0;
    sensorOnState = 0;

    // Optional IR emitter control, if your QTR-6A board has one wired
    // to a digital pin instead of always-on:
    // pinMode(4, OUTPUT);
    // digitalWrite(4, HIGH);
}

void SensorArray::readAnalog() {
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        sensorValues[i] = analogRead(sensorPins[i]);
    }
}

void SensorArray::readDigital() {
    // Uses whatever readAnalog() last populated - call readAnalog()
    // first each loop, this just thresholds the cached raw values.
    uint8_t mask = 0;
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        bool wasOn = (sensorOnState & (1 << i)) != 0;
        // Schmitt-trigger hysteresis kills flicker for readings sitting
        // right on the threshold.
        int edge = wasOn ? (int)threshold[i] - HYSTERESIS_COUNTS
                          : (int)threshold[i] + HYSTERESIS_COUNTS;
        bool onLine = sensorValues[i] > edge;  // darker surface -> higher IR return -> higher ADC
        if (onLine) mask |= (1 << i);
    }
    sensorOnState = mask;
    onLineMask = mask;
}

void SensorArray::calibrate() {
    // Call this repeatedly (once per loop, ~1-2 seconds' worth) WHILE
    // Robot spins the chassis over the line so every sensor sweeps
    // both black and white. This method only samples and folds into
    // the running min/max - it doesn't move anything itself, since
    // motion policy belongs to Robot, not the sensor module.
    readAnalog();
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        if (sensorValues[i] < sensorMin[i]) sensorMin[i] = sensorValues[i];
        if (sensorValues[i] > sensorMax[i]) sensorMax[i] = sensorValues[i];
    }
}

void SensorArray::autoThreshold() {
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        threshold[i] = (sensorMin[i] + sensorMax[i]) / 2;
    }
    sensorOnState = 0;   // stale hysteresis memory is meaningless post-recalibration
}

int SensorArray::getPosition() {
    // Assumes readAnalog() + readDigital() already ran this cycle.
    if (onLineMask == ALL_SENSORS_MASK) return POS_ALL_BLACK;
    if (onLineMask == 0x00) return POS_ALL_WHITE;

    long weightedSum = 0;
    long sumValues = 0;
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        if (onLineMask & (1 << i)) {
            int d = sensorValues[i] - (int)threshold[i];
            if (d < 1) d = 1;
            weightedSum += (long)sensorWeight[i] * d;
            sumValues += d;
        }
    }
    return (int)(weightedSum / sumValues);   // sumValues >= 1 guaranteed when onLineMask != 0
}

int SensorArray::getError() {
    // Target is the centerline (0), so error IS position.
    return getPosition();
}

bool SensorArray::isCentered() {
    int e = getError();
    if (e == POS_ALL_WHITE || e == POS_ALL_BLACK) return false;
    int absE = (e < 0) ? -e : e;
    return absE <= CENTERED_ERROR_THRESHOLD;
}

bool SensorArray::lineDetected() {
    return onLineMask != 0x00;
}

void SensorArray::printAnalog() {
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        Serial.print(sensorValues[i]);
        Serial.print('\t');
    }
    Serial.println();
}

void SensorArray::printDigital() {
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        Serial.print((onLineMask & (1 << i)) ? '1' : '0');
    }
    Serial.println();
}

void SensorArray::printThreshold() {
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        Serial.print(threshold[i]);
        Serial.print('\t');
    }
    Serial.println();
}

void SensorArray::printPosition() {
    Serial.println(getPosition());
}

// =============================================================
//  PIDCONTROLLER IMPLEMENTATION
// =============================================================
void PIDController::begin() {
    reset();
}

void PIDController::reset() {
    _integral = 0;
    _lastError = 0;
}

void PIDController::setTunings(int16_t kpQ8, int16_t kiQ8, int16_t kdQ8) {
    _kpQ8 = kpQ8;
    _kiQ8 = kiQ8;
    _kdQ8 = kdQ8;
}

void PIDController::setOutputLimits(int minimum, int maximum) {
    _outMin = minimum;
    _outMax = maximum;
}

int PIDController::compute(int error) {
    _integral += error;
    if (_integral > INTEGRAL_CLAMP) _integral = INTEGRAL_CLAMP;
    if (_integral < -INTEGRAL_CLAMP) _integral = -INTEGRAL_CLAMP;

    int derivative = error - _lastError;
    _lastError = error;

    // Q8 fixed point: gains were pre-scaled by 256, so shift back down
    // after the multiply-accumulate. Pure integer math, no floats.
    int32_t output = ((int32_t)_kpQ8 * error +
                       (int32_t)_kiQ8 * _integral +
                       (int32_t)_kdQ8 * derivative) >> 8;

    if (output > _outMax) output = _outMax;
    if (output < _outMin) output = _outMin;

    return (int)output;
}

// =============================================================
//  JUNCTIONDETECTOR IMPLEMENTATION
// =============================================================
void JunctionDetector::begin() {
    currentMask = 0;
    previousMask = 0;
    allBlackActive = false;
    allBlackStartTime = 0;
    allWhiteActive = false;
    allWhiteStartTime = 0;
}

void JunctionDetector::update() {
    previousMask = currentMask;
    currentMask = sensor.getMask();
    unsigned long now = millis();

    if (currentMask == ALL_SENSORS_MASK) {
        if (!allBlackActive) {
            allBlackActive = true;
            allBlackStartTime = now;
        }
    } else {
        allBlackActive = false;
    }

    if (currentMask == 0x00) {
        if (!allWhiteActive) {
            allWhiteActive = true;
            allWhiteStartTime = now;
        }
    } else {
        allWhiteActive = false;
    }
}

unsigned long JunctionDetector::whiteDuration() const {
    return allWhiteActive ? (millis() - allWhiteStartTime) : 0;
}

unsigned long JunctionDetector::blackDuration() const {
    return allBlackActive ? (millis() - allBlackStartTime) : 0;
}

bool JunctionDetector::detect45Left() {
    // Only meaningful for a partial mask - caller guarantees this by
    // checking cross/T/white cases first.
    int err = sensor.getError();
    return (err <= -TURN_45_MIN_ERROR && err > -TURN_90_MIN_ERROR) &&
           !(currentMask & RIGHT_HALF_MASK);
}

bool JunctionDetector::detect45Right() {
    int err = sensor.getError();
    return (err >= TURN_45_MIN_ERROR && err < TURN_90_MIN_ERROR) &&
           !(currentMask & LEFT_HALF_MASK);
}

bool JunctionDetector::detect90Left() {
    int err = sensor.getError();

    return (err <= -TURN_90_MIN_ERROR) &&
           (currentMask & LEFT_EXTREME_MASK) &&
           !(currentMask & RIGHT_HALF_MASK);
}

bool JunctionDetector::detect90Right() {
     int err = sensor.getError();

    return (err >= TURN_90_MIN_ERROR) &&
           (currentMask & RIGHT_EXTREME_MASK) &&
           !(currentMask & LEFT_HALF_MASK);
}

bool JunctionDetector::detectCross() {
    // Exiting a full-black bar back into a partial line pattern means
    // the line continues straight ahead - the bar was a simple
    // crossing, not a dead-ended branch. No timing needed: this is a
    // one-frame transition check.
    return previousMask == ALL_SENSORS_MASK &&
           currentMask != ALL_SENSORS_MASK &&
           currentMask != 0x00;
}

bool JunctionDetector::detectTJunction() {
   if (currentMask != ALL_SENSORS_MASK)
        return false;
   if (previousMask == 0x00 || previousMask == ALL_SENSORS_MASK) return false;

    bool wasOnTrunk = (previousMask & 0b001100) != 0;

    return wasOnTrunk;

}

bool JunctionDetector::detectGap() {
    // Short break in the line (dashed segment) - still inside the
    // bridging window, so the caller should coast, not react.
    return allWhiteActive && whiteDuration() < GAP_COAST_MS;
}

bool JunctionDetector::detectWhiteLine() {
    // Instantaneous "nothing detected this scan". detectGap()/
    // detectLostLine() add timing judgment on top of this raw signal.
    return currentMask == 0x00;
}

bool JunctionDetector::detectLostLine() {
    // Past the gap-bridging window - either a sharp turn the reactive
    // pivot logic hasn't caught, or a genuine derailment. Robot::
    // searchLine() resolves which by also checking hasTrackedLine and
    // SEARCH_TIMEOUT_MS.
    return allWhiteActive && whiteDuration() >= GAP_COAST_MS;
}
bool JunctionDetector::detectStopBox() {
    // A finish/stop box is a solid filled rectangle - wider than any
    // junction crossbar. Real T/cross junctions self-clear within a
    // loop or two of the pivot/drive-through starting (the array
    // rotates or drives off a thin bar). If full-black survives well
    // past that, it's not a junction anymore - it's a filled box.
    return allBlackActive && blackDuration() >= STOP_BOX_HOLD_MS;
}

// =============================================================
//  ROBOT IMPLEMENTATION
// =============================================================
void Robot::begin() {
    Serial.begin(9600);   // harmless if you never call the print*() debug methods

    motor.begin();
    sensor.begin();
    pid.begin();
    pid.setTunings(180, 0, 90);   // Q8: ~0.70, 0, ~0.35 real gains
    pid.setOutputLimits(-255, 255);
    junction.begin();

    pinMode(BTN_START, INPUT_PULLUP);
    pinMode(BTN_STOP, INPUT_PULLUP);
    pinMode(BTN_CALIBRATE, INPUT_PULLUP);

    currentState = STATE_IDLE;
}

bool Robot::readButtonPressed(uint8_t pin, uint8_t index) {
    uint8_t current = digitalRead(pin);
    unsigned long now = millis();
    bool pressed = false;
    if (current != _lastBtnState[index] && (now - _lastBtnChangeTime[index]) > DEBOUNCE_MS) {
        _lastBtnChangeTime[index] = now;
        _lastBtnState[index] = current;
        pressed = (current == LOW);   // INPUT_PULLUP: LOW = pressed
    }
    return pressed;
}   

void Robot::updateState() {
    bool calibratePressed = readButtonPressed(BTN_CALIBRATE, 0);
    bool startPressed = readButtonPressed(BTN_START, 1);
    bool stopPressed = readButtonPressed(BTN_STOP, 2);

    if (calibratePressed && currentState == STATE_IDLE) {
        currentState = STATE_CALIBRATING;
        _calibrationStartTime = millis();
        motor.turnLeft(90);   // rotate in place so every sensor sweeps the line
    }

    if (startPressed && (currentState == STATE_STOPPED || currentState == STATE_IDLE)) {
        currentState = STATE_RUNNING;
        pid.reset();
        _cruiseSpeed = BASE_SPEED;
        _lastRampTime = millis();
        _hasTrackedLine = false;
        _lastDirection = 0;
        motor.stop();
    }

    if (stopPressed && currentState == STATE_RUNNING) {
        currentState = STATE_STOPPED;
        motor.stop();
    }
}

void Robot::executeState() {
    switch (currentState) {
        case STATE_CALIBRATING: {
            sensor.calibrate();
            if (millis() - _calibrationStartTime >= CALIBRATION_DURATION_MS) {
                motor.stop();
                sensor.autoThreshold();
                currentState = STATE_STOPPED;
            }
            break;
        }
        case STATE_RUNNING:
            followLine();
            break;
        case STATE_IDLE:
        case STATE_STOPPED:
        default:
            break;   // motors already stopped by whatever transitioned us here
    }
}

void Robot::update() {
    updateState();
    executeState();
}

void Robot::followLine() {
    sensor.readAnalog();
    sensor.readDigital();
    junction.update();
    // ---------- full-width black held long = finish/stop box ----------
    if (junction.detectStopBox()) { handleStopBox(); return; }
    // ---------- full-width black bar: cross / T-junction ----------
   
    if (junction.detectTJunction()) { handleTJunction(); return; }
     if (junction.detectCross()) { handleCross(); return; }

    // ---------- line lost entirely: gap or genuine derailment ----------
    if (junction.detectGap()) { handleGap(); return; }
    if (junction.detectLostLine()) { searchLine(); return; }

    // ---------- sharp corners: commit to a pivot ----------
    if (junction.detect90Left())  { handle90Left();  return; }
    if (junction.detect90Right()) { handle90Right(); return; }

    // ---------- gentle diagonals: PID already steers this correctly,
    // these hooks just keep bookkeeping fresh a beat early ----------
    if (junction.detect45Left())  { handle45Left();  }
    if (junction.detect45Right()) { handle45Right(); }

    // ---------- normal line tracking ----------
    _hasTrackedLine = true;
    int error = sensor.getError();
    _lastDirection = (error > 0) ? 1 : (error < 0 ? -1 : _lastDirection);

    int correction = pid.compute(error);

    int absErr = (error < 0) ? -error : error;
    unsigned long now = millis();
    if (absErr <= STRAIGHT_ERROR_THRESHOLD) {
        if (now - _lastRampTime >= RAMP_INTERVAL_MS) {
            _lastRampTime = now;
            if (_cruiseSpeed < CRUISE_MAX_SPEED) _cruiseSpeed += RAMP_STEP;
        }
    } else {
        _lastRampTime = now;
        int absClamped = (absErr > ERROR_FULL_SCALE) ? ERROR_FULL_SCALE : absErr;
        _cruiseSpeed = BASE_SPEED - (((BASE_SPEED - MIN_SPEED) * absClamped) / ERROR_FULL_SCALE);
    }

    // NOTE (wiring/mirroring): if the robot steers AWAY from the line,
    // swap the two lines below rather than rewiring anything.
    int rightSpeed = _cruiseSpeed + correction;
    int leftSpeed  = _cruiseSpeed - correction;

    motor.drive(leftSpeed, rightSpeed);
}

void Robot::handle45Left() {
    _lastRampTime = millis();
    _lastDirection = -1;
}

void Robot::handle45Right() {
    _lastRampTime = millis();
    _lastDirection = 1;
}

void Robot::handle90Left() {
    motor.turnRight(SEARCH_SPEED);
    _lastDirection = 1;
    _hasTrackedLine = true;
    _cruiseSpeed = BASE_SPEED;   // re-enter the straight-line ramp fresh once this clears
    _lastRampTime = millis();
}

void Robot::handle90Right() {
    motor.turnLeft(SEARCH_SPEED);
    _lastDirection = -1;
    _hasTrackedLine = true;
    _cruiseSpeed = BASE_SPEED;
    _lastRampTime = millis();
}

void Robot::handleCross() {
motor.turnRight(SEARCH_SPEED);

    _lastDirection = -1;
    _hasTrackedLine = true;
    _cruiseSpeed = BASE_SPEED;   // re-enter the straight-line ramp fresh once this clears
    _lastRampTime = millis();
}

void Robot::handleTJunction() {
    motor.turnLeft(SEARCH_SPEED);

    _lastDirection = 1;
    _hasTrackedLine = true;
    _cruiseSpeed = BASE_SPEED;   // re-enter the straight-line ramp fresh once this clears
    _lastRampTime = millis();
}

void Robot::handleGap() {
    // Small dashed-line break - hold last output rather than react;
    // reacting to a single blank scan is what makes robots twitch on
    // dashed tracks.
    motor.drive(motor.getLastLeftSpeed(), motor.getLastRightSpeed());
}

void Robot::handleWhiteLine() {
    // Provided for API completeness / manual use (e.g. call this
    // yourself if you want a different reaction to "any blank scan"
    // than the gap/lost timing split above uses). Not wired into
    // followLine()'s default dispatch since detectGap()/detectLostLine()
    // already cover every case where detectWhiteLine() is true.
    motor.drive(motor.getLastLeftSpeed(), motor.getLastRightSpeed());
}

void Robot::searchLine() {
    unsigned long lostFor = junction.whiteDuration();

    if (_hasTrackedLine && lostFor >= SEARCH_TIMEOUT_MS) {
        // Line was tracked successfully earlier this run and has now
        // been missing well past a reasonable search window - genuine
        // derailment, not a cold start. Fail-safe stop.
        motor.stop();
        currentState = STATE_STOPPED;
        return;
    }

    // Pivot toward whichever side the line was last seen on. If it's
    // never been seen yet this run (cold start slightly off-line, or
    // sitting in a start box), the `_hasTrackedLine` gate above keeps
    // this searching indefinitely instead of giving up.
    if (_lastDirection > 0) {
        motor.turnRight(SEARCH_SPEED);
    } else if (_lastDirection < 0) {
        motor.turnLeft(SEARCH_SPEED);
    } else {
        motor.forward(SEARCH_SPEED);   // no history yet - creep forward
    }
}
void Robot::handleStopBox() {
    motor.brake();              // TB6612 short-brake - fast, decisive stop
    currentState = STATE_STOPPED;
}

// =============================================================
//  ARDUINO ENTRY POINTS
// =============================================================
void setup() {
    robot.begin();
}

void loop() {
    robot.update();
}
