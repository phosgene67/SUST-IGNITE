#include "Robot.h"
#include "MotorDriver.h"
#include "SensorArray.h"
#include "PIDController.h"
#include "JunctionDetector.h"
#include<Arduino.h>

Robot::Robot(MotorDriver& motor, SensorArray& sensor,
             PIDController& pid, JunctionDetector& junction)
    : _motor(motor), _sensor(sensor), _pid(pid), _junction(junction) {}

void Robot::begin() {
    Serial.begin(9600);   // harmless if you never call the print*() debug methods

    _motor.begin();
    _sensor.begin();
    _pid.begin();
    _pid.setTunings(180, 0, 90);   // Q8: ~0.70, 0, ~0.35 real gains
    _pid.setOutputLimits(-255, 255);
    _junction.begin();

    pinMode(BTN_START, INPUT_PULLUP);
    pinMode(BTN_STOP, INPUT_PULLUP);
    pinMode(BTN_CALIBRATE, INPUT_PULLUP);

    _currentState = STATE_IDLE;
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

    if (calibratePressed && _currentState == STATE_IDLE) {
        _currentState = STATE_CALIBRATING;
        _calibrationStartTime = millis();
        _motor.turnLeft(90);   // rotate in place so every sensor sweeps the line
    }

    if (startPressed && (_currentState == STATE_STOPPED || _currentState == STATE_IDLE)) {
        _currentState = STATE_RUNNING;
        _pid.reset();
        _cruiseSpeed = BASE_SPEED;
        _lastRampTime = millis();
        _hasTrackedLine = false;
        _lastDirection = 0;
        _motor.stop();
    }

    if (stopPressed && _currentState == STATE_RUNNING) {
        _currentState = STATE_STOPPED;
        _motor.stop();
    }
}

void Robot::executeState() {
    switch (_currentState) {
        case STATE_CALIBRATING: {
            _sensor.calibrate();
            if (millis() - _calibrationStartTime >= CALIBRATION_DURATION_MS) {
                _motor.stop();
                _sensor.autoThreshold();
                _currentState = STATE_STOPPED;
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
    _sensor.readAnalog();
    _sensor.readDigital();
    _junction.update();
    // ---------- full-width black held long = finish/stop box ----------
    if (_junction.detectStopBox()) { handleStopBox(); return; }
    // ---------- full-width black bar: cross / T-junction ----------
   
    if (_junction.detectTJunction()) { handleTJunction(); return; }
     if (_junction.detectCross()) { handleCross(); return; }

    // ---------- line lost entirely: gap or genuine derailment ----------
    if (_junction.detectGap()) { handleGap(); return; }
    if (_junction.detectLostLine()) { searchLine(); return; }

    // ---------- sharp corners: commit to a pivot ----------
    if (_junction.detect90Left())  { handle90Left();  return; }
    if (_junction.detect90Right()) { handle90Right(); return; }

    // ---------- gentle diagonals: PID already steers this correctly,
    // these hooks just keep bookkeeping fresh a beat early ----------
    if (_junction.detect45Left())  { handle45Left();  }
    if (_junction.detect45Right()) { handle45Right(); }

    // ---------- normal line tracking ----------
    _hasTrackedLine = true;
    int error = _sensor.getError();
    _lastDirection = (error > 0) ? 1 : (error < 0 ? -1 : _lastDirection);

    int correction = _pid.compute(error);

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

    _motor.drive(leftSpeed, rightSpeed);
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
    _motor.turnRight(SEARCH_SPEED);
    _lastDirection = 1;
    _hasTrackedLine = true;
    _cruiseSpeed = BASE_SPEED;   // re-enter the straight-line ramp fresh once this clears
    _lastRampTime = millis();
}

void Robot::handle90Right() {
    _motor.turnLeft(SEARCH_SPEED);
    _lastDirection = -1;
    _hasTrackedLine = true;
    _cruiseSpeed = BASE_SPEED;
    _lastRampTime = millis();
}

void Robot::handleCross() {
_motor.turnRight(SEARCH_SPEED);

    _lastDirection = -1;
    _hasTrackedLine = true;
    _cruiseSpeed = BASE_SPEED;   // re-enter the straight-line ramp fresh once this clears
    _lastRampTime = millis();
}

void Robot::handleTJunction() {
    _motor.turnLeft(SEARCH_SPEED);

    _lastDirection = 1;
    _hasTrackedLine = true;
    _cruiseSpeed = BASE_SPEED;   // re-enter the straight-line ramp fresh once this clears
    _lastRampTime = millis();
}

void Robot::handleGap() {
    // Small dashed-line break - hold last output rather than react;
    // reacting to a single blank scan is what makes robots twitch on
    // dashed tracks.
    _motor.drive(_motor.getLastLeftSpeed(), _motor.getLastRightSpeed());
}

void Robot::handleWhiteLine() {
    // Provided for API completeness / manual use (e.g. call this
    // yourself if you want a different reaction to "any blank scan"
    // than the gap/lost timing split above uses). Not wired into
    // followLine()'s default dispatch since detectGap()/detectLostLine()
    // already cover every case where detectWhiteLine() is true.
    _motor.drive(_motor.getLastLeftSpeed(), _motor.getLastRightSpeed());
}

void Robot::searchLine() {
    unsigned long lostFor = _junction.whiteDuration();

    if (_hasTrackedLine && lostFor >= SEARCH_TIMEOUT_MS) {
        // Line was tracked successfully earlier this run and has now
        // been missing well past a reasonable search window - genuine
        // derailment, not a cold start. Fail-safe stop.
        _motor.stop();
        _currentState = STATE_STOPPED;
        return;
    }

    // Pivot toward whichever side the line was last seen on. If it's
    // never been seen yet this run (cold start slightly off-line, or
    // sitting in a start box), the `_hasTrackedLine` gate above keeps
    // this searching indefinitely instead of giving up.
    if (_lastDirection > 0) {
        _motor.turnRight(SEARCH_SPEED);
    } else if (_lastDirection < 0) {
        _motor.turnLeft(SEARCH_SPEED);
    } else {
        _motor.forward(SEARCH_SPEED);   // no history yet - creep forward
    }
}
void Robot::handleStopBox() {
    _motor.brake();              // TB6612 short-brake - fast, decisive stop
    _currentState = STATE_STOPPED;
}
