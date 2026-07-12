#include "Globals.h"
#include "Robot.h"
#include "MotorDriver.h"
#include "SensorArray.h"
#include "PIDController.h"
#include "JunctionDetector.h"
#include<Arduino.h>

extern MotorDriver motor;
extern SensorArray sensor;
extern PIDController pid;
extern JunctionDetector junction;
extern RobotState currentState;
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
