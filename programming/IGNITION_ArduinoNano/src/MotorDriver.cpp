#include "Config.h"
#include "MotorDriver.h"
#include<Arduino.h>
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
