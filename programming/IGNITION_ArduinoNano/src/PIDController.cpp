#include "PIDController.h"
#include<Arduino.h>
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