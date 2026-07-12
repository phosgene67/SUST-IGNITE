#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H
#include<Arduino.h>
class PIDController {
public:
    inline void begin();
    inline int compute(int error);
    inline void reset();

    // Gains are Q8 fixed-point: pass (real_gain * 256) as an integer.
    // e.g. a real gain of 0.70 -> kpQ8 = 179. AVR328P has no FPU, so
    // fixed-point integer math is the single biggest speed win
    // available on this hot path - no floats anywhere in here.
    inline void setTunings(int16_t kpQ8, int16_t kiQ8, int16_t kdQ8);
    inline void setOutputLimits(int minimum, int maximum);

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


#endif
