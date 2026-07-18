#ifndef JUNCTIONDETECTOR_H
#define JUNCTIONDETECTOR_H
#include<Arduino.h>

class SensorArray;

class JunctionDetector {
public:
    explicit JunctionDetector(SensorArray& sensor);

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
    SensorArray& _sensor;
    uint8_t currentMask = 0;
    uint8_t previousMask = 0;

    bool allBlackActive = false;
    unsigned long allBlackStartTime = 0;

    bool allWhiteActive = false;
    unsigned long allWhiteStartTime = 0;
};
#endif
