#ifndef ROBOT_H
#define ROBOT_H
#include "Config.h"
#include<Arduino.h>
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
#endif
