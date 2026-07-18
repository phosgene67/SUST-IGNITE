#ifndef IGNITION_SENSORARRAY_H
#define IGNITION_SENSORARRAY_H
#include "Config.h"
#include<Arduino.h>
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
    void initADC();
     uint16_t readADC(uint8_t channel);
    int sensorValues[NUM_SENSORS];
    uint16_t sensorMin[NUM_SENSORS];
    uint16_t sensorMax[NUM_SENSORS];
    uint16_t threshold[NUM_SENSORS];
    uint8_t onLineMask = 0;
    uint8_t sensorOnState = 0;   // Schmitt-trigger memory per sensor

    static const int HYSTERESIS_COUNTS = 15;
    static const int CENTERED_ERROR_THRESHOLD = 20;
};
#endif
