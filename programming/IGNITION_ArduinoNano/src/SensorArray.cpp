#include "Config.h"
#include "SensorArray.h"
#include<Arduino.h>
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
    readAnalog();

    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        uint16_t val = sensorValues[i];

        if (val < sensorMin[i])
            sensorMin[i] = val;

        if (val > sensorMax[i])
            sensorMax[i] = val;
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