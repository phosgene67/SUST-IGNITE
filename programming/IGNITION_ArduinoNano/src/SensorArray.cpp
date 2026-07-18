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
    initADC();

    // Optional IR emitter control, if your QTR-6A board has one wired
    // to a digital pin instead of always-on:
    // pinMode(4, OUTPUT);
    // digitalWrite(4, HIGH);
}
void SensorArray::initADC()
{
    // AVcc reference
    ADMUX = (1 << REFS0);
    // Enable ADC
    // Prescaler = 32
    // ADC Clock = 16MHz / 32 = 500kHz
    ADCSRA =
        (1 << ADEN)  |
        (1 << ADPS2) |
        (1 << ADPS0);
    DIDR0 = 0x3F;          // Disable digital input on A0-A5
    
}
inline uint16_t SensorArray:: readADC(uint8_t channel)
{
    ADMUX = _BV(REFS0) | channel;
    ADCSRA |= _BV(ADSC);
    while (ADCSRA & _BV(ADSC));
    return ADC;
}
void SensorArray:: readAnalog()
{
    sensorValues[0] = readADC(5);
    sensorValues[1] = readADC(4);
    sensorValues[2] = readADC(3);
    sensorValues[3] = readADC(2);
    sensorValues[4] = readADC(1);
    sensorValues[5] = readADC(0);
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