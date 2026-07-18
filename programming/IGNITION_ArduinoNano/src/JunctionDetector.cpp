#include "JunctionDetector.h"
#include "SensorArray.h"
#include<Arduino.h>

JunctionDetector::JunctionDetector(SensorArray& sensor) : _sensor(sensor) {}

void JunctionDetector::begin() {
    currentMask = 0;
    previousMask = 0;
    allBlackActive = false;
    allBlackStartTime = 0;
    allWhiteActive = false;
    allWhiteStartTime = 0;
}

void JunctionDetector::update() {
    previousMask = currentMask;
    currentMask = _sensor.getMask();
    unsigned long now = millis();

    if (currentMask == ALL_SENSORS_MASK) {
        if (!allBlackActive) {
            allBlackActive = true;
            allBlackStartTime = now;
        }
    } else {
        allBlackActive = false;
    }

    if (currentMask == 0x00) {
        if (!allWhiteActive) {
            allWhiteActive = true;
            allWhiteStartTime = now;
        }
    } else {
        allWhiteActive = false;
    }
}

unsigned long JunctionDetector::whiteDuration() const {
    return allWhiteActive ? (millis() - allWhiteStartTime) : 0;
}

unsigned long JunctionDetector::blackDuration() const {
    return allBlackActive ? (millis() - allBlackStartTime) : 0;
}

bool JunctionDetector::detect45Left() {
    // Only meaningful for a partial mask - caller guarantees this by
    // checking cross/T/white cases first.
    int err = _sensor.getError();
    return (err <= -TURN_45_MIN_ERROR && err > -TURN_90_MIN_ERROR) &&
           !(currentMask & RIGHT_HALF_MASK);
}

bool JunctionDetector::detect45Right() {
    int err = _sensor.getError();
    return (err >= TURN_45_MIN_ERROR && err < TURN_90_MIN_ERROR) &&
           !(currentMask & LEFT_HALF_MASK);
}

bool JunctionDetector::detect90Left() {
    int err = _sensor.getError();

    return (err <= -TURN_90_MIN_ERROR) &&
           (currentMask & LEFT_EXTREME_MASK) &&
           !(currentMask & RIGHT_HALF_MASK);
}

bool JunctionDetector::detect90Right() {
    int err = _sensor.getError();

    return (err >= TURN_90_MIN_ERROR) &&
           (currentMask & RIGHT_EXTREME_MASK) &&
           !(currentMask & LEFT_HALF_MASK);
}

bool JunctionDetector::detectCross() {
    // Exiting a full-black bar back into a partial line pattern means
    // the line continues straight ahead - the bar was a simple
    // crossing, not a dead-ended branch. No timing needed: this is a
    // one-frame transition check.
    return previousMask == ALL_SENSORS_MASK &&
           currentMask != ALL_SENSORS_MASK &&
           currentMask != 0x00;
}

bool JunctionDetector::detectTJunction() {
   if (currentMask != ALL_SENSORS_MASK)
        return false;
   if (previousMask == 0x00 || previousMask == ALL_SENSORS_MASK) return false;

    bool wasOnTrunk = (previousMask & 0b001100) != 0;

    return wasOnTrunk;

}

bool JunctionDetector::detectGap() {
    // Short break in the line (dashed segment) - still inside the
    // bridging window, so the caller should coast, not react.
    return allWhiteActive && whiteDuration() < GAP_COAST_MS;
}

bool JunctionDetector::detectWhiteLine() {
    // Instantaneous "nothing detected this scan". detectGap()/
    // detectLostLine() add timing judgment on top of this raw signal.
    return currentMask == 0x00;
}

bool JunctionDetector::detectLostLine() {
    // Past the gap-bridging window - either a sharp turn the reactive
    // pivot logic hasn't caught, or a genuine derailment. Robot::
    // searchLine() resolves which by also checking hasTrackedLine and
    // SEARCH_TIMEOUT_MS.
    return allWhiteActive && whiteDuration() >= GAP_COAST_MS;
}
bool JunctionDetector::detectStopBox() {
    // A finish/stop box is a solid filled rectangle - wider than any
    // junction crossbar. Real T/cross junctions self-clear within a
    // loop or two of the pivot/drive-through starting (the array
    // rotates or drives off a thin bar). If full-black survives well
    // past that, it's not a junction anymore - it's a filled box.
    return allBlackActive && blackDuration() >= STOP_BOX_HOLD_MS;
}
