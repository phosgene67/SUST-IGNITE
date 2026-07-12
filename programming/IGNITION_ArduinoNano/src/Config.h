#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

#define PWMA 3      // PD3 = OC2B (Timer2)  - RIGHT motor PWM
#define AI1  7      // PD7                   - RIGHT motor dir
#define AI2  6      // PD6                   - RIGHT motor dir
#define STBY 8      // PB0                   - driver standby/enable
#define PWMB 10     // PB2 = OC1B (Timer1)  - LEFT motor PWM
#define BI1  11     // PB3                   - LEFT motor dir
#define BI2  12     // PB4                   - LEFT motor dir

#define BTN_START     5
#define BTN_STOP      2
#define BTN_CALIBRATE 9

#define NUM_SENSORS 6

const uint8_t sensorPins[NUM_SENSORS] = { A5, A4, A3, A2, A1, A0 };
const int sensorWeight[NUM_SENSORS] = { -250, -150, -50,
                                           50,  150, 250 };

#define POS_ALL_WHITE (-9999)
#define POS_ALL_BLACK  9999
#define BASE_SPEED               180
#define MIN_SPEED                110
#define CRUISE_MAX_SPEED         150
#define SEARCH_SPEED             120
#define ERROR_FULL_SCALE         250
#define STRAIGHT_ERROR_THRESHOLD 25
#define RAMP_INTERVAL_MS         15UL
#define RAMP_STEP                2
#define STOP_BOX_HOLD_MS 100UL   // full-black held this long = filled finish box, not a thin junction bar
#define GAP_COAST_MS      100UL     // bridges small dashed-line gaps
#define SEARCH_TIMEOUT_MS 2000UL   // give up & stop past this, if line was ever tracked
#define CROSS_HOLD_MS     10UL    // full-black held this long = probable T, not a quick crossing

#define TURN_45_MIN_ERROR 100
#define TURN_90_MIN_ERROR 200

#define LEFT_EXTREME_MASK  0b000001   // sensor 0 only
#define RIGHT_EXTREME_MASK 0b100000   // sensor 5 only
#define LEFT_HALF_MASK     0b000111   // sensors 0-2
#define RIGHT_HALF_MASK    0b111000   // sensors 3-5
#define ALL_SENSORS_MASK   0b111111   // all 6 sensors on-line

#define CALIBRATION_DURATION_MS 5000UL
#define DEBOUNCE_MS 25UL

#endif