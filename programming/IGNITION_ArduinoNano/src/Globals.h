#ifndef GLOBALS_H
#define GLOBALS_H

#include "Config.h"

enum RobotState {
    STATE_IDLE,
    STATE_CALIBRATING,
    STATE_RUNNING,
    STATE_STOPPED
};

extern RobotState currentState;

#endif