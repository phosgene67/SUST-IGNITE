#include "Globals.h"
#include "MotorDriver.h"
#include "SensorArray.h"
#include "PIDController.h"
#include "JunctionDetector.h"
#include "Robot.h"

RobotState currentState = STATE_IDLE;
MotorDriver motor;
SensorArray sensor;
PIDController pid;
JunctionDetector junction;
Robot robot;