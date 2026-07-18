#include <Arduino.h>
#include "MotorDriver.h"
#include "SensorArray.h"
#include "PIDController.h"
#include "JunctionDetector.h"
#include "Robot.h"

MotorDriver motor;
SensorArray sensor;
PIDController pid;
JunctionDetector junction(sensor);
Robot robot(motor, sensor, pid, junction);

void setup() {
    robot.begin();
}

void loop() {
    robot.update();
}
