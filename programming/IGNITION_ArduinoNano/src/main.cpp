#include <Arduino.h>
#include "Robot.h"

extern Robot robot;

void setup() {
    robot.begin();
}

void loop() {
    robot.update();
}
