/*
A FreeRTOS powered controller for a 2 wheel robot.

Tasks | Descending priority order:
- Stepper Controllers (2 of them)
- ESP-NOW Communicator
- Kinematics Calculator
- Position Estimator
- Battery Monitor
- LED Status Indicator

More info on each task within their header files.

*/

#include <Arduino.h>
#include "steppers.h"


void setup() {
  Serial.begin(115200);
  Serial.println("INITIALIZED");
}

void loop() {
  
}
