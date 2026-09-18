/*  Developments, edits, changes:
 *   1. 10th July 2026: Finalizing jogging, corrections, etc.
 * 
 */


#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "robot_arm_constants_variables.h"

// ---------------------------------------- code run -------------------------------------------//
void setup(){
  system_initialize();
}

unsigned long lastMovementTime = millis();

void loop(){
  // Only update the servos every 10 milliseconds (100 times per second)
  if (millis() - lastMovementTime >= 10) {
    robot_arm_update_movement();
    lastMovementTime = millis();
  }
  
  // Always listen for serial commands as fast as possible
  handleSerialCommands();
}
