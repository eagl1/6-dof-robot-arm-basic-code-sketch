void servo_run(uint8_t servo_number) {
  // --- INITIALIZATION ---
  if (servoDestination[servo_number] != servoPosition[servo_number] && lock[servo_number] == 0) {
    lock[servo_number] = 1;
    
    if (servoPosition[servo_number] < servoDestination[servo_number]) {
      direction[servo_number] = CW;
    } else {
      direction[servo_number] = CCW;
    }
  }

  // --- MOVEMENT ---
  if (lock[servo_number]) {
    
    // Clockwise
    if (direction[servo_number] == CW) {
      servoPosition[servo_number] += robot_speed;
      if (servoPosition[servo_number] > servoDestination[servo_number]) {
        servoPosition[servo_number] = servoDestination[servo_number];
      }
    }
    // Counter-clockwise
    else {
      servoPosition[servo_number] -= robot_speed;
      if (servoPosition[servo_number] < servoDestination[servo_number]) {
        servoPosition[servo_number] = servoDestination[servo_number];
      }
    }
    
    // --- SAFETY LIMITS ---
    servoPosition[servo_number] = constrain(
      servoPosition[servo_number],
      SERVO_MIN_PWM_WORLD[servo_number],
      SERVO_MAX_PWM_WORLD[servo_number]
    );
    
    // --- APPLY PWM ---
    pwm.setPWM(servo_number, 0, servoPosition[servo_number]);
    
    // --- CHECK COMPLETION ---
    if (servoPosition[servo_number] == servoDestination[servo_number]) {
      lock[servo_number] = 0;
    }
  }
}

void robot_arm_update_movement(void) {
  if (robot_arm_task != ROBOT_ARM_RUN) {
    //Serial.println("robot arm update busy");
    return;
  }
  
  bool all_servos_finished = true;
  
  for (uint8_t i = 0; i < ALL_SERVOS_COUNT; i++) {
    servo_run(i);
    
    if (lock[i] == 1) {
      all_servos_finished = false;
    }
  }
  
  if (all_servos_finished) {
    robot_arm_task = ROBOT_ARM_IDLE;
    Serial.println("robot_arm_update_movement is done\n");
    Serial.println("destination array:");
    for (uint8_t i = 0; i < ALL_SERVOS_COUNT; i++) {
      Serial.print(servoDestination[i]);Serial.print(" ");
    }
    Serial.println();
  }
}
