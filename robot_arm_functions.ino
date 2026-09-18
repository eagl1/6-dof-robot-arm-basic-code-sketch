void system_initialize(void) {
  // initialize serial communication
  Serial.begin(115200);

  // initialize servo system
  pwm.begin();
  pwm.setPWMFreq(330);

  robot_speed = SERVO_SLOW_SPEED;
  //robot_arm_power_on_reset = false; // no power on reset is activated
  jogMode = CARTESIAN_MODE;
  
  // --- STEP 1: Set Home Positions and Calculate Offsets ---
  sync_to_home();

  Serial.println("System Initialized with:");
  Serial.print("1. Wcp Offsets   : ");
  Serial.print("X: "); Serial.print(tempPosWcp[0]);
  Serial.print(", Y: "); Serial.print(tempPosWcp[1]);
  Serial.print(", Z: "); Serial.println(tempPosWcp[2]);
  Serial.print("2. robot_speed   : "); Serial.println("SLOW");
  Serial.print("3. jogMode       : ");  Serial.println("CARTESIAN");
  Serial.print("4. Tool status   : ");  Serial.println("attached");
  
  
}

// ==================================================================== SYSTEM COMMANDS
void handleSerialCommands(void) {
  if (!Serial.available()) { return; }
  
  String command = Serial.readStringUntil('\n');
  command.trim();
  command.toUpperCase();

  // ==================================================================== RESET
  if (command == "RESET") {
    //robot_arm_power_on_reset = true;
    sync_to_home();
    Serial.println("POWER ON RESET IS DONE");
  }

  // ==================================================================== STOP
  else if (command == "STOP") {
    robot_arm_task = ROBOT_ARM_IDLE;
    Serial.println("Stopped.");
  }

  // ==================================================================== GRIPPER CONTROL
  else if (command == "GRIP_OPEN") {
    setGripper(GRIPPER_OPEN);
  }
  else if (command == "GRIP_HALF") {
    setGripper(GRIPPER_HALF);
  }
  else if (command == "GRIP_CLOSE") {
    setGripper(GRIPPER_CLOSE);
  }

  // ==================================================================== SYNC
  else if (command == "MODE_JOINT") {
    jogMode = JOINT_MODE;
    Serial.println("Mode: Joint space");
  }
  else if (command == "MODE_CART") {
    jogMode = CARTESIAN_MODE;
    Serial.println("Mode: Cartesian");
    // Restore Cartesian STEP commands (XF, XR, etc.)
  }

  // ==================================================================== TOOL SETTING
  else if (command == "TOOL_ON") {
    if (isToolAttached) { 
      Serial.println("Tool is already attached."); 
      return; 
    }
    
    isToolAttached = true;
    L4 = L4_BASE + TOOL_LENGTH;
    
    // CRITICAL: We must re-calculate forward kinematics right now!
    // The physical arm hasn't moved, but the TCP just jumped forward by 12cm.
    calculateForwardKinematics();
    tempHandTcp[0] = worldHandTcp[0];
    tempHandTcp[1] = worldHandTcp[1];
    tempHandTcp[2] = worldHandTcp[2];
    
    Serial.print("Tool Attached. L4 is now: "); Serial.println(L4);
    return;
  }
  
  else if (command == "TOOL_OFF") {
    if (!isToolAttached) { 
      Serial.println("Tool is already off."); 
      return; 
    }
    
    isToolAttached = false;
    L4 = L4_BASE; // Shrink the math back to the bare mounting plate
    
    // Sync the system so the new TCP matches the WCP + Base offset
    calculateForwardKinematics();
    tempHandTcp[0] = worldHandTcp[0];
    tempHandTcp[1] = worldHandTcp[1];
    tempHandTcp[2] = worldHandTcp[2];
    
    Serial.print("Tool Detached. L4 is now: "); Serial.println(L4);
    return;
  }

  // ==================================================================== TOOL ORIENTATION MODES
  else if (command == "PITCH_HORIZ") {
    targetToolPitch = 0.0;
    Serial.println("Pitch Mode: HORIZONTAL (0 deg)");
    // Instantly swing the wrist to horizontal while keeping TCP in place
    calculateInverseKinematics();
  }

  else if (command == "PITCH_VERT") {
    targetToolPitch = -90.0;
    Serial.println("Pitch Mode: VERTICAL (-90 deg)");
    // Instantly swing the wrist to point straight down at the desk
    calculateInverseKinematics();
  }

  else if (command.startsWith("PITCH_FREE ")) {
    // Example usage: "PITCH_FREE -45" or "PITCH_FREE 30"
    float newPitch = command.substring(11).toFloat();

    // Constrain to safe physical limits so the servo doesn't crash into the arm
    targetToolPitch = constrain(newPitch, -100.0, 90.0);

    Serial.print("Pitch Mode: FREE / CUSTOM (");
    Serial.print(targetToolPitch);
    Serial.println(" deg)");

    calculateInverseKinematics();
  }
  else if (command.startsWith("TRACK_ON")) {
    // Example usage: "TRACK_ON X15 Y0 Z0" (e.g., an object on the desk)
    trackObjX = parseValue(command, 'X');
    trackObjY = parseValue(command, 'Y');
    trackObjZ = parseValue(command, 'Z');
    isTrackingTarget = true;

    Serial.print("Tracking ON. Object located at -> X:"); Serial.print(trackObjX);
    Serial.print(" Y:"); Serial.print(trackObjY);
    Serial.print(" Z:"); Serial.println(trackObjZ);

    // Calculate the new pitch and immediately swing the wrist to look at it!
    updateTrackingPitch();
    calculateInverseKinematics(); 
  }
  
  else if (command == "TRACK_OFF") {
    isTrackingTarget = false;
    Serial.println("Tracking OFF. Pitch locked at current angle.");
  }

  // ==================================================================== GOTO_2D
  else if (command.startsWith("GOTO_2D")) {
    float tx = parseValue(command, 'X'); 
    float ty = parseValue(command, 'Y');
    float tz = parseValue(command, 'Z');

    Serial.print("Command GOTO_2D X"); Serial.print(tx); 
    Serial.print(" Y"); Serial.print(ty); 
    Serial.print(" Z"); Serial.println(tz);

    // Load into TCP target
    tempHandTcp[0] = tx;
    tempHandTcp[1] = ty;
    tempHandTcp[2] = tz;
    
    // Convert TCP → WCP
    float baseAngle = atan2(ty, tx);
    float toolPitchRad = targetToolPitch * DEG_TO_RAD;
    
    tempPosWcp[0] = tx - L4 * cos(toolPitchRad) * cos(baseAngle);
    tempPosWcp[1] = ty - L4 * cos(toolPitchRad) * sin(baseAngle);
    tempPosWcp[2] = tz - L4 * sin(toolPitchRad);

    if (!checkIfCoordinateIsSafe()) {
      Serial.println("GOTO_2D blocked: unsafe");
      return;
    }
    calculateInverseKinematics();
  }

  // ==================================================================== GOTO_PWM
  else if (command.startsWith("GOTO_PWM")) {
    int p_idx = command.indexOf('P', 9); 
    int s_idx = command.indexOf('S', 9);

    if (p_idx != -1 && s_idx != -1) {
      int p_start = p_idx + 1;
      int p_end = command.indexOf(' ', p_start);
      if (p_end == -1 || p_end > s_idx) { p_end = s_idx; }

      String p_str = command.substring(p_start, p_end);
      p_str.trim();
      uint16_t target_p = p_str.toInt();

      int s_start = s_idx + 1;
      String s_str = command.substring(s_start);
      s_str.trim();
      uint16_t target_s = s_str.toInt();

      Serial.print("Command GOTO_PWM P"); Serial.print(p_str); 
      Serial.print(" S"); Serial.println(s_str);

      if (target_s < ALL_SERVOS_COUNT) {
        target_p = constrain(target_p, SERVO_MIN_PWM_WORLD[target_s], SERVO_MAX_PWM_WORLD[target_s]);
        
        // DO NOT write PWM directly! Use servo_run via destination
        servoDestination[target_s] = target_p;
        
        // Convert to angle for FK
        float physicalAngle = mapFloat(target_p, SERVO_MIN_PWM_WORLD[target_s], SERVO_MAX_PWM_WORLD[target_s], -135, 135);
        robot_arm_angles[target_s] = physicalAngle;
        
        // Update FK
        calculateForwardKinematics();
        
        // Sync targets
        tempPosWcp[0] = worldPosWcp[0];
        tempPosWcp[1] = worldPosWcp[1];
        tempPosWcp[2] = worldPosWcp[2];
        
        tempHandTcp[0] = worldHandTcp[0];
        tempHandTcp[1] = worldHandTcp[1];
        tempHandTcp[2] = worldHandTcp[2];
        
        // Trigger movement
        robot_arm_task = ROBOT_ARM_RUN;
        update_display_numerics = 1;
      }
      else{
        pwm.setPWM(target_s, 0, target_p);
      }
    }
  }

  // ==================================================================== STEP (Joint Space Jogging)
  else if (jogMode == JOINT_MODE) {
    if (robot_arm_task != ROBOT_ARM_IDLE) { 
      Serial.println("Robot busy"); 
      return; 
    }

    bool validCommand = true;

    if      (command == "BL")   { robot_arm_angles[0] += jogStepDeg; Serial.println("BL");   }  // Base left
    else if (command == "BR")   { robot_arm_angles[0] -= jogStepDeg; Serial.println("BR");   }  // Base right
    else if (command == "SU")   { robot_arm_angles[1] += jogStepDeg; Serial.println("SU");   }  // Shoulder up
    else if (command == "SD")   { robot_arm_angles[1] -= jogStepDeg; Serial.println("SD");   }  // Shoulder down
    else if (command == "EU")   { robot_arm_angles[2] += jogStepDeg; Serial.println("EU");   }  // Elbow up
    else if (command == "ED")   { robot_arm_angles[2] -= jogStepDeg; Serial.println("ED");   }  // Elbow down
    else if (command == "WU")   { robot_arm_angles[3] += jogStepDeg; Serial.println("WU");   }  // Wrist pitch up
    else if (command == "WD")   { robot_arm_angles[3] -= jogStepDeg; Serial.println("WD");   }  // Wrist pitch down
    else if (command == "WCW")  { robot_arm_angles[4] += jogStepDeg; Serial.println("WCW");  }  // Wrist roll left
    else if (command == "WCCW") { robot_arm_angles[4] -= jogStepDeg; Serial.println("WCCW"); }  // Wrist roll right
    else {
      Serial.println("Unknown STEP, USE: BL BR SU SD EU ED WU WD WCW WCCW");
      validCommand = false;
      return;
    }

    // Constrain, FK calculate, update PWM, and trigger movement here
    if (validCommand) {
      // Constrain all angles to servo limits
      for (int i = 0; i < 5; i++) {
        robot_arm_angles[i] = constrain(robot_arm_angles[i], SERVO_MIN_DEG, SERVO_MAX_DEG);
      }
  
      // Calculate where this pose puts the TCP
      calculateForwardKinematics();
  
      // Update PWM destinations
      for (int i = 0; i < 5; i++) {
        servoDestination[i] = (int)mapFloat(robot_arm_angles[i], -135.0, 135.0, SERVO_MIN_PWM, SERVO_MAX_PWM);
      }
  
      // Check if TCP is in safe workspace (optional floor check)
      if (worldHandTcp[2] < 2.0) {
        Serial.println("WARNING: TCP near floor");
        // Optional: block or allow with warning
      }
  
      // Sync TCP target to actual for display
      tempHandTcp[0] = worldHandTcp[0];
      tempHandTcp[1] = worldHandTcp[1];
      tempHandTcp[2] = worldHandTcp[2];
      
      tempPosWcp[0] = worldPosWcp[0];
      tempPosWcp[1] = worldPosWcp[1];
      tempPosWcp[2] = worldPosWcp[2];
  
      // Trigger movement
      robot_arm_task = ROBOT_ARM_RUN;
  
      Serial.print("Joints: ");
      Serial.print(robot_arm_angles[0]); Serial.print(", ");
      Serial.print(robot_arm_angles[1]); Serial.print(", ");
      Serial.print(robot_arm_angles[2]); Serial.print(", ");
      Serial.print(robot_arm_angles[3]); Serial.print(", ");
      Serial.println(robot_arm_angles[4]);
  
      Serial.print("TCP: X="); Serial.print(worldHandTcp[0]);
      Serial.print(" Y="); Serial.print(worldHandTcp[1]);
      Serial.print(" Z="); Serial.println(worldHandTcp[2]);      
    }

  }

// ==================================================================== Cartesian Jogging
    else if (jogMode == CARTESIAN_MODE) {    
      if (robot_arm_task != ROBOT_ARM_IDLE) { 
        Serial.println("Robot busy");
        return; 
      }

    // Set how far the arm moves per step (e.g., 1.0 cm)
    float cartStep = 1.0;

    // Save current targets in case the step is unsafe or out of reach
    float oldTx = tempHandTcp[0];
    float oldTy = tempHandTcp[1];
    float oldTz = tempHandTcp[2];

    // Modify the target TCP coordinates based on the command
    if      (command == "XF") {tempHandTcp[0] += cartStep; Serial.println("XF"); } // Forward
    else if (command == "XR") {tempHandTcp[0] -= cartStep; Serial.println("XR"); } // Backward
    else if (command == "YL") {tempHandTcp[1] += cartStep; Serial.println("YL"); } // Left
    else if (command == "YR") {tempHandTcp[1] -= cartStep; Serial.println("YR"); } // Right
    else if (command == "ZU") {tempHandTcp[2] += cartStep; Serial.println("ZU"); } // Up
    else if (command == "ZD") {tempHandTcp[2] -= cartStep; Serial.println("ZD"); } // Down
    else {
      Serial.println("Unknown Cartesian STEP.");
      return;
    }
    
    // Dynamically update the pitch angle before doing the safety check and IK math!
    // this should work only if "isTrackingTarget" is enabled
    updateTrackingPitch(); 

    // 1. Run safety checks (floor, ceiling, base singularity)
    if (!checkIfCoordinateIsSafe()) {
      Serial.println("STEP blocked: unsafe workspace");
      // Revert to old targets
      tempHandTcp[0] = oldTx; tempHandTcp[1] = oldTy; tempHandTcp[2] = oldTz;
      return;
    }


    if (!calculateInverseKinematics()) {
      // IK failed (usually means it's out of reach)
      // The calculateInverseKinematics function already prints "IK FAIL"
      // We just need to revert the targets so the system doesn't get out of sync
      tempHandTcp[0] = oldTx; tempHandTcp[1] = oldTy; tempHandTcp[2] = oldTz;
    }
  }
}

// ============================================
// SETUP
// ============================================

void sync_to_home() {
  robot_arm_task = ROBOT_ARM_IDLE;

//  if(!robot_arm_power_on_reset){
//    for (uint8_t i = 0; i < 5; i++) {
//      servoPosition[i] = rest_position[i];
//      servoDestination[i] = home_position[i];
//      float physicalAngle = mapFloat((float)home_position[i], SERVO_MIN_PWM, SERVO_MAX_PWM, -135.0, 135.0);
//      angleOffset[i] = physicalAngle;
//      robot_arm_angles[i] = physicalAngle;  // Software thinks we're at home math angles
//    }    
//  }
//  else{
    for (uint8_t i = 0; i < 5; i++) {
      servoPosition[i] = home_position[i] - SERVO_PULSE_PER_DEGREE;// --> this line wasn't here, added to solve the startup problem
      servoDestination[i] = home_position[i];
      float physicalAngle = mapFloat((float)home_position[i], SERVO_MIN_PWM, SERVO_MAX_PWM, -135.0, 135.0);
      angleOffset[i] = physicalAngle;
      robot_arm_angles[i] = physicalAngle;  // Software thinks we're at home math angles
    }
//    robot_arm_power_on_reset = false;
//  }


// -> ADD THIS: Initialize the Gripper (Servo Index 5)
  servoPosition[5] = home_position[5];
  servoDestination[5] = home_position[5];
  pwm.setPWM(5, 0, servoPosition[5]);

  calculateForwardKinematics();
  
  // Sync targets to home
  tempPosWcp[0] = worldPosWcp[0];
  tempPosWcp[1] = worldPosWcp[1];
  tempPosWcp[2] = worldPosWcp[2];
  
  tempHandTcp[0] = worldHandTcp[0];
  tempHandTcp[1] = worldHandTcp[1];
  tempHandTcp[2] = worldHandTcp[2];

  targetToolPitch = 0.0;
  targetToolRoll  = 0.0;

  robot_arm_task = ROBOT_ARM_RUN;
  Serial.println("sync_to_home activated ..");
  Serial.println("=== HOMING: Rest → L-Pose ===");
}

// ==============================================================================================================
//                                                UTILS
// ==============================================================================================================
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float parseValue(String cmd, char axis) {
  int index = cmd.indexOf(axis); // Find where 'X', 'Y', or 'Z' is in the text
  if (index == -1) {return 0.0f;}  // If the letter isn't found, return 0
  
  // Cut the string starting from the letter and convert to a number
  return cmd.substring(index + 1).toFloat(); 
}

// ==============================================================================================================
//                                                ...
// ==============================================================================================================
//============================================================================== updateTrackingPitch
void updateTrackingPitch() {
  if (!isTrackingTarget) return;

  // Calculate the distance from our temporary TCP target to the physical object
  float dx = trackObjX - tempHandTcp[0];
  float dy = trackObjY - tempHandTcp[1];
  float dz = trackObjZ - tempHandTcp[2];

  // Get the 2D horizontal distance to the object
  float dist_xy = sqrt(dx * dx + dy * dy);

  // Calculate the required pitch angle using atan2(vertical, horizontal)
  float pitchRad = atan2(dz, dist_xy);

  // Update the global targetToolPitch (converting rads back to degrees)
  targetToolPitch = pitchRad * RAD_TO_DEG;

  // Protect the servos by keeping the wrist within safe physical limits
  targetToolPitch = constrain(targetToolPitch, -100.0, 90.0);
}


//============================================================================== checkIfCoordinateIsSafe

bool checkIfCoordinateIsSafe(void) {
  // Read TCP target from your existing global
  float tx = tempHandTcp[0];
  float ty = tempHandTcp[1];
  float tz = tempHandTcp[2];

  // Convert TCP → WCP using your existing targetToolPitch
  float baseAngle = atan2(ty, tx);
  float toolPitchRad = targetToolPitch * DEG_TO_RAD;
  
  float wcp_x = tx - L4 * cos(toolPitchRad) * cos(baseAngle);
  float wcp_y = ty - L4 * cos(toolPitchRad) * sin(baseAngle);
  float wcp_z = tz - L4 * sin(toolPitchRad);

  // Check base singularity (existing check)
  float horizontalDist = sqrt(wcp_x * wcp_x + wcp_y * wcp_y);
  if (horizontalDist < 2.0) {
    Serial.println("SAFETY: Base singularity");
    return false;
  }

  // Floor check (existing check)
  if (wcp_z < 2.0) {
    Serial.println("SAFETY: Too close to surface");
    return false;
  }

  // Ceiling check (existing check)
  if (wcp_z > 29.0) {
    Serial.println("SAFETY: Above max height");
    return false;
  }

  // REACH CHECK REMOVED — let calculateInverseKinematics() handle it
  // IK will return false if WCP is out of reach

  return true;
}

// ==============================================================================================================
// Metal Claw Gripper function 
// ==============================================================================================================
void setGripper(int val) {
  if(val == GRIPPER_OPEN){
    Serial.print("Gripper: OPENING to ");
  } else if(val == GRIPPER_HALF) {
    Serial.print("Gripper: HALFWAY to ");
  } else if(val == GRIPPER_CLOSE) {
    Serial.print("Gripper: CLOSING to ");
  } else {
    Serial.println("ERROR: Invalid Gripper PWM Value");
    return;
  }
  
  Serial.println(val);
  servoDestination[5] = val;
  robot_arm_task = ROBOT_ARM_RUN;
}
