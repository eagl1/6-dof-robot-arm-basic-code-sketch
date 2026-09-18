// =====================================================================================================================
// FORWARD KINEMATICS - math = (physical - offset)
// =====================================================================================================================
void calculateForwardKinematics(void) {
  // Convert PHYSICAL angles (robot_arm_angles) to MATH angles
  // Formula: math = (physical - offset)
  float a0 = (robot_arm_angles[0] - angleOffset[0]) * DEG_TO_RAD;
  float a1 = (robot_arm_angles[1] - angleOffset[1]) * DEG_TO_RAD;
  float a2 = (robot_arm_angles[2] - angleOffset[2]) * DEG_TO_RAD;
  float a3 = (robot_arm_angles[3] - angleOffset[3]) * DEG_TO_RAD;
  float a4 = (robot_arm_angles[4] - angleOffset[4]) * DEG_TO_RAD;

  // --- WRIST CENTER POINT (WCP) ---
  // This is the end of L3, where the wrist starts
  float phi2 = a1 + a2;  // Forearm absolute angle relative to horizontal
  float reach = L2 * cos(a1) + L3 * cos(phi2);
  
  worldPosWcp[0] = reach * cos(a0);                    // X: forward/back
  worldPosWcp[1] = reach * sin(a0);                    // Y: left/right
  worldPosWcp[2] = L1 + L2 * sin(a1) + L3 * sin(phi2); // Z: height

  // --- TOOL CENTER POINT (TCP) ---
  // This is the tip of the gripper, offset by L4 from wrist
  float toolPitch = phi2 + a3;  // Absolute pitch of tool

  // debug1
  Serial.print("Tool pitch: "); Serial.println(toolPitch * RAD_TO_DEG);

  
  worldHandTcp[0] = worldPosWcp[0] + L4 * cos(a0) * cos(toolPitch);
  worldHandTcp[1] = worldPosWcp[1] + L4 * sin(a0) * cos(toolPitch);
  worldHandTcp[2] = worldPosWcp[2] + L4 * sin(toolPitch);

  // --- DEBUG OUTPUT ---
  Serial.println("FK ->");
  Serial.print("  WCP X:"); Serial.print(worldPosWcp[0], 2);
  Serial.print(" Y:");      Serial.print(worldPosWcp[1], 2);
  Serial.print(" Z:");      Serial.println(worldPosWcp[2], 2);
  Serial.print("  TCP X:"); Serial.print(worldHandTcp[0], 2);
  Serial.print(" Y:");      Serial.print(worldHandTcp[1], 2);
  Serial.print(" Z:");      Serial.println(worldHandTcp[2], 2);

  // CRITICAL: DO NOT touch tempPosWcp or tempHandTcp here!
  // tempPosWcp = command target (where we want to go)
  // worldPosWcp = actual feedback (where we actually are)
}

// =====================================================================================================================
// INVERSE KINEMATICS - physical = (math) + offset
// =====================================================================================================================
bool calculateInverseKinematics(void) {
  // Use your existing tempHandTcp for TCP target
  float tx = tempHandTcp[0];
  float ty = tempHandTcp[1];
  float tz = tempHandTcp[2];

  // Convert TCP → WCP (using your existing targetToolPitch)
  float baseAngle = atan2(ty, tx);
  float toolPitchRad = targetToolPitch * DEG_TO_RAD;
  
  float wcp_x = tx - L4 * cos(toolPitchRad) * cos(baseAngle);
  float wcp_y = ty - L4 * cos(toolPitchRad) * sin(baseAngle);
  float wcp_z = tz - L4 * sin(toolPitchRad);

  float r = sqrt(wcp_x * wcp_x + wcp_y * wcp_y);
  float s = wcp_z - L1;
  float distSq = r * r + s * s;
  float dist = sqrt(distSq);

  // REACH CHECK (moved from safety to here)
  float maxReach = (L2 + L3);  // Exact 20.50
  float minReach = abs(L2 - L3) + 0.5;
  
  if (dist > maxReach || dist < minReach || isnan(dist)) {
    Serial.print("IK FAIL: Out of reach dist = "); Serial.println(dist);
    return false;
  }

  // Base
  float a0_math = atan2(wcp_y, wcp_x);

  // Elbow
  float cos_a2 = (distSq - L2*L2 - L3*L3) / (2.0 * L2 * L3);
  cos_a2 = constrain(cos_a2, -1.0, 1.0);
  float a2_math = -acos(cos_a2);

  // Shoulder
  float alpha = atan2(s, r);
  float beta = atan2(L3 * sin(-a2_math), L2 + L3 * cos(-a2_math));
  float a1_math = alpha + beta;

  // Wrist
  float phi2 = a1_math + a2_math;
  float a3_math = toolPitchRad - phi2;
  float a4_math = targetToolRoll * DEG_TO_RAD;

  // Math → Physical (using your existing angleOffset)
  float raw0 = a0_math * RAD_TO_DEG + angleOffset[0];
  float raw1 = a1_math * RAD_TO_DEG + angleOffset[1];
  float raw2 = a2_math * RAD_TO_DEG + angleOffset[2];
  float raw3 = a3_math * RAD_TO_DEG + angleOffset[3];
  float raw4 = a4_math * RAD_TO_DEG + angleOffset[4];

  // Constrain
  raw0 = constrain(raw0, SERVO_MIN_DEG, SERVO_MAX_DEG);
  raw1 = constrain(raw1, SERVO_MIN_DEG, SERVO_MAX_DEG);
  raw2 = constrain(raw2, SERVO_MIN_DEG, SERVO_MAX_DEG);
  raw3 = constrain(raw3, SERVO_MIN_DEG, SERVO_MAX_DEG);
  raw4 = constrain(raw4, SERVO_MIN_DEG, SERVO_MAX_DEG);

  // Update your existing robot_arm_angles array
  robot_arm_angles[0] = raw0;
  robot_arm_angles[1] = raw1;
  robot_arm_angles[2] = raw2;
  robot_arm_angles[3] = raw3;
  robot_arm_angles[4] = raw4;


  // debug1
  Serial.print("Joint 3: math= "); Serial.print(a3_math * RAD_TO_DEG);
  Serial.print(" phys= "); Serial.println(raw3);

  // Update your existing servoDestination array
  for (int i = 0; i < 5; i++) {
    servoDestination[i] = (int)mapFloat(robot_arm_angles[i], -135.0, 135.0, SERVO_MIN_PWM, SERVO_MAX_PWM);
  }

  // Update feedback using your existing function name
  calculateForwardKinematics();

  // Update your existing tempPosWcp to match actual WCP
  tempPosWcp[0] = worldPosWcp[0];
  tempPosWcp[1] = worldPosWcp[1];
  tempPosWcp[2] = worldPosWcp[2];

  robot_arm_task = ROBOT_ARM_RUN;
  return true;
}
