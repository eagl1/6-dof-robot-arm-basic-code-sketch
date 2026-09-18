
      
      // BL BR SU SD EU ED WU WD WCW WCCW

      // XF XR YL YR ZU ZD

      // Basic PITCH_VERT jogging
      // GOTO_2D X15 Y0 Z5


       /* calibration process:
       *  
       *  1. point to the ceiling
       *  GOTO_2D X0 Y0 Z(L1 + L2 + L3)30
       *  --> GOTO_2D X0 Y0 Z30, GOTO_PWM P1950 S1 just to move the shoulder
       *  
      // GOTO_2D X10  Y0  Z0
      GOTO_PWM P1000 S0 GOTO_PWM P2000 S0
      GOTO_PWM P1200 S1
      GOTO_PWM P1900 S1
      GOTO_PWM P1900 S2
      GOTO_PWM P2550 S3
      GOTO_PWM P2550 S4
      GOTO_PWM P500 S8
      // RESET
       *  2. L pose
       *  GOTO_2D X15 Y0 Z15
       *  
       */
