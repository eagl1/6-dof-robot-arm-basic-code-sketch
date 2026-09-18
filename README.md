This is just a basic sketch, the remaining stages are: design a teach pendant controller using hmi or standalone controller.
The current system of controling the robot arm is with the use of serial commands in `SerialCommands` file.
Depending on the type/brand of servos you're using, you have to set your starting position in `home_position` array declared in `robot_arm_constants_variables.h` line 101.
After that, you have to design your robot arm working area.
