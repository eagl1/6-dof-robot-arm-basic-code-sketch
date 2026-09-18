#ifndef robot_arm_constants_variables_h
#define robot_arm_constants_variables_h

// ------------------------------------- main task defines  -------------------------------------- //
// task status
#define NOT_FINISHED          1
#define FINISHED              0

// servo direction
#define CW                    0
#define CCW                   1

// servo direction
#define JOINT_MODE            0
#define CARTESIAN_MODE        1

// gripper open/close values
#define GRIPPER_CLOSE         2000
#define GRIPPER_HALF          2250
#define GRIPPER_OPEN          2500

//#define SERVO_MIN_PWM         500
//#define SERVO_MID_PWM         2000
//#define SERVO_MAX_PWM         3500

const float SERVO_MIN_PWM = 500.0;
const float SERVO_MID_PWM = 2000.0;
const float SERVO_MAX_PWM = 3500.0;

const float SERVO_MIN_DEG = -135.0;
const float SERVO_MAX_DEG =  135.0;

// number of servos used in this project
#define ALL_SERVOS_COUNT      6 // gripper has standalone function
#define ARM_SERVOS_COUNT      3
#define HAND_SERVOS_COUNT     2

// ------------------------------------ servo system task commands --------------------------------------- //
// robot arm system tasks
#define ROBOT_ARM_IDLE            0
#define ROBOT_ARM_RUN             1
#define ROBOT_ARM_STOP            2
#define ROBOT_ARM_RECORD          3

// ---------------------------------- servo movement scaling values  ----------------------------------- //

// --> 1step pwm/degree = pwm range / degrees range = 3000/270 = 11.11
// servo speed in servo_run() main servo engine function
#define SERVO_PULSE_PER_DEGREE  11.11
#define SERVO_SLOW_SPEED        (SERVO_PULSE_PER_DEGREE*1)
#define SERVO_MEDIUM_SPEED      (SERVO_PULSE_PER_DEGREE*3)
#define SERVO_FAST_SPEED        (SERVO_PULSE_PER_DEGREE*5)


// servo speed in joint-mode degrees

// servo speed in cartesian-mode centimeters
#define ROBOT_ARM_CART_STEP_CM  1
#define CART_STEP_0_5CM         (ROBOT_ARM_CART_STEP_CM*0.5)
#define CART_STEP_1_0CM         (ROBOT_ARM_CART_STEP_CM*1)

// ---------------------------------------- jogging --------------------------------------------------- //
// JOGGING STEP RESOLUTION (Tune these to find the best visual movement)
// JOINT MODE: 1.0 to 5.0 degrees. 
// Try 2.0 or 3.0 first for a good balance of smoothness and speed.
float jogStepDeg = 3.0; 

// CARTESIAN MODE: 0.3 to 1.5 cm. 
// Try 0.5 or 1.0 first. (1.0 cm forces roughly 3-4 degrees of joint rotation)
float cartStep = 1.0;

// Which frame to jog: 0 = WCP (wrist center), 1 = TCP (tool tip)
uint8_t jogMode = 0;                // 0 = Joint space, 1 = Cartesian

// ----------------------------------- robot arm variables ------------------------------------------- //
const float L1 = 11.0;              // Base to shoulder (cm)
const float L2 = 10.5;              // Shoulder to elbow
const float L3 = 10.0;              // Elbow to wrist

// ----------------------------------- DYNAMIC TOOL MATH --------------------------------------------- //
const float L4_BASE = 5.5;          // Distance from wrist joint to the bare mounting plate
const float TOOL_LENGTH = 11.0;     // The actual length of the gripper

float L4 = L4_BASE + TOOL_LENGTH;   // Starts at 17.0, but can now be changed!
bool isToolAttached = true;         // System boots assuming the tool is on

// Tool orientation targets (in degrees, for jogging with fixed orientation modes)
float targetToolPitch = 0;          // Default: pointing down (-90°)
float targetToolRoll  = 0.0;        // Default: no roll

// Tracking Target Mode Variables
bool isTrackingTarget = false;      // State toggle for tracking mode
float trackObjX = 0.0;              // X coordinate of the target object
float trackObjY = 0.0;              // Y coordinate of the target object
float trackObjZ = 0.0;              // Z coordinate of the target object

// ----------------------------------- robot arm variables --------------------------------------------- //
//                                             Bs,   Sh,   Elb,  WrP,  WrR,  Gpr
int   SERVO_MIN_PWM_WORLD[ALL_SERVOS_COUNT] = {500,  500,  500,  500,  500,  2000};      
int   SERVO_MAX_PWM_WORLD[ALL_SERVOS_COUNT] = {3500, 3500, 3500, 3500, 3500, 2500};
int   home_position[ALL_SERVOS_COUNT]       = {2050, 1100, 1950, 2550, 2000, 2000};
float robot_arm_angles[5];
float angleOffset[5] = {0, 0, 0, 0, 0};

// ----------------------------------- robot arm debugging variables ----------------------------------- //
// used for debugging in calculateInverseKinematics
unsigned long lastErrorTime = 0;
// ----------------------------------------------------------------------------------------------------- //
// ----------------- gemini code for user/world frame work
// Tool Center Point (TCP): tip of gripper, Wrist Center Point (WCP): center of wrist / Index: 0 = X, 1 = Y, 2 = Z

float worldPosWcp[3]  = {0.0, 0.0, 0.0};          // world xyz of the wrist
float tempPosWcp[3]   = {0.0, 0.0, 0.0};          // temporary xyz of the wrist

float worldHandTcp[3] = {0.0, 0.0, 0.0};          // world xyz of the tool
float tempHandTcp[3]  = {0.0, 0.0, 0.0};          // temporary xyz of the tool

//float userPosWcp[3]   = {0.0, 0.0, 0.0};          // user  xyz of the wrist
//float userHandTcp[3]  = {0.0, 0.0, 0.0};          // user  xyz of the tool
//int rest_position[5]                      = {1250, 920, 1950, 2550, 2500};

// ------------------------------------ servo variables ------------------------------------------------- //
// using 6 servos for the robot arm
float         servoPosition[6];    // taken from main struct
float         servoDestination[6]; // taken from main struct
bool          lock[6];
bool          direction[6];
bool          update_robot_arm;
bool          update_display_numerics;
float         robot_speed;
uint8_t       robot_arm_task;
bool          robot_arm_power_on_reset;
// --------------------------------- objects instantiations --------------------------------------------------//
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); // PCA9685 I2C PWM driver

// ------------------------------------ servo functions -----------------------------------------------------//
// ---------------------------------- robot arm functions ---------------------------------------------------//
// functions declarations
//void system_initialize(void);
//void robot_arm_update_display(void);
//void robot_arm_update_movement(void);
//void servo_run(uint8_t servo_number, uint16_t target_destination);

#endif // robot_arm_constants_variables_h
