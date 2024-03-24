// the uart used to control servos.
// GPIO 18 - S_RXD, GPIO 19 - S_TXD, as default.
#define RoArmM2_Servo_RXD 18
#define RoArmM2_Servo_TXD 19

// 2: flow feedback.
// 1: [default]print debug info in serial.
// 0: don't print debug info in serial.
byte InfoPrint = 1;

// devices info:
// espNowMode: 0 - none
//             1 - flow-leader(group): sending cmds
//             2 - flow-leader(single): sending cmds to a single follower
//             3 - [default]follower: recv cmds
byte espNowMode = 3;

// set the broadcast ctrl mode.
// broadcast mac address: FF:FF:FF:FF:FF:FF.
// true  - [default]it can be controled by broadcast mac address.
// false - it won't be controled by broadcast mac address.
bool ctrlByBroadcast = true;

// you can define some whitelist mac addresses here.
uint8_t mac_whitelist_broadcast[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Multifunction End-Effector Switching System.
// 0 - end servo as grab.
// 1 - end servo as a joint moving in vertical plane.
byte EEMode = 0;

// run new json cmd
bool runNewJsonCmd = false;

// 1: WAVE ROVER
// 2: UGV02(UGV)
// 3: UGV01(UGV)
byte mainType = 1;

// 0: [Base default] without RoArm-M2 and gimbal.
// 1: [RoArm default] RoArm-M2 mounted on the UGV.
// 2: [Gimbal default] Gimbal mounted on the UGV.
byte moduleType = 0;

// false: gimbal steady mode off.
//  true: gimbal steady mode on.
bool steadyMode = false;

// 0: turn off base info feedback flow.
// 1: [default] turn on base info feedback flow.
bool baseFeedbackFlow = 0;

String thisMacStr;

#define BASE_JOINT     1
#define SHOULDER_JOINT 2
#define ELBOW_JOINT    3
#define EOAT_JOINT     4

// define servoID
//   |---[14]---|
//   ||  |  |  ||
//   ||        ||
//   ||  |  |  ||
//   ||        ||
//   ||  |  |  ||
//   || -[15]- ||
//   ||        ||
//   ||[13][12]||
//     |  __  |
//       [11]
#define BASE_SERVO_ID    11
#define SHOULDER_DRIVING_SERVO_ID 12
#define SHOULDER_DRIVEN_SERVO_ID  13
#define ELBOW_SERVO_ID   14
#define GRIPPER_SERVO_ID 15

#define ARM_SERVO_MIDDLE_POS  2047
#define ARM_SERVO_MIDDLE_ANGLE 180
#define ARM_SERVO_POS_RANGE   4096
#define ARM_SERVO_ANGLE_RANGE  360
#define ARM_SERVO_INIT_SPEED   600
#define ARM_SERVO_INIT_ACC      20

#define ARM_L1_LENGTH_MM    126.06
#define ARM_L2_LENGTH_MM_A  236.82
#define ARM_L2_LENGTH_MM_B	30.00 
#define ARM_L3_LENGTH_MM_A_0	280.15
#define ARM_L3_LENGTH_MM_B_0	1.73

// 	  TYPE:0
//    -------L3A-----------O==L2B===
//    |                    ^       ||
//   L3B                   |       ||
//    |              ELBOW_JOINT   ||
//                                L2A
//                                 ||
//                                 ||
//                                 ||
//               SHOULDER_JOINT -> OO
//                                [||]
//                                 L1
//                                [||]
//                   BASE_JOINT -> X
double l1  = ARM_L1_LENGTH_MM;
double l2A = ARM_L2_LENGTH_MM_A;
double l2B = ARM_L2_LENGTH_MM_B;
double l2  = sqrt(l2A * l2A + l2B * l2B);
double t2rad = atan2(l2B, l2A);
double l3A = ARM_L3_LENGTH_MM_A_0;
double l3B = ARM_L3_LENGTH_MM_B_0;
double l3  = sqrt(l3A * l3A + l3B * l3B);
double t3rad = atan2(l3B, l3A);


#define ARM_L3_LENGTH_MM_A_1	215.99
#define ARM_L3_LENGTH_MM_B_1	0

// edge
double ARM_L4_LENGTH_MM_A =	67.85;

// D-3.2
// double ARM_L4_LENGTH_MM_A =	64.16;

// D-4.2
// double ARM_L4_LENGTH_MM_A =	59.07;

// D-10.2
// double ARM_L4_LENGTH_MM_A =	51.07;

#define ARM_L4_LENGTH_MM_B  5.98

//    TYPE:1
//                   -------L3A-----------O==L2B===
//                   |                    ^       ||
//                  L3B                   |       ||
//                   |              ELBOW_JOINT   ||
//          ---L4A---O                           L2A
//          |                                     ||
//     |   L4B                                    ||
//   /      |                                     ||
// 180°X-EA-X                   SHOULDER_JOINT -> OO
//   \ |                                         [||]
//    EB                                          L1
//     |                                         [||]
//    --------                      BASE_JOINT -> XX

// 		\  T:210°
// 		 \
//  	  EB
//   	   \
// 		-----------

double EoAT_A = 0;
double EoAT_B = 0;
double l4A = ARM_L4_LENGTH_MM_A;
double l4B = ARM_L4_LENGTH_MM_B;
double lEA = EoAT_A + ARM_L4_LENGTH_MM_A;
double lEB = EoAT_B + ARM_L4_LENGTH_MM_B;
double lE  = sqrt(lEA * lEA + lEB * lEB);
double tErad = atan2(lEB, lEA);


double initX = l3A+l2B; //
double initY = 0;
double initZ = l2A-l3B;
double initT = M_PI;

double goalX = initX;
double goalY = initY;
double goalZ = initZ;
double goalT = initT;

double lastX = goalX;
double lastY = goalY;
double lastZ = goalZ;
double lastT = goalT;

double base_r;

double delta_x;
double delta_y;

double beta_x;
double beta_y;

double radB;
double radS;
double radE;
double radG;

#define MAX_SERVO_ID 32 // MAX:253

// the uart used to control servos.
// GPIO 18 - S_RXD, GPIO 19 - S_TXD, as default.
#define S_RXD 18
#define S_TXD 19

double BASE_JOINT_RAD = 0;
double SHOULDER_JOINT_RAD = 0;
double ELBOW_JOINT_RAD = M_PI/2;
double EOAT_JOINT_RAD = M_PI;
double EOAT_JOINT_RAD_BUFFER;

double BASE_JOINT_ANG  = 0;
double SHOULDER_JOINT_ANG = 0;
double ELBOW_JOINT_ANG = 90.0;
double EOAT_JOINT_ANG  = 180.0;

// true: torqueLock ON, servo produces torque.
// false: torqueLock OFF, servo release torque.
bool RoArmM2_torqueLock = true;
bool emergencyStopFlag = false;
bool newCmdReceived = false;

bool nanIK;

bool RoArmM2_initCheckSucceed  = false;
// bool RoArmM2_initCheckSucceed   = true;

// // // args for syncWritePos.
u8  servoID[5] = {11, 12, 13, 14, 15};
s16 goalPos[5] = {2047, 2047, 2047, 2047, 2047};
u16 moveSpd[5] = {0, 0, 0, 0, 0};
u8  moveAcc[5] = {ARM_SERVO_INIT_ACC,
			      ARM_SERVO_INIT_ACC,
			      ARM_SERVO_INIT_ACC,
			      ARM_SERVO_INIT_ACC,
			      ARM_SERVO_INIT_ACC};


double ARM_BASE_LIMIT_MIN_RAD     = -M_PI/2;
double ARM_BASE_LIMIT_MAX_RAD     =  M_PI/2;

double ARM_SHOULDER_LIMIT_MIN_RAD = -M_PI/2;
double ARM_SHOULDER_LIMIT_MAX_RAD =  M_PI/2;

double ARM_ELBOW_LIMIT_MIN_RAD    = -M_PI/2;
double ARM_ELBOW_LIMIT_MAX_RAD    =  M_PI/2;

double ARM_GRIPPER_LIMIT_MIN_RAD  = -M_PI/2;
double ARM_GRIPPER_LIMIT_MAX_RAD  =  M_PI/2;


// --- --- --- Pneumatic Components && Lights --- --- ---

const uint16_t ANALOG_WRITE_BITS = 8;
const uint16_t MAX_PWM = pow(2, ANALOG_WRITE_BITS)-1;
const uint16_t MIN_PWM = MAX_PWM/4;

#define PWMA 25         // Motor A PWM control  
#define AIN2 17         // Motor A input 2     
#define AIN1 21         // Motor A input 1     
#define BIN1 22         // Motor B input 1       
#define BIN2 23         // Motor B input 2       
#define PWMB 26         // Motor B PWM control  

#define AENCA 35        // Encoder A input      
#define AENCB 34

#define BENCB 16        // Encoder B input     
#define BENCA 27

int freq = 100000;
int channel_A = 5;
int channel_B = 6;


// --- --- --- Bus Servo Settings --- --- ---

#define ST_PID_P_ADDR 21
#define ST_PID_D_ADDR 22
#define ST_PID_I_ADDR 23

#define ST_PID_ROARM_P   16
#define ST_PID_DEFAULT_P 32

#define ST_TORQUE_MAX 1000
#define ST_TORQUE_MIN 50


// --- --- --- i2c Settings --- --- ---

#define S_SCL   33
#define S_SDA   32


//  --- --- --- web / constant moving --- --- ---

#define MOVE_STOP 0
#define MOVE_INCREASE 1
#define MOVE_DECREASE 2

#define CONST_ANGLE 0
#define CONST_XYZT  1

float const_spd;
byte  const_mode;

byte const_cmd_base_x;
byte const_cmd_shoulder_y;
byte const_cmd_elbow_z;
byte const_cmd_eoat_t;

float const_goal_base = BASE_JOINT_ANG;
float const_goal_shoulder = SHOULDER_JOINT_ANG;
float const_goal_elbow = ELBOW_JOINT_ANG;
float const_goal_eoat = EOAT_JOINT_ANG;

unsigned long prev_time = 0;

String jsonFeedbackWeb = "";


//  --- --- --- pid controller --- --- ---

float __kp = 20.0;
float __ki = 2000.0;
float __kd = 0;
float windup_limits = 255;


//  --- --- --- ugv base --- --- ---

#define THRESHOLD_PWM 23

// mainType:01 RaspRover
// #define WHEEL_D 0.0800
// #define ONE_CIRCLE_PLUSES	2100
// #define TRACK_WIDTH	0.125
// #define SET_MOTOR_DIR false

// mainType:02 UGV Rover
// #define WHEEL_D 0.0800
// #define ONE_CIRCLE_PLUSES	1650
// #define TRACK_WIDTH	0.172
// #define SET_MOTOR_DIR false

// mainType:03 UGV Beast
// #define WHEEL_D	0.0523
// #define ONE_CIRCLE_PLUSES	1092
// #define TRACK_WIDTH	0.141
// #define SET_MOTOR_DIR true

double WHEEL_D = 0.0800;
int ONE_CIRCLE_PLUSES = 1650;
double TRACK_WIDTH = 0.172;
bool SET_MOTOR_DIR = false;


#define IO4_PIN 4
#define IO5_PIN 5

int IO4_CH = 7;
int IO5_CH = 8;

const uint16_t FREQ = 200;

int feedbackFlowExtraDelay = 0;
bool uartCmdEcho = 1;

#define GIMBAL_PAN_ID  2
#define GIMBAL_TILT_ID 1

#define SERVO_STOP_DELAY 3

int HEART_BEAT_DELAY = 3000;
unsigned long lastCmdRecvTime = millis();


// --- --- --- ugv imu --- --- ---
double icm_pitch, icm_roll, icm_yaw, icm_temp;
unsigned long last_imu_update = 0;

double qw, qx, qy, qz;
double ax, ay, az;
double mx, my, mz;
double gx, gy, gz;