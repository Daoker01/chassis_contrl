/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

#ifdef L298_MOTOR_DRIVER
#define RIGHT_MOTOR_BACKWARD 5
#define LEFT_MOTOR_BACKWARD  6
#define RIGHT_MOTOR_FORWARD  9
#define LEFT_MOTOR_FORWARD   10
#define RIGHT_MOTOR_ENABLE 12
#define LEFT_MOTOR_ENABLE 13
#elif defined MY_MOTOR_DRIVER
#define MOTOR_FIXED_PWM_ADDR      31 //固定PWM控制，属于开环控制,范围（-100~100）
//#define SERVOS_ADDR_CMD 40
#define MOTOR_FIXED_SPEED_ADDR    51 //固定转速控制，属于闭环控制，
//单位：脉冲数每10毫秒，范围（根据具体的编码电机来，受编码线数，电压大小，负载大小等影响，一般在±50左右）
#define MOTOR_TYPE_WITHOUT_ENCODER        0  //无编码器的电机
#define MOTOR_TYPE_TT                     1  //TT编码电机
#define MOTOR_TYPE_N20                    2  //n20编码电机
#define MOTOR_TYPE_JGB37_520_12V_110RPM   3 //磁环每转是44个脉冲   减速比:90  默认
uint8_t MotorType = MOTOR_TYPE_TT;  //设定电机类型
uint8_t MotorEncoderPolarity = 1;
int8_t result[4] = {0, 0, 0, 0};
static uint8_t save_setup_speeds[4]={0,0,0,0};
#endif

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
