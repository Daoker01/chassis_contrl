/* *************************************************************
   Encoder driver function definitions - by James Nugen
   ************************************************************ */
   
   
#ifdef ARDUINO_ENC_COUNTER
  //below can be changed, but should be PORTD pins; 
  //otherwise additional changes in the code are required
  #define LEFT_ENC_PIN_A PD2  //pin 2
  #define LEFT_ENC_PIN_B PD3  //pin 3
  
  //below can be changed, but should be PORTC pins
  #define RIGHT_ENC_PIN_A PC4  //pin A4
  #define RIGHT_ENC_PIN_B PC5   //pin A5
#elif defined CHASSIS_DRIVER
   #include <Wire.h>
#define I2C_ADDR        0x34            //12C地址
#define ADC_BAT_ADDR                  0   //电压地址
#define MOTOR_TYPE_ADDR               20 //编码电机类型设置
#define MOTOR_ENCODER_POLARITY_ADDR   21 //设置编码方向极性，
//如果发现电机转速根本不受控制，要么最快速度转动，要么停止。可以将此地址的值重新设置一下
//范围0或1，默认0

#define MOTOR_FIXED_PWM_ADDR      31 //固定PWM控制，属于开环控制,范围（-100~100）
//#define SERVOS_ADDR_CMD 40        
#define MOTOR_FIXED_SPEED_ADDR    51 //固定转速控制，属于闭环控制，
//单位：脉冲数每10毫秒，范围（根据具体的编码电机来，受编码线数，电压大小，负载大小等影响，一般在±50左右）
#define MOTOR_ENCODER_TOTAL_ADDR  60 //4个编码电机各自的总脉冲值
//如果已知电机每转一圈的脉冲数为U，又已知轮子的直径D，那么就可以通过脉冲计数的方式得知每个轮子行进的距离
//比如读到电机1的脉冲总数为P，那么行进的距离为(P/U) * (3.14159*D)
//对于不同的电机可以自行测试每圈的脉冲数U，可以手动旋转10圈读出脉冲数，然后取平均值得出

//电机类型具体值
#define MOTOR_TYPE_WITHOUT_ENCODER        0  //无编码器的电机
#define MOTOR_TYPE_TT                     1  //TT编码电机
#define MOTOR_TYPE_N20                    2  //n20编码电机
#define MOTOR_TYPE_JGB37_520_12V_110RPM   3 //磁环每转是44个脉冲   减速比:90  默认
bool WireWriteByte(uint8_t val);                                             //检测驱动板I2C是否处于可操作状态
bool WireWriteDataArray(  uint8_t reg,uint8_t *val,unsigned int len);       //向地址中写入数据 （reg：地址 val：数据内容 len：数据长度）
bool WireReadDataByte(uint8_t reg, uint8_t &val);                           //读取地址中的数据（reg：地址 val：数据内容）
int WireReadDataArray(   uint8_t reg, uint8_t *val, unsigned int len);      //读取地址中指定长度的数据（reg：地址 val：数据内容 len：数据长度）
int serial_putc( char c, struct __file * );
void printf_begin(void);
void get_current_vel();                                                     //以m/s为单位输出当前小车四个轮子的速度
void rest_encode();                                                         //编码器计数清零

uint8_t MotorType = MOTOR_TYPE_TT;  //设定电机类型
uint8_t MotorEncoderPolarity = 1; 
int8_t result[4]={0,0,0,0};
int32_t start_EncodeTotal[4];//构造用于储存开始时的脉冲数的数组
int32_t current_EncodeTotal[4];//构造用于储存当前脉冲数的数组

#endif
   
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();

