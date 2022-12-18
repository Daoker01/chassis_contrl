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
#elif defined MY_ENC_COUNTER
#include <I2C.h>
#define I2C_ADDR        0x34            //12C地址
#define ADC_BAT_ADDR                  0   //电压地址
#define MOTOR_TYPE_ADDR               20 //编码电机类型设置
#define MOTOR_ENCODER_POLARITY_ADDR   21 //设置编码方向极性，
//如果发现电机转速根本不受控制，要么最快速度转动，要么停止。可以将此地址的值重新设置一下
//范围0或1，默认0


#define MOTOR_ENCODER_TOTAL_ADDR  60 //4个编码电机各自的总脉冲值
//如果已知电机每转一圈的脉冲数为U，又已知轮子的直径D，那么就可以通过脉冲计数的方式得知每个轮子行进的距离
//比如读到电机1的脉冲总数为P，那么行进的距离为(P/U) * (3.14159*D)
//对于不同的电机可以自行测试每圈的脉冲数U，可以手动旋转10圈读出脉冲数，然后取平均值得出

u8 data[20];
//电机类型具体值

int32_t EncodeTotal[4]; //用于暂存电机累积转动量的值，正转递增，反转递减
//int32_t start_EncodeTotal[4];//构造用于储存开始时的脉冲数的数组
int serial_putc( char c, struct __file * );
void printf_begin(void);
#endif

long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();

