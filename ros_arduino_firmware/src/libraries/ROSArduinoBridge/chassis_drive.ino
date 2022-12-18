//#include <Wire.h>
#include "chassis_drive.h"

bool WireWriteByte(uint8_t val)
{
    Wire.beginTransmission(I2C_ADDR);//开始一次数据传输，发送一个I2C开始字符0x34
    Wire.write(val);//像从机发送数据
    if( Wire.endTransmission() != 0 ) { //返回值为0才成功，否则失败
        return false;
    }
    return true;
}
//向地址中写入数据 （reg：地址 val：数据内容 len：数据长度）
bool WireWriteDataArray(  uint8_t reg,uint8_t *val,unsigned int len)
{
    unsigned int i;

    Wire.beginTransmission(I2C_ADDR);
    Wire.write(reg);//发送地址数据
    for(i = 0; i < len; i++) {
        Wire.write(val[i]);//发送内容
    }
    if( Wire.endTransmission() != 0 ) {//结果判断
        return false;
    }

    return true;
}
//读取地址中的数据（reg：地址 val：数据内容）
bool WireReadDataByte(uint8_t reg, uint8_t &val)
{
    if (!WireWriteByte(reg)) {//如果WireWriteByte（）非真
        return false;
    }
    
    Wire.requestFrom(I2C_ADDR, 1);//主设备请求从设备I2C_ADDR地址处的一个字节，这个字节可以被主设备用 read()或available()接受
    while (Wire.available()) //接收获取的字节
    {
        val = Wire.read();//Wire.requestFrom()请求从机数据后，可以使用read接收
    }
    
    return true;
}
//读取地址中指定长度的数据（reg：地址 val：数据内容 len：数据长度）,最后返回值i要等于len才正常
int WireReadDataArray(   uint8_t reg, uint8_t *val, unsigned int len)
{
    unsigned char i = 0;
    
    /* Indicate which register we want to read from */
    if (!WireWriteByte(reg)) {
        return -1;
    }
    
    /* Read block data */
    Wire.requestFrom(I2C_ADDR, len);
    while (Wire.available()) {
        if (i >= len) {
            return -1;
        }
        val[i] = Wire.read();
        i++;
    }
    
    return i;
}


int serial_putc( char c, struct __file * )
{
  Serial.write( c );//Serial.print() 发送的是字符，Serial.write() 发送的字节.
  return c;
}
void printf_begin(void)
{
  fdevopen( &serial_putc, 0 );//将函数serial_putc指向串口
}




//uint8_t MotorType = MOTOR_TYPE_TT;  //设定电机类型
//uint8_t MotorEncoderPolarity = 1; 
//int8_t result[4]={0,0,0,0};
//int32_t EncodeTotal[4]; //用于暂存电机累积转动量的值，正转递增，反转递减

/*测速函数*/
long start_time = millis();//一个计算周期的开始时刻，初始值为 millis();
long interval_time = 50;//一个计算周期 50ms
 double current_vel[4];
//int32_t start_EncodeTotal[4];//构造用于储存开始时的脉冲数的数组
//int32_t current_EncodeTotal[4];//构造用于储存当前脉冲数的数组
void get_current_vel()
{
  long right_now = millis();  
  long past_time = right_now - start_time;//计算逝去的时间
   
  if(past_time >= interval_time){//如果逝去时间大于等于一个计算周期
    //1.禁止中断
   // noInterrupts();
    //2.计算转速 转速单位可以是秒，也可以是分钟... 自定义即可
    WireReadDataArray(MOTOR_ENCODER_TOTAL_ADDR,(uint8_t*)current_EncodeTotal,16);//获取当前的转动量
    for(int i=0;i<4;i++)
    {
      current_vel[i]=(current_EncodeTotal[i]-start_EncodeTotal[i])*0.3625/past_time*1000;//求当前各个轮子的速度，单位mm/s
     
    }
    //3.重置计数器
        for(int i=0;i<4;i++)
    {
      start_EncodeTotal[i]=current_EncodeTotal[i];
    }
    //4.重置开始时间
    start_time = right_now;
    //5.重启中断
   // interrupts();

     Serial.println("四个轮子的速度(单位mm/s)分别为：");
     Serial.print(current_vel[0]);Serial.print("\t");Serial.print(current_vel[1]);Serial.print("\t");Serial.print(current_vel[2]);Serial.print("\t");Serial.print(current_vel[3]);Serial.println("\t");

  }
}
void rest_encode()//编码器计数清零
{
   EncodeTotal[0]=0,EncodeTotal[1]=0,EncodeTotal[2]=0,EncodeTotal[3]=0;//数组赋值0
  WireWriteDataArray(MOTOR_ENCODER_TOTAL_ADDR,(uint8_t*)EncodeTotal,16);//把驱动板上的编码器计数器归0
  start_EncodeTotal[0]=0;start_EncodeTotal[1]=0;start_EncodeTotal[2]=0;start_EncodeTotal[3]=0;//避免清0后第一次测速数据异常
}
void contrl_vel(int16_t *val)                                             //指定各轮的速度，单位：m/s
{//要把速度转化成脉冲数每10ms,result=val*10/0.3625
  int8_t vel2pulse[4]={0,0,0,0};
  for(int i=0;i<4;i++){
    //vel2pulse[i]=val[i]/100/0.3625;//这里会先把val[i]/100取整型再除以0.3625
    vel2pulse[i]=val[i]/36.25;
  }
    WireWriteDataArray(MOTOR_FIXED_SPEED_ADDR,vel2pulse,4);//由于输入的数据只能为整型，控制精度低，后期考虑自己写pid,用pwm
      Serial.println("四个轮子的目标速度（单位mm/s）为：");
     Serial.print(val[0]);Serial.print("\t");Serial.print(val[1]);Serial.print("\t");Serial.print(val[2]);Serial.print("\t");Serial.print(val[3]);Serial.println("\t");
      Serial.println("四个轮子的目标脉冲(单位脉冲/10ms)为：");
     Serial.print(vel2pulse[0]);Serial.print("\t");Serial.print(vel2pulse[1]);Serial.print("\t");Serial.print(vel2pulse[2]);Serial.print("\t");Serial.print(vel2pulse[3]);Serial.println("\t");
     int8_t read_pwm[4]={0,0,0,0};
     int8_t read_pid[4]={0,0,0,0};
    
     for(int i=0;i<4;i++)
     {
     delay(2000);
     get_current_vel();
      while(4 !=WireReadDataArray(MOTOR_FIXED_PWM_ADDR,(uint8_t*)read_pwm,4));
     Serial.println("四个轮子的PWM为：");
     Serial.print(read_pwm[0]);Serial.print("\t");Serial.print(read_pwm[1]);Serial.print("\t");Serial.print(read_pwm[2]);Serial.print("\t");Serial.print(read_pwm[3]);Serial.println("\t");
      while(4 !=WireReadDataArray(MOTOR_FIXED_SPEED_ADDR,(uint8_t*)read_pid,4));
     Serial.println("四个轮子的pid为：");
     Serial.print(read_pid[0]);Serial.print("\t");Serial.print(read_pid[1]);Serial.print("\t");Serial.print(read_pid[2]);Serial.print("\t");Serial.print(read_pid[3]);Serial.println("\t");
     }
}


