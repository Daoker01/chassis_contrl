/*********************************************************************
 *  ROSArduinoBridge
 
    A set of simple serial commands to control a differential drive
    robot and receive back sensor and odometry data. Default 
    configuration assumes use of an Arduino Mega + Pololu motor
    controller shield + Robogaia Mega Encoder shield.  Edit the
    readEncoder() and setMotorSpeed() wrapper functions if using 
    different motor controller or encoder method.

    Created for the Pi Robot Project: http://www.pirobot.org
    and the Home Brew Robotics Club (HBRC): http://hbrobotics.org
    
    Authors: Patrick Goebel, James Nugen

    Inspired and modeled after the ArbotiX driver by Michael Ferguson
    
    Software License Agreement (BSD License)

    Copyright (c) 2012, Patrick Goebel.
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above
       copyright notice, this list of conditions and the following
       disclaimer in the documentation and/or other materials provided
       with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
//使用基控制器（默认不启用）
//#define USE_BASE      // Enable the base controller code
#undef USE_BASE     // Disable the base controller code

/* Define the motor controller and encoder library you are using */
//编码器和电机驱动相关实现，后期需要修改
#ifdef USE_BASE
//  #include "chassis_drive.h"
  //自定义编码器驱动
//  #define CHASSIS_DRIVER
//  #define L298P_MOTOR_DRIVER
  #define L298_MOTOR_DRIVER
#endif

#include "chassis_drive.h" //使用自己的控制驱动

//是否使用舵机
//#define USE_SERVOS  // Enable use of PWM servos as defined in servos.h
#undef USE_SERVOS     // Disable use of PWM servos

/* Serial port baud rate */
#define BAUDRATE     57600

/* Maximum PWM signal */
#define MAX_PWM        255

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/* Include definition of serial commands */
//串口通信命令
#include "commands.h"

/* Sensor functions */
#include "sensors.h"

/* Include servo support if required */
#ifdef USE_SERVOS
   #include <Servo.h>
   #include "servos.h"
#endif
/*********************************************************************************************************/
#ifdef USE_BASE
  /* Motor driver function definitions */
  #include "motor_driver.h"//电机驱动

  /* Encoder driver function definitions */
  #include "encoder_driver.h"//编码器驱动

  /* PID parameters and functions */
  #include "diff_controller.h"//pid控制实现

  /* Run the PID loop at 30 times per second */
  #define PID_RATE           30     // Hz

  /* Convert the rate into an interval */
  const int PID_INTERVAL = 1000 / PID_RATE;
  
  /* Track the next time we make a PID calculation */
  unsigned long nextPID = PID_INTERVAL;

  /* Stop the robot if it hasn't received a movement command
   in this number of milliseconds */
  #define AUTO_STOP_INTERVAL 2000  //电机接收到速度指令后的运行时间
  long lastMotorCommand = AUTO_STOP_INTERVAL;
#endif
/******************************************************************************************************************/
/* Variable initialization 变量初始化*/

// A pair of varibles to help parse serial commands (thanks Fergs)一对帮助解析串行命令的变量(感谢Fergs)
int arg = 0;
int index = 0;

// Variable to hold an input character变量来保存输入字符
char chr;

// Variable to hold the current single-character command变量保存当前单字符命令
char cmd;

// Character arrays to hold the first and second arguments用于保存第一个和第二个参数的字符数组
char argv1[16];
char argv2[16];
char argv3[16];
char argv4[16];

// The arguments converted to integers 参数转换为整数
long arg1;
long arg2;
long arg3;
long arg4;

/* Clear the current command parameters *//*清除当前命令参数*/
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));//初始化内存
  memset(argv2, 0, sizeof(argv2));
  memset(argv3, 0, sizeof(argv3));
  memset(argv4, 0, sizeof(argv4));
  arg1 = 0;
  arg2 = 0;
  arg3 = 0;
  arg4 = 0;
  arg = 0;
  index = 0;
}

/* Run a command.  Commands are defined in commands.h */
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);
  arg3 = atoi(argv3);
  arg4 = atoi(argv4);
  
  switch(cmd) {
  case GET_BAUDRATE://获取波特率
    Serial.println(BAUDRATE);
    break;
  case ANALOG_READ://模拟读
    Serial.println(analogRead(arg1));
    break;
  case DIGITAL_READ://数字读
    Serial.println(digitalRead(arg1));
    break;
  case ANALOG_WRITE://模拟写
    analogWrite(arg1, arg2);
    Serial.println("OK"); 
    break;
  case DIGITAL_WRITE://数字写
    if (arg2 == 0) digitalWrite(arg1, LOW);
    else if (arg2 == 1) digitalWrite(arg1, HIGH);
    Serial.println("OK"); 
    break;
  case PIN_MODE://端口模式
    if (arg2 == 0) pinMode(arg1, INPUT);
    else if (arg2 == 1) pinMode(arg1, OUTPUT);
    Serial.println("OK");
    break;
  case PING:
    Serial.println(Ping(arg1));
    break;
  case READ_ENCODERS:
   Serial.println("读取编码器计数");
    int32_t read_EncodeTotal[4];
    WireReadDataArray(MOTOR_ENCODER_TOTAL_ADDR,(uint8_t*)read_EncodeTotal,16);
     Serial.print(read_EncodeTotal[0]);Serial.print("\t");Serial.print(read_EncodeTotal[1]);Serial.print("\t");Serial.print(read_EncodeTotal[2]);Serial.print("\t");Serial.print(read_EncodeTotal[3]);Serial.println("\t");
   break;
   case RESET_ENCODERS:
    Serial.println("重置编码器计数");
    rest_encode();
    Serial.println("OK");
    break;
    case MOTOR_SPEEDS:
    /* Reset the auto stop timer */
    Serial.println("读取各个轮子速度，单位：m/s");
    get_current_vel();
    break;
    case SETUP_SPEEDS://设置各轮子速度
    int16_t setup_speeds[4];
    setup_speeds[0]=arg1;setup_speeds[1]=arg2;setup_speeds[2]=arg3;setup_speeds[3]=arg4;
    //   Serial.println("输入的目标速度（单位mm/s）为：");
   //  Serial.print(setup_speeds[0]);Serial.print("\t");Serial.print(setup_speeds[1]);Serial.print("\t");Serial.print(setup_speeds[2]);Serial.print("\t");Serial.print(setup_speeds[3]);Serial.println("\t");
  //   Serial.print(arg1);Serial.print("\t");Serial.print(arg2);Serial.print("\t");Serial.print(arg3);Serial.print("\t");Serial.print(arg4);Serial.println("\t");
    contrl_vel(setup_speeds);
    break;
    
#ifdef USE_SERVOS
  case SERVO_WRITE:
    servos[arg1].setTargetPosition(arg2);
    Serial.println("OK");
    break;
  case SERVO_READ:
    Serial.println(servos[arg1].getServo().read());
    break;
#endif
    
#ifdef USE_BASE
  case READ_ENCODERS:
    Serial.print(readEncoder(LEFT));
    Serial.print(" ");
    Serial.println(readEncoder(RIGHT));
    break;
   case RESET_ENCODERS:
    resetEncoders();
    resetPID();
    Serial.println("OK");
    break;
  case MOTOR_SPEEDS:
    /* Reset the auto stop timer */
    lastMotorCommand = millis();
    if (arg1 == 0 && arg2 == 0) {
      setMotorSpeeds(0, 0);
      resetPID();
      moving = 0;
    }
    else moving = 1;
    leftPID.TargetTicksPerFrame = arg1;
    rightPID.TargetTicksPerFrame = arg2;
    Serial.println("OK"); 
    break;
  case UPDATE_PID:
    while ((str = strtok_r(p, ":", &p)) != '\0') {
       pid_args[i] = atoi(str);
       i++;
    }
    Kp = pid_args[0];
    Kd = pid_args[1];
    Ki = pid_args[2];
    Ko = pid_args[3];
    Serial.println("OK");
    break;
#endif
  default:
    Serial.println("Invalid Command");
    break;
  }
}

/* Setup function--runs once at startup. */

void setup() {
  Serial.begin(BAUDRATE);

// Initialize the motor controller if used */
#ifdef USE_BASE
  #ifdef ARDUINO_ENC_COUNTER
    //set as inputs
    DDRD &= ~(1<<LEFT_ENC_PIN_A);
    DDRD &= ~(1<<LEFT_ENC_PIN_B);
    DDRC &= ~(1<<RIGHT_ENC_PIN_A);
    DDRC &= ~(1<<RIGHT_ENC_PIN_B);
    
    //enable pull up resistors
    PORTD |= (1<<LEFT_ENC_PIN_A);
    PORTD |= (1<<LEFT_ENC_PIN_B);
    PORTC |= (1<<RIGHT_ENC_PIN_A);
    PORTC |= (1<<RIGHT_ENC_PIN_B);
    
    // tell pin change mask to listen to left encoder pins
    PCMSK2 |= (1 << LEFT_ENC_PIN_A)|(1 << LEFT_ENC_PIN_B);
    // tell pin change mask to listen to right encoder pins
    PCMSK1 |= (1 << RIGHT_ENC_PIN_A)|(1 << RIGHT_ENC_PIN_B);
    
    // enable PCINT1 and PCINT2 interrupt in the general interrupt mask
    PCICR |= (1 << PCIE1) | (1 << PCIE2);
    #elif defined CHASSIS_DRIVER
   
  #endif
  initMotorController();
  resetPID();
#endif

/* Attach servos if used */
  #ifdef USE_SERVOS
    int i;
    for (i = 0; i < N_SERVOS; i++) {
      servos[i].initServo(
          servoPins[i],
          stepDelay[i],
          servoInitPosition[i]);
    }
  #endif

   /*****************************************底盘驱动中的setup*****************************************/
  Wire.begin();//初始化 I2C，以 Ardunio UNO 为例 I2C 口为：A4(SCL)、A5(CLK)
  //Serial.begin(57600);
  printf_begin();
  delay(200);
  WireWriteDataArray(MOTOR_TYPE_ADDR,&MotorType,1);//在电机类型地址中写入电机类型编号
 //向地址中写入数据 （reg：地址 val：数据内容 len：数据长度）
 
  delay(5);
  WireWriteDataArray(MOTOR_ENCODER_POLARITY_ADDR,&MotorEncoderPolarity,1);//向编码方向极性地址中写入MotorEncoderPolarity地址，长度为1
  WireWriteDataArray(MOTOR_FIXED_SPEED_ADDR,result,4);//开机时保持停止

  //复位后清零，调试用
 rest_encode();

  WireReadDataArray(MOTOR_ENCODER_TOTAL_ADDR,(uint8_t*)start_EncodeTotal,16);//获取开始时的转动量
  /*****************************************底盘驱动中的setup*****************************************/
}

/* Enter the main loop.  Read and parse input from the serial port
   and run any valid commands. Run a PID calculation at the target
   interval and check for auto-stop conditions.
*/

/*用数组传递电机速度，正数为设置前进速度，负数为设置后退速度
以 p1、p2 为例：p1=4 个电机以 50 的速度前进 p2=4 个电机以 20 的速度后退*/
int8_t p1[4]={50,50,50,50};
int8_t p2[4]={-50,-50,-50,-50};
int8_t s1[4]={0,0,0,6};
int8_t s2[4]={-2,-2,-2,-2};
//int8_t s1[4]={2,20,10,30};
//int8_t s2[4]={-20,-20,-30,-9};
static u16 v=0;  //用于暂存电压值
u8 data[20];

void loop() {
// while(1){
//    WireReadDataArray(ADC_BAT_ADDR,data,2);   //读取电压地址中存放的电压
//  v = data[0]+ (data[1]<<8); //将电压转换为 mV
// Serial.print("V = ");Serial.print(v);Serial.println("mV     "); //打印电压的值
// WireReadDataArray(MOTOR_ENCODER_TOTAL_ADDR,(uint8_t*)EncodeTotal,16);  //读取电机累积转动量
//  printf("Encode1 = %ld  Encode2 = %ld  Encode3 = %ld  Encode4 = %ld  \r\n", EncodeTotal[0], EncodeTotal[1], EncodeTotal[2], EncodeTotal[3]);
//  delay(2000);
// }
  while (Serial.available() > 0) {

//打印四个电机的累积转动量
//  Serial.println(millis());
//  get_current_vel();//测速并打印
//  Serial.println(millis());
//  delay(2000);
    
    // Read the next character读下一个字符
    chr = Serial.read();

    // Terminate a command with a CR使用CR终止命令
    if (chr == 13) {//按下回车
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
       else if (arg == 3) argv3[index] = NULL;
        else if (arg == 4) argv4[index] = NULL;
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      }
          else if (arg == 2)  {
        argv2[index] = NULL;
        arg = 3;
        index = 0;
      }
               else if (arg == 3)  {
        argv3[index] = NULL;
        arg = 4;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
       else if (arg == 3) {
        argv3[index] = chr;
        index++;
      }
       else if (arg == 4) {
        argv4[index] = chr;
        index++;
      }
    }
  }
  
// If we are using base control, run a PID calculation at the appropriate intervals
#ifdef USE_BASE
  if (millis() > nextPID) {
    updatePID();
    nextPID += PID_INTERVAL;
  }
  
  // Check to see if we have exceeded the auto-stop interval
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {;
    setMotorSpeeds(0, 0);
    moving = 0;
  }
#endif

// Sweep servos
#ifdef USE_SERVOS
  int i;
  for (i = 0; i < N_SERVOS; i++) {
    servos[i].doSweep();
  }
#endif

}

