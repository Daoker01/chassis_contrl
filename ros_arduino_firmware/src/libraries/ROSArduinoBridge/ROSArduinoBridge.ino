

/*********************************************************************
    ROSArduinoBridge
  m 30
    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

       Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
       Redistributions in binary form must reproduce the above
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
    POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
//鏄惁浣跨敤鍩烘帶鍒跺櫒锛屽悗鏈熼渶瑕佷娇鐢�
#define USE_BASE      // Enable the base controller code
//#undef USE_BASE     // Disable the base controller code

/* Define the motor controller and encoder library you are using */
//缂栫爜鍣ㄥ拰鐢垫満椹卞姩鐩稿叧锛屽悗鏈熷惎鐢�
#ifdef USE_BASE
/* The Pololu VNH5019 dual motor driver shield */
//#define POLOLU_VNH5019

/* The Pololu MC33926 dual motor driver shield */
//#define POLOLU_MC33926

/* The RoboGaia encoder shield */
// #define ROBOGAIA

/* Encoders directly attached to Arduino board */
//#define ARDUINO_ENC_COUNTER

/* L298 Motor driver*/
//#define L298_MOTOR_DRIVER
//鑷畾涔夌紪鐮佸櫒椹卞姩
#define MY_ENCOUNT
#define MY_MOTOR_DRIVER
#endif

//鏄惁浣跨敤鑸垫満
//#define USE_SERVOS  // Enable use of PWM servos as defined in servos.h
#undef USE_SERVOS     // Disable use of PWM servos

/* Serial port baud rate */
#define BAUDRATE     57600

/* Maximum PWM signal */
#define MAX_PWM        100

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/* Include definition of serial commands */
#include "commands.h"//涓插彛閫氫俊鍛戒护

/* Sensor functions */
#include "sensors.h"

/* Include servo support if required */
#ifdef USE_SERVOS
#include <Servo.h>
#include "servos.h"
#endif
/************************************************************************************************************************************************************************************************************/
#ifdef USE_BASE
/* Motor driver function definitions */
#include "motor_driver.h"//鐢垫満椹卞姩

/* Encoder driver function definitions */
#include "encoder_driver.h"//缂栫爜鍣ㄩ┍鍔�

/* PID parameters and functions */
#include "diff_controller.h"//PID鎺у埗瀹炵幇

/* Run the PID loop at 30 times per second */
#define PID_RATE           30     // Hz

/* Convert the rate into an interval */
const int PID_INTERVAL = 1000 / PID_RATE;

/* Track the next time we make a PID calculation */
unsigned long nextPID = PID_INTERVAL;

/* Stop the robot if it hasn't received a movement command
  in this number of milliseconds */
#define AUTO_STOP_INTERVAL 5000 //鐢垫満鎺ユ敹閫熷害鎸囦护鍚庣殑杩愯鏃堕棿
long lastMotorCommand = AUTO_STOP_INTERVAL;
#endif

/* Variable initialization */

// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int index = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;

/* Clear the current command parameters */
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
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

  switch (cmd) {
    case GET_BAUDRATE:
      Serial.println(BAUDRATE);
      break;
    case ANALOG_READ:
      Serial.println(analogRead(arg1));
      break;
    case DIGITAL_READ:
      Serial.println(digitalRead(arg1));
      break;
    case ANALOG_WRITE:
      analogWrite(arg1, arg2);
      Serial.println("OK");
      break;
    case DIGITAL_WRITE:
      if (arg2 == 0) digitalWrite(arg1, LOW);
      else if (arg2 == 1) digitalWrite(arg1, HIGH);
      Serial.println("OK");
      break;
    case PIN_MODE:
      if (arg2 == 0) pinMode(arg1, INPUT);
      else if (arg2 == 1) pinMode(arg1, OUTPUT);
      Serial.println("OK");
      break;
    case PING:
      Serial.println(Ping(arg1));
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
    case STOP:
      //contrl_vel();
      u8 data[20];
      static u16 v=0;  //用于暂存电压值
      WireReadDataArray(ADC_BAT_ADDR, data, 2); //读取电压地址中存放的电压
      v = data[0] + (data[1] << 8); //将电压转换为 mV
      //Serial.print("V = "); 
      Serial.print(v); Serial.print(" ");Serial.println(v);
      //Serial.println("mV     "); //打印电压的值
      break;
    case MOTOR_SPEEDS:
      /* Reset the auto stop timer */
      lastMotorCommand = millis();
      if (arg1 == 0 && arg2 == 0) {
        //setMotorSpeeds(0, 0);
        contrl_vel();
        resetPID();
        moving = 0;
        Serial.println("停止");
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
  DDRD &= ~(1 << LEFT_ENC_PIN_A);
  DDRD &= ~(1 << LEFT_ENC_PIN_B);
  DDRC &= ~(1 << RIGHT_ENC_PIN_A);
  DDRC &= ~(1 << RIGHT_ENC_PIN_B);

  //enable pull up resistors
  PORTD |= (1 << LEFT_ENC_PIN_A);
  PORTD |= (1 << LEFT_ENC_PIN_B);
  PORTC |= (1 << RIGHT_ENC_PIN_A);
  PORTC |= (1 << RIGHT_ENC_PIN_B);

  // tell pin change mask to listen to left encoder pins
  PCMSK2 |= (1 << LEFT_ENC_PIN_A) | (1 << LEFT_ENC_PIN_B);
  // tell pin change mask to listen to right encoder pins
  PCMSK1 |= (1 << RIGHT_ENC_PIN_A) | (1 << RIGHT_ENC_PIN_B);

  // enable PCINT1 and PCINT2 interrupt in the general interrupt mask
  PCICR |= (1 << PCIE1) | (1 << PCIE2);
#elif defined MY_ENCOUNT
  /*****************************************搴曠洏椹卞姩涓殑setup*****************************************/

  Wire.begin();//鍒濆鍖� I2C锛屼互 Ardunio UNO 涓轰緥 I2C 鍙ｄ负锛欰4(SCL)銆丄5(CLK)
  printf_begin();
  delay(200);
  WireWriteDataArray(MOTOR_TYPE_ADDR, &MotorType, 1); //鍦ㄧ數鏈虹被鍨嬪湴鍧�涓啓鍏ョ數鏈虹被鍨嬬紪鍙�
  // Serial.println("运行到这");
  //鍚戝湴鍧�涓啓鍏ユ暟鎹� 锛坮eg锛氬湴鍧� val锛氭暟鎹唴瀹� len锛氭暟鎹暱搴︼級

  delay(5);
  WireWriteDataArray(MOTOR_ENCODER_POLARITY_ADDR, &MotorEncoderPolarity, 1); //鍚戠紪鐮佹柟鍚戞瀬鎬у湴鍧�涓啓鍏otorEncoderPolarity鍦板潃锛岄暱搴︿负1
  WireWriteDataArray(MOTOR_FIXED_SPEED_ADDR, result, 4); //寮�鏈烘椂淇濇寔鍋滄

  //澶嶄綅鍚庢竻闆讹紝璋冭瘯鐢�
  // rest_encode(0,0,0,0);

  WireReadDataArray(MOTOR_ENCODER_TOTAL_ADDR, (uint8_t*)start_EncodeTotal, 16); //鑾峰彇寮�濮嬫椂鐨勮浆鍔ㄩ噺

  pinMode(LED_BUILTIN, OUTPUT);//指示灯
  /*****************************************搴曠洏椹卞姩涓殑setup*****************************************/
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
}

/* Enter the main loop.  Read and parse input from the serial port
   and run any valid commands. Run a PID calculation at the target
   interval and check for auto-stop conditions.
*/
static u16 v = 0; //鐢ㄤ簬鏆傚瓨鐢靛帇鍊�
void loop() {
    u8 data[20];
    WireReadDataArray(ADC_BAT_ADDR, data, 2); //璇诲彇鐢靛帇鍦板潃涓瓨鏀剧殑鐢靛帇
   v = data[0] + (data[1] << 8); //灏嗙數鍘嬭浆鎹负 mV
  // Serial.print("V = "); Serial.print(v); Serial.println("mV     "); //鎵撳嵃鐢靛帇鐨勫��
  //  WireReadDataArray(MOTOR_ENCODER_TOTAL_ADDR, (uint8_t*)EncodeTotal, 16); //璇诲彇鐢垫満绱Н杞姩閲�
  //  printf("Encode1 = %ld  Encode2 = %ld  Encode3 = %ld  Encode4 = %ld  \r\n", EncodeTotal[0], EncodeTotal[1], EncodeTotal[2], EncodeTotal[3]);
  while (Serial.available() > 0) {
          
  
    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
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
    }
  }

  // If we are using base control, run a PID calculation at the appropriate intervals
#ifdef USE_BASE
  if (millis() > nextPID) {
    updatePID();
    //nextPID += PID_INTERVAL;//寰幆鏁村畾pid
    nextPID += 500;//500ms重发指令
  }

  // Check to see if we have exceeded the auto-stop interval
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) { //长时间没有命令则停止
    ;
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
