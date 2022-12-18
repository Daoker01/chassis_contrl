/***************************************************************
   Motor driver definitions

   Add a "#elif defined" block to this file to include support
   for a particular motor driver.  Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.

   *************************************************************/

#ifdef USE_BASE

#ifdef POLOLU_VNH5019
/* Include the Pololu library */
#include "DualVNH5019MotorShield.h"

/* Create the motor driver object */
DualVNH5019MotorShield drive;

/* Wrap the motor driver initialization */
void initMotorController() {
  drive.init();
}

/* Wrap the drive motor set speed function */
void setMotorSpeed(int i, int spd) {
  if (i == LEFT) drive.setM1Speed(spd);
  else drive.setM2Speed(spd);
}

// A convenience function for setting both motor speeds
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  setMotorSpeed(LEFT, leftSpeed);
  setMotorSpeed(RIGHT, rightSpeed);
}
#elif defined POLOLU_MC33926
/* Include the Pololu library */
#include "DualMC33926MotorShield.h"

/* Create the motor driver object */
DualMC33926MotorShield drive;

/* Wrap the motor driver initialization */
void initMotorController() {
  drive.init();
}

/* Wrap the drive motor set speed function */
void setMotorSpeed(int i, int spd) {
  if (i == LEFT) drive.setM1Speed(spd);
  else drive.setM2Speed(spd);
}

// A convenience function for setting both motor speeds
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  setMotorSpeed(LEFT, leftSpeed);
  setMotorSpeed(RIGHT, rightSpeed);
}
#elif defined L298_MOTOR_DRIVER
void initMotorController() {
  digitalWrite(RIGHT_MOTOR_ENABLE, HIGH);
  digitalWrite(LEFT_MOTOR_ENABLE, HIGH);
}

void setMotorSpeed(int i, int spd) {
  unsigned char reverse = 0;

  if (spd < 0)
  {
    spd = -spd;
    reverse = 1;
  }
  if (spd > 255)
    spd = 255;

  if (i == LEFT) {
    if      (reverse == 0) {
      analogWrite(RIGHT_MOTOR_FORWARD, spd);
      analogWrite(RIGHT_MOTOR_BACKWARD, 0);
    }
    else if (reverse == 1) {
      analogWrite(RIGHT_MOTOR_BACKWARD, spd);
      analogWrite(RIGHT_MOTOR_FORWARD, 0);
    }
  }
  else { /*if (i == RIGHT) //no need for condition*/
    if      (reverse == 0) {
      analogWrite(LEFT_MOTOR_FORWARD, spd);
      analogWrite(LEFT_MOTOR_BACKWARD, 0);
    }
    else if (reverse == 1) {
      analogWrite(LEFT_MOTOR_BACKWARD, spd);
      analogWrite(LEFT_MOTOR_FORWARD, 0);
    }
  }
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  setMotorSpeed(LEFT, leftSpeed);
  setMotorSpeed(RIGHT, rightSpeed);
}
#elif defined MY_MOTOR_DRIVER
void initMotorController() {
  I2c.write(I2C_ADDR, MOTOR_TYPE_ADDR, MotorType);//写入电机类型
  delay(5);
  I2c.write(I2C_ADDR, MOTOR_ENCODER_POLARITY_ADDR, MotorEncoderPolarity);//设置电机极性
  I2c.write(I2C_ADDR, MOTOR_FIXED_SPEED_ADDR, result, 4);//保持电机不转
}
void setMotorSpeed(int i, int spd) {
  int8_t setup_speeds[4] = {0, 0, 0, 0};
  if (i == LEFT) {
    setup_speeds[0] = spd;
    setup_speeds[2] = spd;
    setup_speeds[1] = save_setup_speeds[1];
    setup_speeds[3] = save_setup_speeds[3];
    //设定速度
    I2c.write(I2C_ADDR, MOTOR_FIXED_SPEED_ADDR, setup_speeds, 4);
   // I2c.write(I2C_ADDR, MOTOR_FIXED_PWM_ADDR, setup_speeds, 4);
    //储存速度
    save_setup_speeds[0] = spd; save_setup_speeds[2] = spd;
  }
  else { /*if (i == RIGHT) //no need for condition*/
    setup_speeds[1] = spd;
    setup_speeds[3] = spd;
    setup_speeds[0] = save_setup_speeds[0];
    setup_speeds[2] = save_setup_speeds[2];
    I2c.write(I2C_ADDR, MOTOR_FIXED_SPEED_ADDR, setup_speeds, 4);
    //I2c.write(I2C_ADDR, MOTOR_FIXED_PWM_ADDR, setup_speeds, 4);
    save_setup_speeds[1] = spd; save_setup_speeds[3] = spd;
  }
}
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  setMotorSpeed(LEFT, leftSpeed);
  setMotorSpeed(RIGHT, rightSpeed);
}
#else
#error A motor driver must be selected!
#endif

#endif
