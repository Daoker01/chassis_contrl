/* *************************************************************
   Encoder definitions

   Add an "#ifdef" block to this file to include support for
   a particular encoder board or library. Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.

   ************************************************************ */

#ifdef USE_BASE

#ifdef ROBOGAIA
/* The Robogaia Mega Encoder shield */
#include "MegaEncoderCounter.h"

/* Create the encoder shield object */
MegaEncoderCounter encoders = MegaEncoderCounter(4); // Initializes the Mega Encoder Counter in the 4X Count mode

/* Wrap the encoder reading function */
long readEncoder(int i) {
  if (i == LEFT) return encoders.YAxisGetCount();
  else return encoders.XAxisGetCount();
}

/* Wrap the encoder reset function */
void resetEncoder(int i) {
  if (i == LEFT) return encoders.YAxisReset();
  else return encoders.XAxisReset();
}
#elif defined(ARDUINO_ENC_COUNTER)
volatile long left_enc_pos = 0L;
volatile long right_enc_pos = 0L;
static const int8_t ENC_STATES [] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0}; //encoder lookup table

/* Interrupt routine for LEFT encoder, taking care of actual counting */
ISR (PCINT2_vect) {
  static uint8_t enc_last = 0;

  enc_last <<= 2; //shift previous state two places
  enc_last |= (PIND & (3 << 2)) >> 2; //read the current state into lowest 2 bits

  left_enc_pos += ENC_STATES[(enc_last & 0x0f)];
}

/* Interrupt routine for RIGHT encoder, taking care of actual counting */
ISR (PCINT1_vect) {
  static uint8_t enc_last = 0;

  enc_last <<= 2; //shift previous state two places
  enc_last |= (PINC & (3 << 4)) >> 4; //read the current state into lowest 2 bits

  right_enc_pos += ENC_STATES[(enc_last & 0x0f)];
}

/* Wrap the encoder reading function */
long readEncoder(int i) {
  if (i == LEFT) return left_enc_pos;
  else return right_enc_pos;
}

/* Wrap the encoder reset function */
void resetEncoder(int i) {
  if (i == LEFT) {
    left_enc_pos = 0L;
    return;
  } else {
    right_enc_pos = 0L;
    return;
  }
}
#elif defined MY_ENC_COUNTER
int serial_putc( char c, struct __file * )
{
  Serial.write( c );//Serial.print() 发送的是字符，Serial.write() 发送的字节.
  return c;
}
void printf_begin(void)
{
  fdevopen( &serial_putc, 0 );//将函数serial_putc指向串口
}
//功能，读取编码器脉冲数量
long readEncoder(int i) {
  I2c.read(I2C_ADDR, MOTOR_ENCODER_TOTAL_ADDR, 16, (uint8_t*)EncodeTotal);//读取编码器计数，这里有时候会读取出错，可以判断返回值然后再重新读取
  if (i == LEFT) return EncodeTotal[0];//左轮为1,3电机，即EncodeTotal[0]  EncodeTotal[2]
  else return EncodeTotal[1];
}
void resetEncoder(int i) {
  EncodeTotal[0] = 0, EncodeTotal[1] = 0, EncodeTotal[2] = 0, EncodeTotal[3] = 0;
  I2c.write(I2C_ADDR, MOTOR_ENCODER_TOTAL_ADDR, (uint8_t*)EncodeTotal, 16);//编码器计数归零
}

#else
#error A encoder driver must be selected!
#endif

/* Wrap the encoder reset function */
void resetEncoders() {
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
}

#endif

