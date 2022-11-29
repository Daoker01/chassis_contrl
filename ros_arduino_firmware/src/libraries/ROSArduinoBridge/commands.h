/* Define single-letter commands that will be sent by the PC over the
   serial link.
*/

#ifndef COMMANDS_H
#define COMMANDS_H

#define ANALOG_READ    'a'
#define GET_BAUDRATE   'b'
#define PIN_MODE       'c'
#define DIGITAL_READ   'd'
#define READ_ENCODERS  'e'//读取编码器计数
#define MOTOR_SPEEDS   'm'//读取小车速度
#define PING           'p'
#define RESET_ENCODERS 'r'//重置编码器
#define SERVO_WRITE    's'
#define SETUP_SPEEDS    'v'
#define SERVO_READ     't'
#define UPDATE_PID     'u'
#define DIGITAL_WRITE  'w'
#define ANALOG_WRITE   'x'
#define LEFT            0
#define RIGHT           1

#endif


