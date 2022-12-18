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
#elif defined MY_ENCOUNT
//鐎电厧鍙嗘す鍗炲З閺夊じ鑵戦惃鍕毐閺佹澘鐤勯悳锟�
//start******************************************************************************************************************************************************************

bool WireWriteByte(uint8_t val)
{
  Wire.beginTransmission(I2C_ADDR);//瀵拷婵绔村▎鈩冩殶閹诡喕绱舵潏鎿勭礉閸欐垿锟戒椒绔存稉鐙�2C瀵拷婵鐡х粭锟�0x34
// I2c.begin();
  Wire.write(val);//閸嶅繋绮犻張鍝勫絺闁焦鏆熼幑锟�
  if ( Wire.endTransmission() != 0 ) { //鏉╂柨娲栭崐闂磋礋0閹靛秵鍨氶崝鐕傜礉閸氾箑鍨径杈Е
    return false;
  }
  return true;
}
//閸氭垵婀撮崸锟芥稉顓炲晸閸忋儲鏆熼幑锟� 閿涘澁eg閿涙艾婀撮崸锟� val閿涙碍鏆熼幑顔煎敶鐎癸拷 len閿涙碍鏆熼幑顕�鏆辨惔锔肩礆
bool WireWriteDataArray(  uint8_t reg, uint8_t *val, unsigned int len)
{
  unsigned int i;

  Wire.beginTransmission(I2C_ADDR);
  Wire.write(reg);//閸欐垿锟戒礁婀撮崸锟介弫鐗堝祦
  for (i = 0; i < len; i++) {
    Wire.write(val[i]);//閸欐垿锟戒礁鍞寸�癸拷
  }
 //Serial.println("进入endTransmission");
  bool test32 = Wire.endTransmission();//在A4 和 A5 接4.7k的5v上拉电阻，要不然容易无响应
// Serial.println(test32);
 // Serial.println("出endTransmission");
  if ( test32 != 0 ) {
    return false;
  }

  return true;
}
//鐠囪褰囬崷鏉挎絻娑擃厾娈戦弫鐗堝祦閿涘澁eg閿涙艾婀撮崸锟� val閿涙碍鏆熼幑顔煎敶鐎圭櫢绱�
bool WireReadDataByte(uint8_t reg, uint8_t &val)
{
  if (!WireWriteByte(reg)) {//婵″倹鐏塛ireWriteByte閿涘牞绱氶棃鐐垫埂
    return false;
  }

  Wire.requestFrom(I2C_ADDR, 1);//娑撴槒顔曟径鍥嚞濮瑰倷绮犵拋鎯ь槵I2C_ADDR閸︽澘娼冩径鍕畱娑擄拷娑擃亜鐡ч懞鍌︾礉鏉╂瑤閲滅�涙濡崣顖欎簰鐞氼偂瀵岀拋鎯ь槵閻拷 read()閹存溂vailable()閹恒儱褰�
  while (Wire.available()) //閹恒儲鏁归懢宄板絿閻ㄥ嫬鐡ч懞锟�
  {
    val = Wire.read();//Wire.requestFrom()鐠囬攱鐪版禒搴㈡簚閺佺増宓侀崥搴礉閸欘垯浜掓担璺ㄦ暏read閹恒儲鏁�
  }

  return true;
}
//鐠囪褰囬崷鏉挎絻娑擃厽瀵氱�规岸鏆辨惔锔炬畱閺佺増宓侀敍鍧甧g閿涙艾婀撮崸锟� val閿涙碍鏆熼幑顔煎敶鐎癸拷 len閿涙碍鏆熼幑顕�鏆辨惔锔肩礆
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
  Serial.write( c );//Serial.print() 閸欐垿锟戒胶娈戦弰顖氱摟缁楋讣绱漇erial.write() 閸欐垿锟戒胶娈戠�涙濡�.
  return c;
}
void printf_begin(void)
{
  fdevopen( &serial_putc, 0 );//鐏忓棗鍤遍弫鐨奺rial_putc閹稿洤鎮滄稉鎻掑經
}




//uint8_t MotorType = MOTOR_TYPE_TT;  //鐠佹儳鐣鹃悽鍨簚缁鐎�
//uint8_t MotorEncoderPolarity = 1;
//int8_t result[4]={0,0,0,0};
//int32_t EncodeTotal[4]; //閻€劋绨弳鍌氱摠閻㈠灚婧�缁鳖垳袧鏉烆剙濮╅柌蹇曟畱閸婄》绱濆锝堟祮闁帒顤冮敍灞藉冀鏉烆剟锟芥帒鍣�

/*濞村锟界喎鍤遍弫锟�*/
long start_time = millis();//娑擄拷娑擃亣顓哥粻妤�鎳嗛張鐔烘畱瀵拷婵妞傞崚浼欑礉閸掓繂顫愰崐闂磋礋 millis();
long interval_time = 50;//娑擄拷娑擃亣顓哥粻妤�鎳嗛張锟� 50ms
double current_vel[4];
//int32_t start_EncodeTotal[4];//閺嬪嫰锟界姷鏁ゆ禍搴″亶鐎涙ê绱戞慨瀣閻ㄥ嫯鍓﹂崘鍙夋殶閻ㄥ嫭鏆熺紒锟�
//int32_t current_EncodeTotal[4];//閺嬪嫰锟界姷鏁ゆ禍搴″亶鐎涙ê缍嬮崜宥堝墻閸愬弶鏆熼惃鍕殶缂侊拷
void get_current_vel()
  {
  long right_now = millis();
  long past_time = right_now - start_time;//鐠侊紕鐣婚柅婵嗗箵閻ㄥ嫭妞傞梻锟�

  if(past_time >= interval_time){//婵″倹鐏夐柅婵嗗箵閺冨爼妫挎径褌绨粵澶夌艾娑擄拷娑擃亣顓哥粻妤�鎳嗛張锟�
    //1.缁備焦顒涙稉顓熸焽
   // noInterrupts();
    //2.鐠侊紕鐣绘潪顒勶拷锟� 鏉烆剟锟界喎宕熸担宥呭讲娴犮儲妲哥粔鎺炵礉娑旂喎褰叉禒銉︽Ц閸掑棝鎸�... 閼奉亜鐣炬稊澶婂祮閸欙拷
    WireReadDataArray(MOTOR_ENCODER_TOTAL_ADDR,(uint8_t*)current_EncodeTotal,16);//閼惧嘲褰囪ぐ鎾冲閻ㄥ嫯娴嗛崝銊╁櫤
    for(int i=0;i<4;i++)
    {
      current_vel[i]=(current_EncodeTotal[i]-start_EncodeTotal[i])*0.3625/past_time*1000;//濮瑰倸缍嬮崜宥呮倗娑擃亣鐤嗙�涙劗娈戦柅鐔峰閿涘苯宕熸担宄琺/s

    }
    //3.闁插秶鐤嗙拋鈩冩殶閸ｏ拷
        for(int i=0;i<4;i++)
    {
      start_EncodeTotal[i]=current_EncodeTotal[i];
    }
    //4.闁插秶鐤嗗锟芥慨瀣闂傦拷
    start_time = right_now;
    //5.闁插秴鎯庢稉顓熸焽
   // interrupts();

     Serial.println("閸ユ稐閲滄潪顔肩摍閻ㄥ嫰锟界喎瀹�(閸楁洑缍卪m/s)閸掑棗鍩嗘稉鐚寸窗");
     Serial.print(current_vel[0]);Serial.print("\t");Serial.print(current_vel[1]);Serial.print("\t");Serial.print(current_vel[2]);Serial.print("\t");Serial.print(current_vel[3]);Serial.println("\t");

  }
  }
  void rest_encode(int32_t M1,int32_t M2,int32_t M3,int32_t M4)//缂傛牜鐖滈崳銊吀閺佷即鍣哥拋锟�
  {
   EncodeTotal[0]=M1,EncodeTotal[1]=M2,EncodeTotal[2]=M3,EncodeTotal[3]=M4;//閺佹壆绮嶇挧瀣拷锟�0
  WireWriteDataArray(MOTOR_ENCODER_TOTAL_ADDR,(uint8_t*)EncodeTotal,16);//閹跺﹪鈹嶉崝銊︽緲娑撳﹦娈戠紓鏍垳閸ｃ劏顓搁弫鏉挎珤瑜帮拷0
  // start_EncodeTotal[0]=0;start_EncodeTotal[1]=0;start_EncodeTotal[2]=0;start_EncodeTotal[3]=0;//闁灝鍘ゅ〒锟�0閸氬海顑囨稉锟藉▎鈩冪ゴ闁喐鏆熼幑顔肩磽鐢拷
  }
  void contrl_vel(int16_t *val)                                             //閹稿洤鐣鹃崥鍕枂閻ㄥ嫰锟界喎瀹抽敍灞藉礋娴ｅ稄绱癿/s
  {//鐟曚焦濡搁柅鐔峰鏉烆剙瀵查幋鎰墻閸愬弶鏆熷В锟�10ms,result=val*10/0.3625
  int16_t vel2pulse[4]={0,0,0,0};
  for(int i=0;i<4;i++){
    //vel2pulse[i]=val[i]/100/0.3625;//鏉╂瑩鍣锋导姘帥閹跺al[i]/100閸欐牗鏆ｉ崹瀣晙闂勩倓浜�0.3625
    vel2pulse[i]=val[i]/36.25;
  }
    while(true != WireWriteDataArray(MOTOR_FIXED_SPEED_ADDR,(int8_t *)vel2pulse,4)){
      Serial.println("韫囷拷");;//閻㈠彉绨潏鎾冲弳閻ㄥ嫭鏆熼幑顔煎涧閼虫垝璐熼弫鏉戠�烽敍灞惧付閸掑墎绨挎惔锔跨秵
    }
  //      Serial.println("閸ユ稐閲滄潪顔肩摍閻ㄥ嫮娲伴弽鍥拷鐔峰閿涘牆宕熸担宄琺/s閿涘璐熼敍锟�");
  //     Serial.print(val[0]);Serial.print("\t");Serial.print(val[1]);Serial.print("\t");Serial.print(val[2]);Serial.print("\t");Serial.print(val[3]);Serial.println("\t");
  //      Serial.println("閸ユ稐閲滄潪顔肩摍閻ㄥ嫮娲伴弽鍥墻閸愶拷(閸楁洑缍呴懘澶婂暱/10ms)娑撶尨绱�");
  //     Serial.print(vel2pulse[0]);Serial.print("\t");Serial.print(vel2pulse[1]);Serial.print("\t");Serial.print(vel2pulse[2]);Serial.print("\t");Serial.print(vel2pulse[3]);Serial.println("\t");
  //     for(int i=0;i<4;i++)
  //     {
  //     delay(2000);
  //     get_current_vel();
  //     }
  }
  void contrl_vel(){
  int16_t vel2pulse[4]={0,0,0,0};
  contrl_vel(vel2pulse);
  //Serial.println("紧急停车");
  }

long resetEncoder_L()
{
  //瀹革箒绔熼悽鍨簚鐠佲剝鏆熷〒鍛存祩閿涘苯宓嗛悽鍨簚1閵嗕胶鏁搁張锟�3鐠佲剝鏆熷〒鍛存祩
  int32_t reset_encode[4] = {0, 0, 0, 0};
  while (16 != WireReadDataArray(MOTOR_ENCODER_TOTAL_ADDR, (uint8_t*)reset_encode, 16)) {
    ;//鐠囪褰囬崢鐔告箒缂傛牜鐖滈崳銊吀閺侊拷
  }
  reset_encode[0] = 0; reset_encode[2] = 0; //閹讹拷1,3閻㈠灚婧�鐠佲剝鏆熷〒鍛存祩
  while (false == WireWriteDataArray(MOTOR_ENCODER_TOTAL_ADDR, (uint8_t*)reset_encode, 16)) {
    ;//閹跺﹥鏆熼幑顔煎晸閸ョ偛骞�
  }
  return;
}
long resetEncoder_R()
{
  //閸欏疇绔熼悽鍨簚鐠佲剝鏆熷〒鍛存祩閿涘苯宓嗛悽鍨簚2閵嗕胶鏁搁張锟�4鐠佲剝鏆熷〒鍛存祩
  int32_t reset_encode[4] = {0, 0, 0, 0};
  while (WireReadDataArray(MOTOR_ENCODER_TOTAL_ADDR, (uint8_t*)reset_encode, 16) != 16) {
    ;//鐠囪褰囬崢鐔告箒缂傛牜鐖滈崳銊吀閺侊拷
  }
  reset_encode[1] = 0; reset_encode[3] = 0; //閹讹拷2,4閻㈠灚婧�鐠佲剝鏆熷〒鍛存祩
  while (false == WireWriteDataArray(MOTOR_ENCODER_TOTAL_ADDR, (uint8_t*)reset_encode, 16)) {
    ;//閹跺﹥鏆熼幑顔煎晸閸ョ偛骞�
  }
  return;
}
void resetEncoder(int i) {
  if (i == LEFT) {
    resetEncoder_L();
    return;
  }
  if (i == RIGHT) {
    resetEncoder_R();
    return;
  }
  return;
}

long readEncoder(int i) {
  int32_t reset_encode[4] = {0, 0, 0, 0};
  while (WireReadDataArray(MOTOR_ENCODER_TOTAL_ADDR, (uint8_t*)reset_encode, 16) != 16) {
    ;//鐠囪褰囬崢鐔告箒缂傛牜鐖滈崳銊吀閺侊拷
  }
  if (i == LEFT) {
    //鐠囪褰�1,3缂傛牜鐖滈崳銊︽殶閹癸拷
    return (reset_encode[0]+reset_encode[2])/2;
  }
  else return (reset_encode[1]+reset_encode[3])/2;
}

//end********************************************************************************************************************************************************************

#else
#error A encoder driver must be selected!
#endif


/* Wrap the encoder reset function */
void resetEncoders() {
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
}

#endif
