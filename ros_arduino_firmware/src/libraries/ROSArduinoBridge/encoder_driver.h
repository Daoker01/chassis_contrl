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
#elif defined MY_ENCOUNT
  //閹跺﹪鈹嶉崝銊︽緲娑擃厾娈戞径瀛樻瀮娴犺泛鍞寸�圭懓绱╅崗锟�
  //start***********************************************************************************************************************************************************************
  #include <Wire.h>
  #include <I2C.h>
  #define I2C_ADDR        0x34            //12C閸︽澘娼�
#define ADC_BAT_ADDR                  0   //閻㈤潧甯囬崷鏉挎絻
#define MOTOR_TYPE_ADDR               20 //缂傛牜鐖滈悽鍨簚缁鐎风拋鍓х枂
#define MOTOR_ENCODER_POLARITY_ADDR   21 //鐠佸墽鐤嗙紓鏍垳閺傜懓鎮滈弸浣癸拷褝绱�
//婵″倹鐏夐崣鎴犲箛閻㈠灚婧�鏉烆剟锟界喐鐗撮張顑跨瑝閸欐甯堕崚璁圭礉鐟曚椒绠為張锟借箛顐︼拷鐔峰鏉烆剙濮╅敍宀冾洣娑斿牆浠犲鈧拷鍌氬讲娴犮儱鐨㈠銈呮勾閸э拷閻ㄥ嫬锟藉ジ鍣搁弬鎷岊啎缂冾喕绔存稉锟�
//閼煎啫娲�0閹达拷1閿涘矂绮拋锟�0

#define MOTOR_FIXED_PWM_ADDR      31 //閸ュ搫鐣綪WM閹貉冨煑閿涘苯鐫樻禍搴＄磻閻滎垱甯堕崚锟�,閼煎啫娲块敍锟�-100~100閿涳拷
//#define SERVOS_ADDR_CMD 40        
#define MOTOR_FIXED_SPEED_ADDR    51 //閸ュ搫鐣炬潪顒勶拷鐔稿付閸掕绱濈仦鐐扮艾闂傤厾骞嗛幒褍鍩楅敍锟�
//閸楁洑缍呴敍姘冲墻閸愬弶鏆熷В锟�10濮ｎ偆顫楅敍宀冨瘱閸ヨ揪绱欓弽瑙勫祦閸忚渹缍嬮惃鍕椽閻胶鏁搁張鐑樻降閿涘苯褰堢紓鏍垳缁炬寧鏆熼敍宀�鏁搁崢瀣亣鐏忓骏绱濈拹鐔绘祰婢堆冪毈缁涘濂栭崫宥忕礉娑擄拷閼割剙婀崵50瀹革箑褰搁敍锟�
#define MOTOR_ENCODER_TOTAL_ADDR  60 //4娑擃亞绱惍浣烘暩閺堝搫鎮囬懛顏嗘畱閹槒鍓﹂崘鎻掞拷锟�
//婵″倹鐏夊鑼叀閻㈠灚婧�濮ｅ繗娴嗘稉锟介崷鍫㈡畱閼村鍟块弫棰佽礋U閿涘苯寮靛鑼叀鏉烆喖鐡欓惃鍕纯瀵板嚍閿涘矂鍋呮稊鍫濇皑閸欘垯浜掗柅姘崇箖閼村鍟跨拋鈩冩殶閻ㄥ嫭鏌熷蹇撶繁閻儲鐦℃稉顏囩枂鐎涙劘顢戞潻娑氭畱鐠烘繄顬�
//濮ｆ柨顩х拠璇插煂閻㈠灚婧�1閻ㄥ嫯鍓﹂崘鍙夛拷缁樻殶娑撶瘻閿涘矂鍋呮稊鍫ｎ攽鏉╂稓娈戠捄婵堫瀲娑擄拷(P/U) * (3.14159*D)
//鐎甸�涚艾娑撳秴鎮撻惃鍕暩閺堝搫褰叉禒銉ㄥ殰鐞涘本绁寸拠鏇熺槨閸﹀牏娈戦懘澶婂暱閺佺櫊閿涘苯褰叉禒銉﹀閸斻劍妫嗘潪锟�10閸﹀牐顕伴崙楦垮墻閸愬弶鏆熼敍宀�鍔ч崥搴″絿楠炲啿娼庨崐鐓庣繁閸戯拷

//閻㈠灚婧�缁鐎烽崗铚傜秼閸婏拷
#define MOTOR_TYPE_WITHOUT_ENCODER        0  //閺冪姷绱惍浣告珤閻ㄥ嫮鏁搁張锟�
#define MOTOR_TYPE_TT                     1  //TT缂傛牜鐖滈悽鍨簚
#define MOTOR_TYPE_N20                    2  //n20缂傛牜鐖滈悽鍨簚
#define MOTOR_TYPE_JGB37_520_12V_110RPM   3 //绾句胶骞嗗В蹇氭祮閺勶拷44娑擃亣鍓﹂崘锟�   閸戝繘锟界喐鐦�:90  姒涙顓�
bool WireWriteByte(uint8_t val);                                             //濡拷濞村鈹嶉崝銊︽緲I2C閺勵垰鎯佹径鍕艾閸欘垱鎼锋担婊呭Ц閹拷
bool WireWriteDataArray(  uint8_t reg,uint8_t *val,unsigned int len);       //閸氭垵婀撮崸锟芥稉顓炲晸閸忋儲鏆熼幑锟� 閿涘澁eg閿涙艾婀撮崸锟� val閿涙碍鏆熼幑顔煎敶鐎癸拷 len閿涙碍鏆熼幑顕�鏆辨惔锔肩礆
bool WireReadDataByte(uint8_t reg, uint8_t &val);                           //鐠囪褰囬崷鏉挎絻娑擃厾娈戦弫鐗堝祦閿涘澁eg閿涙艾婀撮崸锟� val閿涙碍鏆熼幑顔煎敶鐎圭櫢绱�
int WireReadDataArray(   uint8_t reg, uint8_t *val, unsigned int len);      //鐠囪褰囬崷鏉挎絻娑擃厽瀵氱�规岸鏆辨惔锔炬畱閺佺増宓侀敍鍧甧g閿涙艾婀撮崸锟� val閿涙碍鏆熼幑顔煎敶鐎癸拷 len閿涙碍鏆熼幑顕�鏆辨惔锔肩礆
int serial_putc( char c, struct __file * );
void printf_begin(void);
void get_current_vel();                                                     //娴狀櫝/s娑撳搫宕熸担宥堢翻閸戝搫缍嬮崜宥呯毈鏉烇箑娲撴稉顏囩枂鐎涙劗娈戦柅鐔峰
void rest_encode(int32_t M1,int32_t M2,int32_t M3,int32_t M4);             //缂傛牜鐖滈崳銊吀閺佷即鍣哥拋鎾呯礉閸忋劑鍎撮幐鍥х暰娑擄拷0 閸掓瑦绔婚梿锟�
void contrl_vel(int16_t *val);                                             //閹稿洤鐣鹃崥鍕枂閻ㄥ嫰锟界喎瀹抽敍灞藉礋娴ｅ稄绱癿m/s
long resetEncoder_R();
long resetEncoder_L();
void contrl_vel();                                            //急停

uint8_t MotorType = MOTOR_TYPE_TT;  //鐠佹儳鐣鹃悽鍨簚缁鐎�
uint8_t MotorEncoderPolarity = 1; 
int8_t result[4]={0,0,0,0};
int32_t EncodeTotal[4]; //閻€劋绨弳鍌氱摠閻㈠灚婧�缁鳖垳袧鏉烆剙濮╅柌蹇曟畱閸婄》绱濆锝堟祮闁帒顤冮敍灞藉冀鏉烆剟锟芥帒鍣�
int32_t start_EncodeTotal[4];//閺嬪嫰锟界姷鏁ゆ禍搴″亶鐎涙ê绱戞慨瀣閻ㄥ嫯鍓﹂崘鍙夋殶閻ㄥ嫭鏆熺紒锟�
int32_t current_EncodeTotal[4];//閺嬪嫰锟界姷鏁ゆ禍搴″亶鐎涙ê缍嬮崜宥堝墻閸愬弶鏆熼惃鍕殶缂侊拷
//end****************************************************************************************************************************************************************
#endif
   
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();



