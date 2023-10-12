#include <CAN.h>
#include <ps5Controller.h>
#include <Adafruit_PWMServoDriver.h>
#include "robot.hpp"
#include "util.hpp"

#define PIN_EMERGENCY 23 //緊急停止
#define PIN_LED 2 //インジケータ
#define PIN_CAN_RX 26
#define PIN_CAN_TX 27
#define ENA_LEFT 13
#define IN1_LEFT 5
#define IN2_LEFT 4

#define ENA_RIGHT 12
#define IN1_RIGHT 17
#define IN2_RIGHT 16

#define ENA_ARM 14
#define IN1_ARM 19
#define IN2_ARM 18

namespace dc{
  const int frequency = 500;
  const int pwm_channel_left = 0;
  const int pwm_channel_right = 1;
  const int pwm_channel_arm = 2;
  const int resolution = 8;
  const float duty_gain = 0.5f;
}
namespace servo{
  const int servo_min = 500;  // 最小パルス幅(μs)
  const int servo_max = 2400; // 最大パルス幅(μs)
  const int PIN_LEFT = 0;
  const int PIN_RIGHT = 1;
  const int closed_angle = 0;
  const int open_angle = 90;
}
namespace wheel{
  const float straight_gain = 1.0f;
  const float steering_gain = 0.6f;
  const bool is_reverse_left = false;
  const bool is_reverse_right = false;
  const float duty_gain_left = 1.f;
  const float duty_gain_right = 1.f;
}
namespace robomaster{
  const int current = 500;
}

using namespace std;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);  // PCA9685のI2Cアドレスを指定
UpEdge upedge_ps;
UpEdge upedge_circle;
UpEdge upedge_square;
bool is_emergency = true;
bool is_open_arm = true;
bool is_reverse_mode = false;


void setup() {
  //シリアル
  Serial.begin(115200);
  while (!Serial);

  // ps5
  ps5.begin("24:a6:fa:5d:c7:46"); //MACアドレス
  Serial.println("PS5 Ready.");

  // CAN
  Serial.println("CAN Sender");

  CAN.setPins(PIN_CAN_RX, PIN_CAN_TX);
  // start the CAN bus at 1M bps
  if (!CAN.begin(1000E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }

  // 以下2行を追記
  volatile uint32_t* pREG_IER = (volatile uint32_t*)0x3ff6b010;
  *pREG_IER &= ~(uint8_t)0x10;

  // 遠隔緊急停止
  pinMode(PIN_EMERGENCY,OUTPUT);

  //インジケータ
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  // // DCモータ
  pinMode(ENA_LEFT, OUTPUT);
  pinMode(ENA_RIGHT, OUTPUT);
  pinMode(ENA_ARM, OUTPUT);
  pinMode(IN1_LEFT, OUTPUT);
  pinMode(IN1_RIGHT, OUTPUT);
  pinMode(IN1_ARM, OUTPUT);
  pinMode(IN2_LEFT, OUTPUT);
  pinMode(IN2_RIGHT, OUTPUT);
  pinMode(IN2_ARM, OUTPUT);

  digitalWrite(IN1_LEFT, LOW);
  digitalWrite(IN1_RIGHT, LOW);
  digitalWrite(IN1_ARM, LOW);
  digitalWrite(IN2_LEFT, LOW);
  digitalWrite(IN2_RIGHT, LOW);
  digitalWrite(IN2_ARM, LOW);

  ledcSetup(dc::pwm_channel_left, dc::frequency, dc::resolution);
  ledcSetup(dc::pwm_channel_right, dc::frequency, dc::resolution);
  ledcSetup(dc::pwm_channel_arm, dc::frequency, dc::resolution);
  ledcAttachPin(ENA_LEFT, dc::pwm_channel_left);
  ledcAttachPin(ENA_RIGHT, dc::pwm_channel_right);
  ledcAttachPin(ENA_ARM, dc::pwm_channel_arm);

  // サーボ
  pwm.begin();         // 初期設定
  pwm.setPWMFreq(50);  // PWM周期を50Hzに設定

  delay(1000);
}

void loop() {
  // コントローラ入力
  if(!ps5.isConnected()){
    digitalWrite(PIN_EMERGENCY, LOW);
    Serial.printf("コントローラが接続されていません \n");
    return;
  }
  // インジケータ
  digitalWrite(PIN_LED, HIGH);

  // 変数の定義
  float wheel_gain_left = 0.f;
  float wheel_gain_right = 0.f;
  int arm_angle_left = 0;
  int arm_angle_right = 0;

  //遠隔緊急停止
  if (upedge_ps(ps5.PSButton())){
    is_emergency = !is_emergency;
    if(is_emergency) digitalWrite(PIN_EMERGENCY, LOW);
    else digitalWrite(PIN_EMERGENCY, HIGH);
    Serial.printf("緊急停止 : %d\n", is_emergency);
  }

  // 反転モード
  if (upedge_square(ps5.Square())){
    is_reverse_mode = !is_reverse_mode;
    Serial.printf("反転モード : %d\n", is_reverse_mode);
  }

  // DCモータ
  // if (ps5.R2()) {
  //   Serial.printf("R2 button at %d\n", ps5.R2Value());
  // }
  float duty_dc_arm = 0.f;
  if (ps5.L2()) {
    digitalWrite(IN1_ARM, LOW);
    digitalWrite(IN2_ARM, HIGH);
    duty_dc_arm += (is_reverse_mode ? -1 : 1) * (float)ps5.L2Value()/255.f;
  }
  else if (ps5.R2()) {
    digitalWrite(IN1_ARM, HIGH);
    digitalWrite(IN2_ARM, LOW);
    duty_dc_arm -= (is_reverse_mode ? -1 : 1) * (float)ps5.R2Value()/255.f;
  }
  if(duty_dc_arm > 0.f) { digitalWrite(IN1_ARM, LOW); digitalWrite(IN2_ARM, HIGH); }
  else if(duty_dc_arm < 0.f) { digitalWrite(IN1_ARM, HIGH); digitalWrite(IN2_ARM, LOW); }
  else { digitalWrite(IN1_ARM, HIGH); digitalWrite(IN2_ARM, HIGH); duty_dc_arm = 1.f;}

  ledcWrite(dc::pwm_channel_arm, (int)(duty_dc_arm * dc::duty_gain *255));

  // アーム
  if (upedge_circle(ps5.Circle())){
    is_open_arm = !is_open_arm;
    if(is_open_arm) {arm_angle_left = servo::closed_angle; arm_angle_right = servo::closed_angle;}
    else {arm_angle_left = servo::open_angle; arm_angle_right = servo::closed_angle;}

    Serial.printf("アーム角度左 : %d\n", arm_angle_left);

    arm_angle_left = map(arm_angle_left, 0, 180, servo::servo_min, servo::servo_max);  // 角度(0~180)をパルス幅(500~2400μs)に変換
    arm_angle_right = map(arm_angle_right, 0, 180, servo::servo_min, servo::servo_max);  // 角度(0~180)をパルス幅(500~2400μs)に変換
    pwm.writeMicroseconds(servo::PIN_LEFT, arm_angle_left);
    pwm.writeMicroseconds(servo::PIN_RIGHT, arm_angle_right);
    delay(1000);
  }

  //足回り
  if (abs(ps5.LStickY()) > 10) {
    wheel_gain_left += (float)ps5.LStickY() * (is_reverse_mode ? -1 : 1) * wheel::straight_gain /127.f;
    wheel_gain_right += (float)ps5.LStickY() * (is_reverse_mode ? -1 : 1) * wheel::straight_gain /127.f;
  }
  if (ps5.RStickX() > 10) {
    wheel_gain_left += (float)ps5.RStickX() *wheel::steering_gain /127.f;
    wheel_gain_right -= (float)ps5.RStickX() *wheel::steering_gain /127.f;
  }
  if (ps5.RStickX() < -10) {
    wheel_gain_left += (float)ps5.RStickX() *wheel::steering_gain /127.f;
    wheel_gain_right -= (float)ps5.RStickX() *wheel::steering_gain /127.f;
  }

  // Serial.printf("left gain %f\n", wheel_gain_left);
  // Serial.printf("right gain %f\n", wheel_gain_right);

  float duty_dc_left = (wheel::is_reverse_left ? -1 : 1) * constrain(wheel_gain_left * wheel::duty_gain_left, -1.f, 1.f);
  float duty_dc_right = (wheel::is_reverse_right ? -1 : 1) * constrain(wheel_gain_right * wheel::duty_gain_right, -1.f, 1.f);

  if(duty_dc_left > 0.f) { digitalWrite(IN1_LEFT, LOW); digitalWrite(IN2_LEFT, HIGH); }
  else if(duty_dc_left < 0.f) { digitalWrite(IN1_LEFT, HIGH); digitalWrite(IN2_LEFT, LOW); }
  else { digitalWrite(IN1_LEFT, HIGH); digitalWrite(IN2_LEFT, HIGH); duty_dc_left = 1.f;}

  if(duty_dc_right > 0.f) { digitalWrite(IN1_RIGHT, LOW); digitalWrite(IN2_RIGHT, HIGH); }
  else if(duty_dc_right < 0.f) { digitalWrite(IN1_RIGHT, HIGH); digitalWrite(IN2_RIGHT, LOW); }
  else { digitalWrite(IN1_RIGHT, HIGH); digitalWrite(IN2_RIGHT, HIGH); duty_dc_right = 1.f;}

  duty_dc_left = abs(duty_dc_left);
  duty_dc_right = abs(duty_dc_right);

  ledcWrite(dc::pwm_channel_left, (int)(duty_dc_left*255));
  ledcWrite(dc::pwm_channel_right, (int)(duty_dc_right*255));


  int inagaki_current = 0;
  if(is_reverse_mode) inagaki_current = robomaster::current;
  unsigned char data0 = (unsigned char)(inagaki_current >> 8);
  unsigned char data1 = (unsigned char)(inagaki_current & 0xFF);

  CAN.beginPacket(0x200);
  CAN.write(data0);
  CAN.write(data1);
  CAN.endPacket();
  Serial.printf("\n");

  delay(200);
}
