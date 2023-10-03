#include <CAN.h>
#include <ps5Controller.h>
#include <Adafruit_PWMServoDriver.h>
#include "robot.hpp"
#include "util.hpp"

#define PIN_EMERGENCY 23 //緊急停止
#define PIN_CAN_RX 26
#define PIN_CAN_TX 27
#define ENA 13
#define IN1 5
#define IN2 4

namespace dc{
  const int frequency = 500;
  const int pwm_channel = 0;
  const int resolution = 8;
}
namespace servo{
  const int servo_min = 500;  // 最小パルス幅(μs)
  const int servo_max = 2400; // 最大パルス幅(μs)
  const int PIN_LEFT = 0;
  const int PIN_RIGHT = 1;
  const int open_angle = 0;
  const int closed_angle = 180;
}
namespace wheel{
  const int current_max = 10000;
  const float straight_gain = 1.f;
  const float steering_gain = 1.f;
}

using namespace std;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);  // PCA9685のI2Cアドレスを指定
UpEdge upedge_ps;
UpEdge upedge_circle;
bool is_emergency = true;
bool is_open_arm = true;


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

  // DCモータ
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  ledcSetup(dc::pwm_channel, dc::frequency, dc::resolution);
  ledcAttachPin(ENA, dc::pwm_channel);

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
  // 変数の定義
  float wheel_gain_left = 0.f;
  float wheel_gain_right = 0.f;
  float duty_dc = 0.f;
  int arm_angle_left = 0;
  int arm_angle_right = 0;

  //遠隔緊急停止
  if (upedge_ps(ps5.PSButton())){
    is_emergency = !is_emergency;
    if(is_emergency) digitalWrite(PIN_EMERGENCY, LOW);
    else digitalWrite(PIN_EMERGENCY, HIGH);
    Serial.printf("緊急停止 : %d\n", is_emergency);
  }

  // DCモータ
  // if (ps5.R2()) {
  //   Serial.printf("R2 button at %d\n", ps5.R2Value());
  // }
  if (ps5.R2()) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    duty_dc = (float)ps5.R2Value()/255.f;
    Serial.printf("DC dut R %f\n", duty_dc);
  }
  else if (ps5.L2()) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    duty_dc = (float)ps5.L2Value()/255.f;
    Serial.printf("DC duty L %f\n", duty_dc);
  }
  else {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, HIGH);
    duty_dc = 1.f;
    Serial.printf("DC stop!\n");
  }
  ledcWrite(dc::pwm_channel, (int)(duty_dc*255));

  // アーム
  if (upedge_circle(ps5.Circle())){
    is_open_arm = !is_open_arm;
    if(is_open_arm) {arm_angle_left = servo::closed_angle; arm_angle_right = servo::open_angle;}
    else {arm_angle_left = servo::open_angle; arm_angle_right = servo::closed_angle;}

    // Serial.printf("アーム角度左 : %d\n", arm_angle_left);

    arm_angle_left = map(arm_angle_left, 0, 180, servo::servo_min, servo::servo_max);  // 角度(0~180)をパルス幅(500~2400μs)に変換
    arm_angle_right = map(arm_angle_right, 0, 180, servo::servo_min, servo::servo_max);  // 角度(0~180)をパルス幅(500~2400μs)に変換
    pwm.writeMicroseconds(servo::PIN_LEFT, arm_angle_left);
    pwm.writeMicroseconds(servo::PIN_RIGHT, arm_angle_right);
    delay(100);
  }

  //足回り
  if (abs(ps5.LStickY()) > 10) {
    wheel_gain_left = (float)ps5.LStickY() *wheel::straight_gain /127.f;
    wheel_gain_right = (float)ps5.LStickY() *wheel::straight_gain /127.f;
  }
  if (ps5.RStickX() > 10) {
    wheel_gain_left += (float)ps5.RStickX() *wheel::steering_gain /127.f;
    wheel_gain_right -= (float)ps5.RStickX() *wheel::steering_gain /127.f;
  }
  if (ps5.RStickX() < -10) {
    wheel_gain_left += (float)ps5.RStickX() *wheel::steering_gain /127.f;
    wheel_gain_right -= (float)ps5.RStickX() *wheel::steering_gain /127.f;
  }

  Serial.printf("left gain %f\n", wheel_gain_left);
  Serial.printf("right gain %f\n", wheel_gain_right);
  Serial.printf("\n");

  int current_left = wheel::current_max * constrain(wheel_gain_left, -1.f, 1.f);
  int current_right = wheel::current_max * constrain(wheel_gain_right, -1.f, 1.f);
  unsigned char data0 = (unsigned char)(current_left >> 8);
  unsigned char data1 = (unsigned char)(current_left & 0xFF);
  unsigned char data2 = (unsigned char)(current_right >> 8);
  unsigned char data3 = (unsigned char)(current_right & 0xFF);

  CAN.beginPacket(0x200);
  CAN.write(data0);
  CAN.write(data1);
  CAN.write(data2);
  CAN.write(data3);
  CAN.endPacket();

  delay(200);
}
