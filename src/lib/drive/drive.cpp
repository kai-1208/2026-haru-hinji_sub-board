#include "drive.hpp"
#include "mbed.h"
#include <algorithm>
#include <array>
#include <atomic>
#include "c610.hpp"
#include "pid.hpp"
#include "S2460.hpp"

// main.cpp のグローバルを参照するための extern 宣言
extern C610 dji_driver;
extern C610 dji_driver2; // あたらしくついかしました
extern PID omni_pid[4];
extern std::array<int16_t,4> omni_power;
extern std::atomic<int> inteldon2_target;
extern std::atomic<int> inteldon2_current;
extern std::array<int16_t,4> inteldon_data;
extern std::array<int16_t,4> inteldon_data1;
extern std::array<uint8_t, 8> servo;
extern std::atomic<int> rakupini_target;
extern std::atomic<int> rakupini_current;
extern CAN can1;
extern CAN can2;
extern const int lift_up;
extern const int lift_down;


// オムニホイールの速度制御
void update_omni_motors(float dt_omni) {
  // led1 = 1;
  for (int i = 0; i < 4; ++i) {
    omni_pid[i].set_dt(dt_omni);
    omni_pid[i].set_goal(omni_power[i]);
    int id = i + 1;
    int current_rpm = dji_driver.get_rpm(id);
    int power = static_cast<int>(omni_pid[i].do_pid(current_rpm));
    dji_driver.set_power(id, power);
  }
  // led1 = 0;
}

// CAN送信のみを行う関数（メインスレッドから30ms間隔で呼ばれる）
void send_can_messages() {
  dji_driver.send_message();
  dji_driver2.send_message();
  CANMessage msg_inteldon(1, reinterpret_cast<const uint8_t *>(inteldon_data.data()), 8);
  CANMessage msg_inteldon1(2, reinterpret_cast<const uint8_t *>(inteldon_data1.data()), 8);
  CANMessage msg_servo(140, reinterpret_cast<const uint8_t *>(servo.data()), 8);
  can1.write(msg_servo);
  can2.write(msg_inteldon);
  can2.write(msg_inteldon1);
  
}

// 上位ループから呼ばれるラッパー（CAN送信なし）
void do_drive_and_hand_step(float dt_omni) {
  update_omni_motors(dt_omni);
}