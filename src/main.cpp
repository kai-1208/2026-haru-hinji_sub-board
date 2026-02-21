#include "c610.hpp"
#include "controller_input.hpp"
#include "led_controller.hpp"
#include "mbed.h"
#include "pid.hpp"
#include "serial_manager.hpp"

#define WITH_SERIAL_MANAGER

#define MECHANISM_LOOP_INTERVAL 0.01f  // 100Hz
#define LED_LOOP_INTERVAL 0.05f        // 10Hz
#define MAIN_LOOP_INTERVAL 0.001f      // 1000Hz

// 定数定義
const int BRUSHLESS_POWER = 3000;
const int SERVO_POS_LOW = 80;
const int SERVO_POS_HIGH = 20;

BufferedSerial pc (USBTX, USBRX, 115200);
SerialManager serial (pc, 2, LED1, BUTTON1);  // ID:2 サブNucleo

// CAN割り当て
CAN can1 (PA_11, PA_12, (int)1e6);  // やぐらあーむ用あぶそ
CAN can2 (PB_12, PB_13, (int)1e6);  // いなばうわあ用
C610 mech_brushless (can2);         // いなばうわあ
PidParameter param = {
    .gain = {.kp = 2.0f, .ki = 0.0f, .kd = 0.0f},
      .min = -5000.0f, .max = 5000.0f
};
Pid inaba_pid (param);
CANMessage msg1;

ControllerInput input;
LedController Led (PA_9, PA_10);

int16_t inaba_power[3] = {0};        // 0: 右, 1: 左, 2: 中央
std::array<uint8_t, 8> servo = {0};  // 0: 右, 1: 中央, 2: 左

bool servo_state[3] = {false};
bool all_servo_state = false;

DigitalIn emergency_sw (PC_13), limit_sw1 (PC_10), limit_sw2 (PC_11), limit_sw3 (PC_12), limit_sw4 (PC_0), limit_sw5 (PC_1), limit_sw6 (PC_2), limit_laser1 (PC_5), limit_laser2 (PC_6), limit_laser3 (PC_7);

LedState prev_state = LedState::Unknown;
LedState curr_state = LedState::Normal;
mbed::HighResClock::time_point last_can1_time = HighResClock::now ();
mbed::HighResClock::time_point last_can2_time = HighResClock::now ();

bool can1_timeout = false;
bool can2_timeout = false;

std::array<float, 4> enc_vals = {0.0f, 0.0f, 0.0f, 0.0f};

/**
 * @brief 機構制御
 */
void mechanism_control_thread () {
  if (serial.is_connected ()) {
    input.update (serial);
    // いなばうわあ
    inaba_power[0] = input.cross ? BRUSHLESS_POWER : (input.triangle ? -BRUSHLESS_POWER : 0);
    inaba_power[1] = input.up ? BRUSHLESS_POWER : (input.down ? -BRUSHLESS_POWER : 0);
    inaba_power[2] = input.r1 ? BRUSHLESS_POWER : (input.l1 ? -BRUSHLESS_POWER : 0);

    if (limit_sw1.read () == 0 && inaba_power[0] < 0) inaba_power[0] = 0;
    if (limit_sw2.read () == 0 && inaba_power[1] < 0) inaba_power[1] = 0;
    if (limit_sw3.read () == 0 && inaba_power[2] < 0) inaba_power[2] = 0;
    if (limit_sw4.read () == 0 && inaba_power[0] > 0) inaba_power[0] = 0;
    if (limit_sw5.read () == 0 && inaba_power[1] > 0) inaba_power[1] = 0;
    if (limit_sw6.read () == 0 && inaba_power[2] > 0) inaba_power[2] = 0;

    inaba_power[2] = -inaba_power[2];  // モーターの向きに合わせて反転 //条件の整合性のため、最後に反転させる

    for (int i = 0; i < 3; i++) {
      mech_brushless.set_power (i + 1, inaba_pid.calc (inaba_power[i], mech_brushless.get_rpm (i + 1), MECHANISM_LOOP_INTERVAL));
    }

    // やぐらあーむ

    // 以下センサーによる自動制御
    static bool pre_laser[3] = {false, false, false};
    bool curr_laser[3] = {limit_laser1.read () == 0, limit_laser2.read () == 0, limit_laser3.read () == 0};
    for (int i = 0; i < 3; i++) {
      if (curr_laser[i] && !pre_laser[i]) {
        servo_state[i] = false;
      }
      pre_laser[i] = curr_laser[i];
    }

    static bool pre_ps = false;
    static bool pre_r2 = false;
    static bool pre_options = false;
    static bool pre_l2 = false;
    if (input.ps && !pre_ps) {
      all_servo_state = !all_servo_state;
      for (int i = 0; i < 3; i++) {
        servo_state[i] = all_servo_state;
      }
    }
    pre_ps = input.ps;
    if (input.r2 && !pre_r2) servo_state[0] = !servo_state[0];
    pre_r2 = input.r2;
    if (input.options && !pre_options) servo_state[1] = !servo_state[1];
    pre_options = input.options;
    if (input.l2 && !pre_l2) servo_state[2] = !servo_state[2];

    for (int i = 0; i < 3; i++) {
      servo[i] = servo_state[i] ? SERVO_POS_HIGH : SERVO_POS_LOW;
    }
    pre_l2 = input.l2;
  } else {
    // 通信切断時は停止
    for (int i = 1; i < 4; i++) mech_brushless.set_power (i, 0);
    // サーボも初期位置に
    for (int i = 0; i < 3; i++) servo[i] = SERVO_POS_LOW;
  }
}

/**
 * @brief ネオピクセル光らせます
 */
void led_state_thread () {
  // can死んでないか確認
  CANMessage msg;
  if (can1.read (msg)) last_can1_time = HighResClock::now ();
  if (can2.read (msg)) last_can2_time = HighResClock::now ();

  auto now = HighResClock::now ();
  can1_timeout = std::chrono::duration<float> (now - last_can1_time).count () > 0.05f;
  can2_timeout = std::chrono::duration<float> (now - last_can2_time).count () > 0.05f;

  if (emergency_sw.read () == 0) {
    curr_state = LedState::OFF;
  } else if (can1_timeout || can2_timeout) {
    curr_state = LedState::CommLost;
  } else {
    curr_state = LedState::Normal;
  }
  Led.sendLedState (curr_state);
}

int main () {
  pc.set_blocking (true);
  pc.set_baud (115200);
  Led.setup ();
  emergency_sw.mode (PullUp);
  limit_sw1.mode (PullUp);
  limit_sw2.mode (PullUp);
  limit_sw3.mode (PullUp);
  limit_sw4.mode (PullUp);
  limit_sw5.mode (PullUp);
  limit_sw6.mode (PullUp);
  limit_laser1.mode (PullUp);
  limit_laser2.mode (PullUp);
  limit_laser3.mode (PullUp);

  // スレッド起動
  // Thread mech_thread;
  // mech_thread.start (mechanism_control_thread);

  // Thread led_thread;
  // led_thread.start (led_state_thread);

  serial.send_log ("Sub Nucleo Started");

  mbed::HighResClock::time_point now_timestamp = HighResClock::now ();
  mbed::HighResClock::time_point main_loop_timestamp = HighResClock::now ();
  mbed::HighResClock::time_point mechanism_loop_timestamp = HighResClock::now ();
  mbed::HighResClock::time_point led_loop_timestamp = HighResClock::now ();

  while (1) {
    now_timestamp = HighResClock::now ();
    std::chrono::duration<float> elapsed_seconds = now_timestamp - main_loop_timestamp;
    float main_loop_dt = elapsed_seconds.count ();
    float mechanism_loop_dt = std::chrono::duration<float> (now_timestamp - mechanism_loop_timestamp).count ();
    float led_loop_dt = std::chrono::duration<float> (now_timestamp - led_loop_timestamp).count ();

    if (mechanism_loop_dt > MECHANISM_LOOP_INTERVAL) {  // 100Hzの機構制御ループ
      mech_brushless.param_update ();                   // パラメータ更新

      mechanism_control_thread ();
      mechanism_loop_timestamp = now_timestamp;

      mech_brushless.send_message ();
      CANMessage msg1 (140, reinterpret_cast<const uint8_t *> (servo.data ()), 8);
      can1.write (msg1);
    }

    if (led_loop_dt > LED_LOOP_INTERVAL) {
      led_state_thread ();
      led_loop_timestamp = now_timestamp;
    }

    if (main_loop_dt > MAIN_LOOP_INTERVAL) {  // 1000Hzのメインループ
      main_loop_timestamp = now_timestamp;

#ifndef WITH_SERIAL_MANAGER
      if (limit_sw1.read () == 0) printf ("1");
      if (limit_sw2.read () == 0) printf ("2      ");
      if (limit_sw3.read () == 0) printf ("3");
      if (limit_sw4.read () == 0) printf ("4");
      if (limit_sw5.read () == 0) printf ("5");
      if (limit_sw6.read () == 0) printf ("6");
      if (limit_laser1.read () == 0) printf ("7");  // 左 pc 5
      if (limit_laser2.read () == 0) printf ("8");  // 真ん中 pc 6
      if (limit_laser3.read () == 0) printf ("9");  // 右 pc 7

      if (emergency_sw.read () == 0) printf ("きんきゅううううう");
      printf ("\n");
#else
      static int log_counter = 0;
      log_counter++;
      if (serial.is_connected () && log_counter >= 100) {  // デバッグ用ログ送信
        std::string log_msg = "";
        log_msg += input.circle ? "○ " : "";
        log_msg += input.cross ? "× " : "";
        log_msg += input.triangle ? "△ " : "";
        log_msg += input.square ? "□ " : "";
        log_msg += input.l1 ? "L1 " : "";
        log_msg += input.r1 ? "R1 " : "";
        log_msg += input.l2 ? "L2 " : "";
        log_msg += input.r2 ? "R2 " : "";
        log_msg += input.share ? "Share " : "";
        log_msg += input.options ? "Options " : "";
        log_msg += input.ps ? "PS " : "";
        log_msg += input.l3 ? "L3 " : "";
        log_msg += input.r3 ? "R3 " : "";
        log_msg += input.down ? "Down " : "";
        log_msg += input.right ? "Right " : "";
        log_msg += input.up ? "Up " : "";
        log_msg += input.left ? "Left " : "";

        if (can1_timeout) log_msg += "CAN1_TIMEOUT ";
        if (can2_timeout) log_msg += "CAN2_TIMEOUT ";

        serial.send_log (log_msg);
        log_counter = 0;
      }
#endif
    }
  }
}