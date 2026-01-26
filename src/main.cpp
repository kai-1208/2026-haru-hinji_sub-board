#include "mbed.h"
#include "serial_manager.hpp"
#include "c610.hpp"
#include "controller_input.hpp"

std::array<uint8_t, 8> servo = {0}; // 0: 右, 1: 中央, 2: 左

// 定数定義
const int brushless_power = 5000;
const int servo_pos_low = 0;
const int servo_pos_high = 200;

BufferedSerial pc(USBTX, USBRX, 115200);
SerialManager serial(pc, 2, LED1, BUTTON1); // ID:2 サブNucleo

// CAN割り当て
CAN can1(PA_11, PA_12, (int)1e6); // やぐらあーむ用
CAN can2(PB_12, PB_13, (int)1e6); // いなばうわあ用

C610 mech_brushless(can2); // いなばうわあ
ControllerInput input;

int16_t inaba_power[3] = {0}; // 0: 右, 1: 左, 2: 中央

bool servo_state[3] = {false};
bool all_servo_state = false;

/**
 * @brief 機構制御
 */
void mechanism_control_thread() {
    while (1) {
        if (serial.is_connected()) {
            input.update(serial);
            input.print_debug(); // デバッグ表示

            // いなばうわあ
            if (input.cross) inaba_power[0] = brushless_power;
            else if (input.triangle) inaba_power[0] = -brushless_power;
            if (input.up) inaba_power[1] = brushless_power;
            else if (input.down) inaba_power[1] = -brushless_power;
            if (input.r1) inaba_power[2] = brushless_power;
            else if (input.l1) inaba_power[2] = -brushless_power;
            else for (int i = 0; i < 3; i++) {
                inaba_power[i] = 0;
            }

            for (int i = 0; i < 3; i++) {
                mech_brushless.set_power(i+1, inaba_power[i]);
            }

            // やぐらあーむ

            if (input.ps) {
                all_servo_state = !all_servo_state;
                for (int i = 0; i < 3; i++) servo[i] = all_servo_state ? servo_pos_high : servo_pos_low;
            } else if (input.r2) {
                servo_state[0] = !servo_state[0];
                servo[0] = servo_state[0] ? servo_pos_high : servo_pos_low;
            } else if (input.options) {
                servo_state[1] = !servo_state[1];
                servo[1] = servo_state[1] ? servo_pos_high : servo_pos_low;
            } else if (input.l2) {
                servo_state[2] = !servo_state[2];
                servo[2] = servo_state[2] ? servo_pos_high : servo_pos_low;
            }

            // if (serial.received_nums.size() >= 3) {
            //     for (int i = 0; i < 3; i++) {
            //         float target_power = serial.received_nums[i];
            //         if (target_power > brushless_max_power) target_power = brushless_max_power;
            //         if (target_power < -brushless_max_power) target_power = -brushless_max_power;
                    
            //         mech_brushless.set_power(i + 1, (int16_t)target_power);
            //     }
            // }

            // // フラグがtrueなら200, falseなら0
            // if (serial.received_flags.size() >= 3) {
            //     for (int i = 0; i < 3; i++) {
            //         servo[i] = serial.received_flags[i] ? servo_pos_high : servo_pos_low;
            //     }
            // }
        } else {
            // 通信切断時は停止
            for (int i = 1; i < 4; i++) mech_brushless.set_power(i, 0);
            // サーボも初期位置に
            for (int i = 0; i < 3; i++) servo[i] = servo_pos_low;
        }

        ThisThread::sleep_for(15ms);
    }
}

int main() {
    // スレッド起動
    Thread mech_thread;
    mech_thread.start(mechanism_control_thread);

    Timer can_timer;
    can_timer.start();

    while (1) {
        // can送信
        if (can_timer.elapsed_time().count() / 1000 >= 30) {
            can_timer.reset();
            mech_brushless.send_message();
            CANMessage msg1(140, reinterpret_cast<const uint8_t *>(servo.data()), 8);
            can1.write(msg1);
        }

        // 定期的にPCへ状態（モーターのRPMなど）を報告
        // if (serial.is_connected() && can_timer.elapsed_time().count() / 1000 >= 100) {
        //     serial.send_log("Sub-Nucleo Operating");
        //     std::vector<float> feedback_data = {
        //         (float)mech_brushless.get_rpm(1), 
        //         (float)mech_brushless.get_rpm(2)
        //     };
        //     serial.send_msg(feedback_data);
        // }

        ThisThread::sleep_for(1ms);
    }
}