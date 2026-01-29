#include "mbed.h"
#include "serial_manager.hpp"
#include "c610.hpp"
#include "controller_input.hpp"
#include "led_controller.hpp"

std::array<uint8_t, 8> servo = {0}; // 0: 右, 1: 中央, 2: 左

// 定数定義
const int BRUSHLESS_POWER = 5000;
const int SERVO_POS_LOW = 0;
const int SERVO_POS_HIGH = 200;

BufferedSerial pc(USBTX, USBRX, 115200);
SerialManager serial(pc, LED1, BUTTON1); // ID:2 サブNucleo

// CAN割り当て
CAN can1(PA_11, PA_12, (int)1e6); // やぐらあーむ用
CAN can2(PB_12, PB_13, (int)1e6); // いなばうわあ用

C610 mech_brushless(can2); // いなばうわあ
ControllerInput input;
LedController Led(PA_9, PA_10);

int16_t inaba_power[3] = {0}; // 0: 右, 1: 左, 2: 中央

bool servo_state[3] = {false};
bool all_servo_state = false;

DigitalIn emergency_sw(PC_12), limit_sw1(PC_1),
    limit_sw2(PC_2), limit_sw3(PC_3),
    limit_sw4(PC_4), limit_sw5(PC_10),
    limit_sw6(PC_11), limit_laser1(PC_13),
    limit_laser2(PC_14), limit_laser3(PC_15);

LedState prev_state = LedState::Unknown;
LedState curr_state = LedState::Normal;

float dt = 0.0;

/**
 * @brief 機構制御
 */
void mechanism_control_thread()
{
    while (1)
    {
        if (serial.is_connected())
        {
            input.update(serial.received_nums, serial.received_flags);
            // input.print_debug(); // デバッグ表示

            // いなばうわあ
            inaba_power[0] = input.cross ? BRUSHLESS_POWER : (input.triangle ? -BRUSHLESS_POWER : 0);
            inaba_power[1] = input.up ? BRUSHLESS_POWER : (input.down ? -BRUSHLESS_POWER : 0);
            inaba_power[2] = input.r1 ? BRUSHLESS_POWER : (input.l1 ? -BRUSHLESS_POWER : 0);
            if (limit_sw1.read() == 1 && inaba_power[0] < 0)
                inaba_power[0] = 0;
            if (limit_sw2.read() == 1 && inaba_power[1] < 0)
                inaba_power[1] = 0;
            if (limit_sw3.read() == 1 && inaba_power[2] < 0)
                inaba_power[2] = 0;
            if (limit_sw4.read() == 1 && inaba_power[0] > 0)
                inaba_power[0] = 0;
            if (limit_sw5.read() == 1 && inaba_power[1] > 0)
                inaba_power[1] = 0;
            if (limit_sw6.read() == 1 && inaba_power[2] > 0)
                inaba_power[2] = 0;
            for (int i = 0; i < 3; i++)
            {
                mech_brushless.set_power(i + 1, inaba_power[i]);
            }

            // やぐらあーむ
            if (input.ps)
            {
                all_servo_state = !all_servo_state;
                for (int i = 0; i < 3; i++)
                    servo[i] = all_servo_state ? SERVO_POS_HIGH : SERVO_POS_LOW;
            }
            else if (input.r2)
            {
                servo_state[0] = !servo_state[0];
                servo[0] = servo_state[0] ? SERVO_POS_HIGH : SERVO_POS_LOW;
            }
            else if (input.options)
            {
                servo_state[1] = !servo_state[1];
                servo[1] = servo_state[1] ? SERVO_POS_HIGH : SERVO_POS_LOW;
            }
            else if (input.l2)
            {
                servo_state[2] = !servo_state[2];
                servo[2] = servo_state[2] ? SERVO_POS_HIGH : SERVO_POS_LOW;
            }

            // 以下センサーによる自動制御
            if (limit_laser1.read() == 1)
            {
                servo[0] = SERVO_POS_HIGH;
            }
            else if (limit_laser2.read() == 1)
            {
                servo[1] = SERVO_POS_HIGH;
            }
            else if (limit_laser3.read() == 1)
            {
                servo[2] = SERVO_POS_HIGH;
            }
        }
        else
        {
            // 通信切断時は停止
            for (int i = 1; i < 4; i++)
                mech_brushless.set_power(i, 0);
            // サーボも初期位置に
            for (int i = 0; i < 3; i++)
                servo[i] = SERVO_POS_LOW;
        }
        ThisThread::sleep_for(15ms);
    }
}

/**
 * @brief ネオピクセル光らせます
 */
void led_state_thread()
{
    while (1)
    {
        if (emergency_sw.read() == 1)
        {
            curr_state = LedState::OFF;
        }
        else
        {
            curr_state = LedState::Normal;
        }
        Led.sendLedState(curr_state);

        ThisThread::sleep_for(50ms);
    }
}

void move(std::vector<float> wheel_speeds)
{ // rad/s
    if (wheel_speeds.size() == 4)
    {
        std::string log_msg = "";
        for (size_t i = 0; i < wheel_speeds.size(); i++)
        {
            float power = wheel_speeds[i] * 30.0 / M_PI;
            log_msg += std::to_string(int(power)) + " ";
        }
        serial.send_log(log_msg);
    }
}

int main()
{
    pc.set_blocking(true);
    pc.set_baud(115200);
    Led.setup();
    emergency_sw.mode(PullUp);
    limit_sw1.mode(PullUp);
    limit_sw2.mode(PullUp);
    limit_sw3.mode(PullUp);
    limit_sw4.mode(PullUp);
    limit_sw5.mode(PullUp);
    limit_sw6.mode(PullUp);
    limit_laser1.mode(PullUp);
    limit_laser2.mode(PullUp);
    limit_laser3.mode(PullUp);

    // スレッド起動
    Thread mech_thread;
    mech_thread.start(mechanism_control_thread);

    Thread led_thread;
    led_thread.start(led_state_thread);

    Timer can_send_timer;
    can_send_timer.start();

    auto start = HighResClock::now();
    auto end = HighResClock::now();

    while (1)
    {
        end = HighResClock::now();
        std::chrono::duration<float> elapsed_seconds = end - start;
        dt = elapsed_seconds.count();

        if (std::chrono::duration<float>(dt) > 10ms)
        {
            start = HighResClock::now();
            move(serial.received_nums);
            // can送信
            // if (can_send_timer.elapsed_time().count() / 1000 >= 20) {
            //     can_send_timer.reset();
            //     mech_brushless.send_message();
            //     CANMessage msg1(140, reinterpret_cast<const uint8_t *>(servo.data()), 8);
            //     can1.write(msg1);
            // }

            // pcへ現在の状態を送信
            // if (serial.is_connected() && can_send_timer.elapsed_time().count() / 1000 >= 100) {
            //     std::vector<float> feedback_data = {
            //         (float)mech_brushless.get_rpm(1),
            //         (float)mech_brushless.get_rpm(2)
            //     };
            //     serial.send_msg(feedback_data);
            // }
        }
        // ThisThread::sleep_for(1ms);
    }
}