#ifndef CONTROLLER_INPUT_HPP
#define CONTROLLER_INPUT_HPP

#include "serial_manager.hpp"
#include <string>

class ControllerInput {
public:
    // ボタンの状態 (received_flags[0～16])
    bool cross = false;
    bool circle = false;
    bool triangle = false;
    bool square = false;
    bool l1 = false;
    bool r1 = false;
    bool l2 = false;
    bool r2 = false;
    bool share = false;
    bool options = false;
    bool ps = false;
    bool l3 = false;
    bool r3 = false;
    bool down = false;
    bool right = false;
    bool up = false;
    bool left = false;

    // 各ホイールの各速度 (received_nums[0～3])
    float front_left = 0.0f;
    float rear_left = 0.0f;
    float rear_right = 0.0f;
    float front_right = 0.0f;

    // SerialManagerからボタン状態を更新
    void update(SerialManager &serial);

    // デバッグ用関数
    std::string to_string() const;
    void print_debug() const;
};

#endif // CONTROLLER_INPUT_HPP