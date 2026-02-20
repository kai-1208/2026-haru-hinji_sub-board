#include "controller_input.hpp"

#include <cstdio>

void ControllerInput::update (SerialManager &serial) {
  // ボタン状態 (received_flags[0～16])
  if (serial.received_flags.size () >= 17) {
    cross = serial.received_flags[0];
    circle = serial.received_flags[1];
    triangle = serial.received_flags[2];
    square = serial.received_flags[3];
    l1 = serial.received_flags[4];
    r1 = serial.received_flags[5];
    l2 = serial.received_flags[6];
    r2 = serial.received_flags[7];
    share = serial.received_flags[8];
    options = serial.received_flags[9];
    ps = serial.received_flags[10];
    l3 = serial.received_flags[11];
    r3 = serial.received_flags[12];
    down = serial.received_flags[13];
    right = serial.received_flags[14];
    up = serial.received_flags[15];
    left = serial.received_flags[16];
  }

  // 各ホイールの角速度 (received_nums[0～3])
  if (serial.received_nums.size () >= 4) {
    front_left = serial.received_nums[0];
    rear_left = serial.received_nums[1];
    rear_right = serial.received_nums[2];
    front_right = serial.received_nums[3];
  }
}

std::string ControllerInput::to_string () const {
  char buffer[512];
  snprintf (
      buffer, sizeof (buffer),
      "Buttons: ×=%d ○=%d △=%d □=%d L1=%d R1=%d L2=%d R2=%d "
      "Share=%d Opt=%d PS=%d L3=%d R3=%d ↓=%d →=%d ↑=%d ←=%d | "
      "Wheels: FL=%.2f RL=%.2f RR=%.2f FR=%.2f",
      cross, circle, triangle, square, l1, r1, l2, r2, share, options, ps, l3, r3, down, right, up, left, front_left, rear_left, rear_right, front_right);
  return std::string (buffer);
}