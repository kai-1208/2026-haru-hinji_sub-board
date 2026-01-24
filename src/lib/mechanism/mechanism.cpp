#include "mechanism.hpp"
#include <cstdint>
#include <array>
#include <atomic>
#include "mbed.h"

extern std::array<int16_t,4> inteldon_data;
extern std::array<int16_t,4> inteldon_data1;

void set_inteldon_slot(size_t idx, int pos_state, int neg_state, int pos_value, int neg_value, bool hold_value) {
  if (pos_state == 1) {
    inteldon_data[idx] = static_cast<int16_t>(pos_value);
  } else if (neg_state == 1) {
    inteldon_data[idx] = static_cast<int16_t>(neg_value);
  } else if (!hold_value) {
    inteldon_data[idx] = 0;
  }
}

void set_inteldon1_slot(size_t idx, int pos_state, int neg_state, int pos_value, int neg_value, bool hold_value) {
  if (pos_state == 1) {
    inteldon_data1[idx] = static_cast<int16_t>(pos_value);
  } else if (neg_state == 1) {
    inteldon_data1[idx] = static_cast<int16_t>(neg_value);
  } else if (!hold_value) {
    inteldon_data1[idx] = 0;
  }
}

void set_servo_slot(std::array<uint8_t,8> &servo, size_t idx, int pos_state, int neg_state, uint8_t pos_value, uint8_t neg_value, bool hold_value) {
  if (pos_state == 1) {
    servo[idx] = pos_value;
  } else if (neg_state == 1) {
    servo[idx] = neg_value;
  } else if (!hold_value) {
    servo[idx] = 0;
  }
}

void set_atomic_target(std::atomic<int> &target, int pos_state, int neg_state, int pos_value, int neg_value) {
  if (pos_state == 1) {
    target.store(pos_value);
  } else if (neg_state == 1) {
    target.store(neg_value);
  } else {
    target.store(0);
  }
}