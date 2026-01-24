#pragma once

#include <cstddef>
#include <cstdint>
#include <array>
#include <atomic>

void set_inteldon_slot(size_t idx, int pos_state, int neg_state, int pos_value, int neg_value, bool hold_value = false);
void set_inteldon1_slot(size_t idx, int pos_state, int neg_state, int pos_value, int neg_value, bool hold_value = false);
void set_servo_slot(std::array<uint8_t,8> &servo, size_t idx, int pos_state, int neg_state, uint8_t pos_value, uint8_t neg_value, bool hold_value = false);
void set_atomic_target(std::atomic<int> &target, int pos_state, int neg_state, int pos_value, int neg_value);