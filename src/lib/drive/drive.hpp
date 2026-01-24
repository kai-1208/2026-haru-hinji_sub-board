#pragma once

#include <cstdint>

void update_omni_motors(float dt_omni);
void update_hand_motor(float dt_omni);
void send_can_messages();
void do_drive_and_hand_step(float dt_omni);
void update_rakupini();