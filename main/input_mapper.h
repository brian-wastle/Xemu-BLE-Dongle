#pragma once

#include "usb_input_host.h"
#include "hid_gamepad.h"

// Initialize mapper task that converts raw USB frames to gamepad_state_t
void input_mapper_init(void);
