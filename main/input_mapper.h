#pragma once

#include "usb_gip_host.h"
#include "hid_gamepad.h"

// Initialize mapper task that converts GIP frames to gamepad_state_t
void input_mapper_init(void);
