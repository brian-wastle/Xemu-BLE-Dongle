#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_hidd.h"

// Simple gamepad state used across the app
typedef struct {
    uint16_t buttons;   // 16 buttons (bitmask)
    int8_t x;           // -127..127
    int8_t y;           // -127..127
    int8_t rx;          // -127..127 (right stick X)
    int8_t ry;          // -127..127 (right stick Y)
} gamepad_state_t;

// Initialize BLE HID device with a gamepad report map.
// Returns ESP_OK on success.
esp_err_t hid_gamepad_init(esp_hidd_dev_t **out_dev);

// Send a gamepad input report based on the provided state.
esp_err_t hid_gamepad_send_state(const gamepad_state_t *state);

