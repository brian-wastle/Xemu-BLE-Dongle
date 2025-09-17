#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_hidd.h"

// Gamepad state
typedef struct
{
    uint16_t buttons; // 16 buttons (bitmask)
    uint8_t hat;      // 0..7 (N,NE,E,SE,S,SW,W,NW) or 0x0F for neutral
    int8_t x;         // -127..127
    int8_t y;         // -127..127
    int8_t rx;        // -127..127 (right stick X)
    int8_t ry;        // -127..127 (right stick Y)
    uint8_t z;        // 0..255 (left trigger)
    uint8_t rz;       // 0..255 (right trigger)
} gamepad_state_t;

// Initialize BLE HID device with a gamepad report map.
// Returns ESP_OK on success.
esp_err_t hid_gamepad_init(esp_hidd_dev_t **out_dev);

// Send a gamepad input report based on the provided state.
esp_err_t hid_gamepad_send_state(const gamepad_state_t *state);
