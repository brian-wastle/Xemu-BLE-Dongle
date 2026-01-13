#pragma once

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// Generic USB interrupt transfer payload captured from any HID-style gamepad.
typedef struct {
    uint8_t data[64];   // placeholder size; actual may vary per transfer
    uint16_t len;       // valid bytes in data
} usb_input_frame_t;

// Queue where parsed/raw frames will be published
extern QueueHandle_t g_usb_input_queue;

// Initialize USB host stack and create task to read controller input (stub for now)
void usb_input_host_init(void);
