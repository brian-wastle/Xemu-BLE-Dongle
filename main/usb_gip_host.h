#pragma once

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// Raw GIP frame as captured from the Hyperkin DuchesS over USB
typedef struct {
    uint8_t data[64];   // placeholder size; actual may vary per transfer
    uint16_t len;       // valid bytes in data
} gip_frame_t;

// Queue where parsed/raw frames will be published
extern QueueHandle_t g_gip_out_queue;

// Initialize USB host stack and create task to read controller input (stub for now)
void usb_gip_host_init(void);

