#include "input_mapper.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// Example packet that a custom USB HID gamepad might send: buttons (16 bits),
// hat (0..7, 0x0F neutral), 4 signed axes, and 2 analog triggers.
typedef struct __attribute__((packed))
{
    uint16_t buttons;
    uint8_t hat;
    int8_t lx;
    int8_t ly;
    int8_t rx;
    int8_t ry;
    uint8_t lt;
    uint8_t rt;
} simple_hid_gamepad_report_t;

static void input_mapper_task(void *arg)
{
    usb_input_frame_t frame;
    while (1)
    {
        if (g_usb_input_queue && xQueueReceive(g_usb_input_queue, &frame, pdMS_TO_TICKS(1000)))
        {
            if (frame.len < sizeof(simple_hid_gamepad_report_t))
            {
                continue;
            }

            const simple_hid_gamepad_report_t *report = (const simple_hid_gamepad_report_t *)frame.data;
            gamepad_state_t state = {0};
            state.buttons = report->buttons;
            state.hat = report->hat;
            state.x = report->lx;
            state.y = (int8_t)-report->ly; // invert to match HID convention
            state.rx = report->rx;
            state.ry = (int8_t)-report->ry;
            state.z = report->lt;
            state.rz = report->rt;

            (void)hid_gamepad_send_state(&state);
        }
    }
}

void input_mapper_init(void)
{
    xTaskCreate(input_mapper_task, "input_mapper_task", 4096, NULL, 5, NULL);
}
