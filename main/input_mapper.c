#include "input_mapper.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// An real-time operating system (RTOS) task sits on a loop reading GIP (gamepad) frames from a queue. 
// When data arrives, a gamepad state is built and sent to the BLE side 
// last_report handles shorter "event" frames for when the Guide menu button is pressed

static inline int16_t read_le_i16(const uint8_t *p)
{
    return (int16_t)((int16_t)p[0] | ((int16_t)p[1] << 8));
}

static inline int8_t scale16_to_s8(int16_t v)
{
    int32_t s = v / 256; // approximate 16->8 with clamp
    if (s > 127)
        s = 127;
    if (s < -127)
        s = -127;
    return (int8_t)s;
}

// Keep the last sent HID state so we can emit immediate updates for event-only frames
static gamepad_state_t last_report = {0};

static void input_mapper_task(void *arg)
{
    gip_frame_t frame;
    static uint8_t guide_down = 0;           // tracked from 0x07 frames
    static TickType_t guide_latch_until = 0; // ensure at least one state frame reports guide
    while (1)
    {
        if (g_gip_out_queue && xQueueReceive(g_gip_out_queue, &frame, pdMS_TO_TICKS(1000)))
        {
            // Handle short event frames (e.g., Guide) on channel 0x20 with type 0x07
            if (frame.len >= 6 && frame.data[0] == 0x07 && frame.data[1] == 0x20)
            {
                // Observed format: 07 20 <seq> 02 <val> 5B  where val=01 press, 00 release, 5B=GUIDE code
                if (frame.data[3] == 0x02 && frame.data[5] == 0x5B)
                {
                    uint8_t pressed = (frame.data[4] != 0);
                    guide_down = pressed;
                    if (pressed)
                    {
                        // Latch for ~60ms so a very quick tap still registers in next 0x20 state frame
                        guide_latch_until = xTaskGetTickCount() + pdMS_TO_TICKS(60);
                    }
                    // Emit an immediate HID update based on last state so Guide always registers
                    if (pressed)
                        last_report.buttons |= (1u << 8);
                    else
                        last_report.buttons &= ~(1u << 8);
                    (void)hid_gamepad_send_state(&last_report);
                }
                continue;
            }

            // Only parse dev->host state frames on channel 0x20
            if (frame.len < 20 || frame.data[0] != 0x20)
                continue;

            // Buttons mapping from bytes 4 and 5
            // Byte 4 bits: 0x10 A, 0x20 B, 0x40 X, 0x80 Y, 0x08 Back/View, 0x04 Start/Menu
            // Byte 5 bits: 0x01 DpadUp, 0x02 DpadDown, 0x04 DpadLeft, 0x08 DpadRight,
            //              0x10 LB, 0x20 RB, 0x40 LS click, 0x80 RS click
            uint8_t btn_byte4 = frame.data[4];
            uint8_t btn_byte5 = frame.data[5];
            uint16_t buttons = 0;
            // Button indices (match SDL built-in Xbox mapping):
            // 0 A,1 B,2 X,3 Y,4 LB,5 RB,6 Back(View),7 Start(Menu),8 Guide,9 LS,10 RS
            if (btn_byte4 & 0x10)
                buttons |= (1u << 0); // A
            if (btn_byte4 & 0x20)
                buttons |= (1u << 1); // B
            if (btn_byte4 & 0x40)
                buttons |= (1u << 2); // X
            if (btn_byte4 & 0x80)
                buttons |= (1u << 3); // Y
            if (btn_byte5 & 0x10)
                buttons |= (1u << 4); // LB
            if (btn_byte5 & 0x20)
                buttons |= (1u << 5); // RB
            if (btn_byte4 & 0x08)
                buttons |= (1u << 6); // Back/View
            if (btn_byte4 & 0x04)
                buttons |= (1u << 7); // Start/Menu
            // Guide/L3/R3 order per common SDL mapping
            // Consider a brief latch window to avoid missing very quick taps
            TickType_t tick_now = xTaskGetTickCount();
            if (guide_down || (int32_t)(guide_latch_until - tick_now) > 0)
                buttons |= (1u << 8); // Guide
            if (btn_byte5 & 0x40)
                buttons |= (1u << 9); // LS click
            if (btn_byte5 & 0x80)
                buttons |= (1u << 10); // RS click
            // Do not mirror D-pad to button bits; use hat only to avoid SDL fallback mis-maps

            // Sticks (confirmed from logs):
            //   [10..11]=LSX, [12..13]=LSY, [14..15]=RSX, [16..17]=RSY
            int16_t ls_x_raw = read_le_i16(&frame.data[10]);
            int16_t ls_y_raw = read_le_i16(&frame.data[12]);
            int16_t rs_x_raw = read_le_i16(&frame.data[14]);
            int16_t rs_y_raw = read_le_i16(&frame.data[16]);

            // Triggers: uint10 values at [6..7]=LT, [8..9]=RT (0..1023)
            uint16_t lt_raw = (uint16_t)read_le_i16(&frame.data[6]);
            uint16_t rt_raw = (uint16_t)read_le_i16(&frame.data[8]);
            if (lt_raw < 8)
                lt_raw = 0; // small noise guard
            if (rt_raw < 8)
                rt_raw = 0;

            gamepad_state_t state = (gamepad_state_t){0};
            state.buttons = buttons;
            // Hat switch from D-pad bits (0..7; 0x0F neutral)
            bool d_up = (btn_byte5 & 0x01) != 0;
            bool d_dn = (btn_byte5 & 0x02) != 0;
            bool d_lt = (btn_byte5 & 0x04) != 0;
            bool d_rt = (btn_byte5 & 0x08) != 0;
            uint8_t hat = 0x0F; // neutral
            if (!(d_up && d_dn) && !(d_lt && d_rt))
            {
                int h = -1;
                if (d_up)
                    h = 0;
                else if (d_dn)
                    h = 4; // base
                if (d_rt)
                    h = (h < 0) ? 2 : (h + 1) % 8;
                if (d_lt)
                    h = (h < 0) ? 6 : (h + 7) % 8;
                if (h >= 0)
                    hat = (uint8_t)h;
            }
            state.hat = hat;
            state.x = scale16_to_s8(ls_x_raw);
            state.y = (int8_t)-scale16_to_s8(ls_y_raw);
            state.rx = scale16_to_s8(rs_x_raw);
            state.ry = (int8_t)-scale16_to_s8(rs_y_raw);

            // Scale triggers to 8-bit (0..255)
            state.z = (uint8_t)(lt_raw >> 2);
            state.rz = (uint8_t)(rt_raw >> 2);

            // TODO: Share button not mapped

            (void)hid_gamepad_send_state(&state);
            last_report = state; // remember for event-only frames
        }
    }
}

void input_mapper_init(void)
{
    xTaskCreate(input_mapper_task, "input_mapper_task", 4096, NULL, 5, NULL);
}
