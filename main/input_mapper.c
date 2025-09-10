#include "input_mapper.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

static const char *TAG_MAP = "INPUT_MAP";

static void input_mapper_task(void *arg)
{
    ESP_LOGI(TAG_MAP, "Input mapper task running (stub)");
    gip_frame_t frame;
    while (1) {
        if (g_gip_out_queue && xQueueReceive(g_gip_out_queue, &frame, pdMS_TO_TICKS(1000))) {
            // TODO: Parse GIP frame into a gamepad_state_t
            gamepad_state_t st = {0};
            // Placeholder: neutral sticks, no buttons
            (void)frame;
            hid_gamepad_send_state(&st);
        }
    }
}

void input_mapper_init(void)
{
    xTaskCreate(input_mapper_task, "input_mapper_task", 4096, NULL, 5, NULL);
}

