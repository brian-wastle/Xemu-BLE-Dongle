#include "usb_gip_host.h"
#include "esp_log.h"

QueueHandle_t g_gip_out_queue = NULL;

static const char *TAG_GIP = "USB_GIP";

static void usb_gip_task(void *arg)
{
    ESP_LOGI(TAG_GIP, "USB GIP host task running (stub)");
    // TODO: Initialize TinyUSB/USB Host and claim the Hyperkin DuchesS interface.
    // For now, this task periodically logs a placeholder.
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void usb_gip_host_init(void)
{
    if (!g_gip_out_queue) {
        g_gip_out_queue = xQueueCreate(8, sizeof(gip_frame_t));
    }
    xTaskCreate(usb_gip_task, "usb_gip_task", 4096, NULL, 5, NULL);
}

