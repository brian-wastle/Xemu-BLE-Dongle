#include "power_manager.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "driver/gpio.h"

static const char *TAG_PWR = "POWER";
static power_status_t s_status = {0};

static void power_task(void *arg)
{
    ESP_LOGI(TAG_PWR, "Power manager task running (stub)");
    while (1) {
        // TODO: If wired, sample battery ADC, read charger status pins,
        // and toggle 5V enable for USB host as needed.
        // For now just keep 5V off and report 0 mV.
        s_status.usb_host_5v_on = false;
        s_status.batt_mv = 0;
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void power_manager_init(void)
{
    // Optional: configure 5V enable GPIO for USB host, default off
#if (CONFIG_XEMUBOX_USB_5V_EN_GPIO >= 0)
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << CONFIG_XEMUBOX_USB_5V_EN_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    gpio_set_level(CONFIG_XEMUBOX_USB_5V_EN_GPIO, 0);
#endif
    xTaskCreate(power_task, "power_task", 4096, NULL, 4, NULL);
}

power_status_t power_manager_get(void)
{
    return s_status;
}
