#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "sdkconfig.h"
#include "esp_hidd.h"
#include "esp_hid_gap.h"

#include "hid_gamepad.h"
#include "usb_gip_host.h"
#include "input_mapper.h"
#include "power_manager.h"

static const char *TAG = "XEMUBOX";

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "Initializing HID GAP");
    ESP_ERROR_CHECK(esp_hid_gap_init(HID_DEV_MODE));

    // Advertise as a generic HID device with our gamepad name (configurable)
    ESP_ERROR_CHECK(esp_hid_ble_gap_adv_init(ESP_HID_APPEARANCE_GENERIC, CONFIG_XEMUBOX_DEVICE_NAME));

#if CONFIG_BT_BLE_ENABLED && !CONFIG_BT_NIMBLE_ENABLED
    // Only needed for Bluedroid BLE
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(esp_hidd_gatts_event_handler));
#endif

    ESP_LOGI(TAG, "Initializing BLE HID gamepad device");
    ESP_ERROR_CHECK(hid_gamepad_init(NULL));

    // Start app subsystems (stubs for now)
    power_manager_init();
    usb_gip_host_init();
    input_mapper_init();

    ESP_LOGI(TAG, "Startup complete; waiting for BLE connections and USB input");
}
