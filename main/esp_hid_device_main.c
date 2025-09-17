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
    // NVS partition
    esp_err_t part = nvs_flash_init();
    if (part == ESP_ERR_NVS_NO_FREE_PAGES || part == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        part = nvs_flash_init();
    }
    ESP_ERROR_CHECK(part);
    esp_log_level_set("XEMUBOX", ESP_LOG_WARN);
    esp_log_level_set("USB_GIP", ESP_LOG_WARN);
    esp_log_level_set("INPUT_MAP", ESP_LOG_WARN);
    esp_log_level_set("HID_GAMEPAD", ESP_LOG_WARN);
    esp_log_level_set("ESP_HID_GAP", ESP_LOG_WARN);
    ESP_LOGI(TAG, "Initializing HID GAP");
    ESP_ERROR_CHECK(esp_hid_gap_init(HID_DEV_MODE));

// Advertise as an HID Gamepad
#ifdef XEMUBOX_ADV_NAME_STR
    ESP_ERROR_CHECK(esp_hid_ble_gap_adv_init(ESP_HID_APPEARANCE_GAMEPAD, XEMUBOX_ADV_NAME_STR));
#else
    ESP_ERROR_CHECK(esp_hid_ble_gap_adv_init(ESP_HID_APPEARANCE_GAMEPAD, CONFIG_XEMUBOX_DEVICE_NAME));
#endif

// Bluedroid BLE status
#if CONFIG_BT_BLE_ENABLED && !CONFIG_BT_NIMBLE_ENABLED
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(esp_hidd_gatts_event_handler));
#endif

    ESP_LOGI(TAG, "Initializing BLE HID gamepad device");
    ESP_ERROR_CHECK(hid_gamepad_init(NULL));

    // Start app subsystems (stubs for now)
    power_manager_init();
    usb_gip_host_init();
    input_mapper_init();

    ESP_LOGI(TAG, "Startup complete");
}
