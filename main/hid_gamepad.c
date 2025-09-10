#include "hid_gamepad.h"
#include <string.h>
#include "esp_log.h"
#include "esp_hid_gap.h"
#include "sdkconfig.h"

static const char *TAG_HID_GP = "HID_GAMEPAD";

// Single report map: 16 buttons + 4 analog axes (8-bit, -127..127)
// Report ID = 1
static const uint8_t gamepad_report_map[] = {
    0x05, 0x01,       // Usage Page (Generic Desktop)
    0x09, 0x05,       // Usage (Game Pad)
    0xA1, 0x01,       // Collection (Application)
    0x85, 0x01,       //   Report ID (1)

    // Buttons (16)
    0x05, 0x09,       //   Usage Page (Button)
    0x19, 0x01,       //   Usage Min (1)
    0x29, 0x10,       //   Usage Max (16)
    0x15, 0x00,       //   Logical Min (0)
    0x25, 0x01,       //   Logical Max (1)
    0x95, 0x10,       //   Report Count (16)
    0x75, 0x01,       //   Report Size (1)
    0x81, 0x02,       //   Input (Data,Var,Abs)

    // Axes (X, Y, Rx, Ry) signed 8-bit
    0x05, 0x01,       //   Usage Page (Generic Desktop)
    0x09, 0x30,       //   Usage (X)
    0x09, 0x31,       //   Usage (Y)
    0x09, 0x33,       //   Usage (Rx)
    0x09, 0x34,       //   Usage (Ry)
    0x15, 0x81,       //   Logical Min (-127)
    0x25, 0x7F,       //   Logical Max (127)
    0x75, 0x08,       //   Report Size (8)
    0x95, 0x04,       //   Report Count (4)
    0x81, 0x02,       //   Input (Data,Var,Abs)

    0xC0              // End Collection
};

static esp_hidd_dev_t *s_hid_dev = NULL;

static void ble_hidd_event_cb(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_hidd_event_t event = (esp_hidd_event_t)id;
    esp_hidd_event_data_t *param = (esp_hidd_event_data_t *)event_data;
    switch (event) {
        case ESP_HIDD_START_EVENT:
            if (param->start.status == ESP_OK) {
                ESP_LOGI(TAG_HID_GP, "HID device started");
                // Start BLE advertising once HID is up
                (void)esp_hid_ble_gap_adv_start();
            } else {
                ESP_LOGE(TAG_HID_GP, "HID start failed: %d", param->start.status);
            }
            break;
        case ESP_HIDD_CONNECT_EVENT:
            ESP_LOGI(TAG_HID_GP, "HID client connected");
            break;
        case ESP_HIDD_DISCONNECT_EVENT:
            ESP_LOGI(TAG_HID_GP, "HID client disconnected");
            break;
        default:
            break;
    }
}

esp_err_t hid_gamepad_init(esp_hidd_dev_t **out_dev)
{
    static esp_hid_raw_report_map_t report_maps[] = {
        { .data = gamepad_report_map, .len = sizeof(gamepad_report_map) },
    };

    static esp_hid_device_config_t hid_config = {
        .vendor_id         = 0x16C0, // V-USB VID (placeholder)
        .product_id        = 0x27DB, // Placeholder PID
        .version           = 0x0100,
        .device_name       = CONFIG_XEMUBOX_DEVICE_NAME,
        .manufacturer_name = "XemuBox",
        .serial_number     = "0001",
        .report_maps       = report_maps,
        .report_maps_len   = 1,
    };

    if (s_hid_dev) {
        if (out_dev) *out_dev = s_hid_dev;
        return ESP_OK;
    }

    esp_err_t err = esp_hidd_dev_init(&hid_config, ESP_HID_TRANSPORT_BLE, ble_hidd_event_cb, &s_hid_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_HID_GP, "esp_hidd_dev_init failed: %d", err);
        return err;
    }
    if (out_dev) *out_dev = s_hid_dev;
    return ESP_OK;
}

esp_err_t hid_gamepad_send_state(const gamepad_state_t *state)
{
    if (!s_hid_dev || !state) return ESP_ERR_INVALID_STATE;
    uint8_t report[2 + 4] = {0}; // 16 buttons + 4 axes
    report[0] = (uint8_t)(state->buttons & 0xFF);
    report[1] = (uint8_t)((state->buttons >> 8) & 0xFF);
    report[2] = (uint8_t)state->x;
    report[3] = (uint8_t)state->y;
    report[4] = (uint8_t)state->rx;
    report[5] = (uint8_t)state->ry;
    return esp_hidd_dev_input_set(s_hid_dev, 0, 1, report, sizeof(report));
}
