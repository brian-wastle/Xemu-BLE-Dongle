#include "hid_gamepad.h"
#include <string.h>
#include "esp_log.h"
#include "esp_hid_gap.h"
#include "sdkconfig.h"

static const char *TAG_HID_GP = "HID_GAMEPAD";

// TODO: Identity selection via build-time defines (set from environment in CMake):
#ifndef XEMUBOX_VENDOR_ID
#define XEMUBOX_VENDOR_ID 0xFFFF // unset by default, hard-code here
#endif
#ifndef XEMUBOX_PRODUCT_ID
#define XEMUBOX_PRODUCT_ID 0x0000 // unset by default, hard-code here
#endif

// Optional second device profile for testing
#ifdef XEMUBOX_IDENTITY_PRIV
#define HID_VID 'VID'
#define HID_PID 'PID'
#define HID_DEV_NAME "i.e. Xemupad Wireless Controller"
#define HID_MFG_NAME "i.e. Giantsoft"
#else
#define HID_VID XEMUBOX_VENDOR_ID
#define HID_PID XEMUBOX_PRODUCT_ID
#define HID_DEV_NAME CONFIG_XEMUBOX_DEVICE_NAME
#define HID_MFG_NAME "XemuBox"
#endif

// Single report map:
// 16 facebuttons
// D-pad 1 hat (4-bit)
// 4 stick axes (-127..127)
// 2 trigger axes (0..255)
// Report ID = 1
static const uint8_t gamepad_report_map[] = {
    0x05, 0x01, // Usage Page (Generic Desktop)
    0x09, 0x05, // Usage (Game Pad)
    0xA1, 0x01, // Collection (Application)
    0x85, 0x01, // Report ID (1)

    // Buttons (16)
    0x05, 0x09, // Usage Page (Button)
    0x19, 0x01, // Usage Min (1)
    0x29, 0x10, // Usage Max (16)
    0x15, 0x00, // Logical Min (0)
    0x25, 0x01, // Logical Max (1)
    0x95, 0x10, // Report Count (16)
    0x75, 0x01, // Report Size (1)
    0x81, 0x02, // Input (Data,Var,Abs)

    // Hat switch (4-bit) + 4-bit padding
    0x05, 0x01,       // Usage Page (Generic Desktop)
    0x09, 0x39,       // Usage (Hat switch)
    0x15, 0x00,       // Logical Min (0)
    0x25, 0x07,       // Logical Max (7)
    0x35, 0x00,       // Physical Min (0)
    0x46, 0x3B, 0x01, // Physical Max (315)
    0x75, 0x04,       // Report Size (4)
    0x95, 0x01,       // Report Count (1)
    0x81, 0x42,       // Input (Data,Var,Abs,Null)
    0x75, 0x04,       // Report Size (4)
    0x95, 0x01,       // Report Count (1)
    0x81, 0x03,       // Input (Const,Var,Abs) padding

    // Sticks: X, Y, Rx, Ry (signed-127..127)
    0x05, 0x01, // Usage Page (Generic Desktop)
    0x09, 0x30, // Usage (X)
    0x09, 0x31, // Usage (Y)
    0x09, 0x33, // Usage (Rx)
    0x09, 0x34, // Usage (Ry)
    0x15, 0x81, // Logical Min (-127)
    0x25, 0x7F, // Logical Max (127)
    0x75, 0x08, // Report Size (8)
    0x95, 0x04, // Report Count (4)
    0x81, 0x02, // Input (Data,Var,Abs)

    // Triggers: Z, Rz (unsigned 0..255)
    0x05, 0x01,       // Usage Page (Generic Desktop)
    0x09, 0x32,       // Usage (Z)
    0x09, 0x35,       // Usage (Rz)
    0x15, 0x00,       // Logical Min (0)
    0x26, 0xFF, 0x00, // Logical Max (255)
    0x75, 0x08,       // Report Size (8)
    0x95, 0x02,       // Report Count (2)
    0x81, 0x02,       // Input (Data,Var,Abs)

    0xC0 // End Collection
};

static esp_hidd_dev_t *s_hid_dev = NULL;

static void ble_hidd_event_cb(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_hidd_event_t event = (esp_hidd_event_t)id;
    const esp_hidd_event_data_t *param = (const esp_hidd_event_data_t *)event_data;

    switch (event)
    {
    case ESP_HIDD_START_EVENT:
    {
        esp_err_t adv_rc = ESP_OK;
        if (param && param->start.status != ESP_OK)
        {
            ESP_LOGE(TAG_HID_GP, "HID start failed: %d", param->start.status);
            break;
        }
        if (!param)
        {
            ESP_LOGW(TAG_HID_GP, "HID start event with NULL data; assuming OK");
        }
        else
        {
            ESP_LOGI(TAG_HID_GP, "HID device started");
        }
        // Start BLE advertising once HID is up
        adv_rc = esp_hid_ble_gap_adv_start();
        if (adv_rc != ESP_OK)
        {
            ESP_LOGE(TAG_HID_GP, "Failed to start BLE advertising: %d", adv_rc);
        }
        break;
    }
    case ESP_HIDD_CONNECT_EVENT:
        ESP_LOGI(TAG_HID_GP, "HID client connected");
        // Send a neutral report to help hosts classify the device promptly
        {
            gamepad_state_t zero = {0};
            (void)hid_gamepad_send_state(&zero);
        }
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
        {.data = gamepad_report_map, .len = sizeof(gamepad_report_map)},
    };

    static esp_hid_device_config_t hid_config = {
        .vendor_id = HID_VID,
        .product_id = HID_PID,
        .version = 0x0100,
        .device_name = HID_DEV_NAME,
        .manufacturer_name = HID_MFG_NAME,
        .serial_number = "0001",
        .report_maps = report_maps,
        .report_maps_len = 1,
    };

    if (s_hid_dev)
    {
        if (out_dev)
            *out_dev = s_hid_dev;
        return ESP_OK;
    }

    esp_err_t err = esp_hidd_dev_init(&hid_config, ESP_HID_TRANSPORT_BLE, ble_hidd_event_cb, &s_hid_dev);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG_HID_GP, "esp_hidd_dev_init failed: %d", err);
        return err;
    }
    if (out_dev)
        *out_dev = s_hid_dev;
    return ESP_OK;
}

esp_err_t hid_gamepad_send_state(const gamepad_state_t *state)
{
    if (!s_hid_dev || !state)
        return ESP_ERR_INVALID_STATE;
    uint8_t report[2 + 1 + 4 + 2] = {0}; // 16 buttons + hat + 4 stick axes + 2 triggers
    report[0] = (uint8_t)(state->buttons & 0xFF);
    report[1] = (uint8_t)((state->buttons >> 8) & 0xFF);
    // Hat is 4-bit value (0..7), neutral = 0x0F
    uint8_t hat_nib = (state->hat <= 7) ? (state->hat & 0x0F) : 0x0F;
    report[2] = (uint8_t)(hat_nib | 0xF0); // upper 4 bits padding
    report[3] = (uint8_t)state->x;
    report[4] = (uint8_t)state->y;
    report[5] = (uint8_t)state->rx;
    report[6] = (uint8_t)state->ry;
    report[7] = (uint8_t)state->z;  // LT
    report[8] = (uint8_t)state->rz; // RT
    return esp_hidd_dev_input_set(s_hid_dev, 0, 1, report, sizeof(report));
}
