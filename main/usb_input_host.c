#include "usb_input_host.h"

#include "esp_log.h"
#include "sdkconfig.h"
#include <string.h>
#include <stdbool.h>

QueueHandle_t g_usb_input_queue = NULL;

static const char *TAG_USB_INPUT = "USB_INPUT";

#if CONFIG_XEMUBOX_ENABLE_USB_HOST

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "usb/usb_host.h"
#include "usb/usb_types_ch9.h"

typedef struct {
    usb_host_client_handle_t client_hdl;
    usb_device_handle_t dev_hdl;
    uint8_t intf_num;
    uint8_t alt_setting;
    uint8_t ep_in_addr;
    uint16_t ep_in_mps;
    usb_transfer_t *xfer_in;
} usb_host_ctx_t;

static usb_host_ctx_t usb_ctx = {0};

static bool find_hid_interrupt_in(const usb_config_desc_t *cfg,
                                  const usb_intf_desc_t **out_intf,
                                  const usb_ep_desc_t **out_ep)
{
    if (!cfg || !out_intf || !out_ep) return false;
    const uint8_t *p = (const uint8_t *)cfg + cfg->bLength;
    const uint8_t *end = (const uint8_t *)cfg + cfg->wTotalLength;
    const usb_intf_desc_t *current = NULL;
    while (end - p >= 2) {
        uint8_t len = p[0];
        uint8_t type = p[1];
        if (len < 2 || p + len > end) break;
        if (type == USB_B_DESCRIPTOR_TYPE_INTERFACE && len >= sizeof(usb_intf_desc_t)) {
            current = (const usb_intf_desc_t *)p;
        } else if (type == USB_B_DESCRIPTOR_TYPE_ENDPOINT && current && len >= sizeof(usb_ep_desc_t)) {
            if (current->bInterfaceClass == 0x03) {
                const usb_ep_desc_t *ep = (const usb_ep_desc_t *)p;
                bool intr = USB_EP_DESC_GET_XFERTYPE(ep) == USB_TRANSFER_TYPE_INTR;
                bool dir_in = USB_EP_DESC_GET_EP_DIR(ep);
                if (intr && dir_in) {
                    *out_intf = current;
                    *out_ep = ep;
                    return true;
                }
            }
        }
        p += len;
    }
    return false;
}

static void log_device_summary(usb_device_handle_t dev_hdl)
{
    const usb_device_desc_t *dev_desc = NULL;
    if (usb_host_get_device_descriptor(dev_hdl, &dev_desc) != ESP_OK || !dev_desc) {
        ESP_LOGW(TAG_USB_INPUT, "Failed to read device descriptor");
        return;
    }
    ESP_LOGI(TAG_USB_INPUT, "Attached USB HID candidate VID=0x%04x PID=0x%04x bcdUSB=0x%04x",
             dev_desc->idVendor, dev_desc->idProduct, dev_desc->bcdUSB);
}

static void usb_input_in_transfer_cb(usb_transfer_t *xfer);

static void handle_new_device(uint8_t addr)
{
    usb_device_handle_t dev_hdl = NULL;
    esp_err_t err = usb_host_device_open(usb_ctx.client_hdl, addr, &dev_hdl);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_USB_INPUT, "usb_host_device_open(%u) failed: %d", addr, err);
        return;
    }

    log_device_summary(dev_hdl);

    const usb_config_desc_t *cfg = NULL;
    if (usb_host_get_active_config_descriptor(dev_hdl, &cfg) != ESP_OK || !cfg) {
        ESP_LOGW(TAG_USB_INPUT, "No active configuration descriptor");
        usb_host_device_close(usb_ctx.client_hdl, dev_hdl);
        return;
    }

    const usb_intf_desc_t *hid_intf = NULL;
    const usb_ep_desc_t *hid_ep = NULL;
    if (!find_hid_interrupt_in(cfg, &hid_intf, &hid_ep)) {
        ESP_LOGW(TAG_USB_INPUT, "No HID interrupt IN endpoint found");
        usb_host_device_close(usb_ctx.client_hdl, dev_hdl);
        return;
    }

    err = usb_host_interface_claim(usb_ctx.client_hdl, dev_hdl,
                                   hid_intf->bInterfaceNumber, hid_intf->bAlternateSetting);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_USB_INPUT, "interface_claim IF=%u alt=%u failed: %d",
                 hid_intf->bInterfaceNumber, hid_intf->bAlternateSetting, err);
        usb_host_device_close(usb_ctx.client_hdl, dev_hdl);
        return;
    }

    usb_ctx.dev_hdl = dev_hdl;
    usb_ctx.intf_num = hid_intf->bInterfaceNumber;
    usb_ctx.alt_setting = hid_intf->bAlternateSetting;
    usb_ctx.ep_in_addr = hid_ep->bEndpointAddress;
    usb_ctx.ep_in_mps = USB_EP_DESC_GET_MPS(hid_ep);

    ESP_LOGI(TAG_USB_INPUT, "Capturing IF=%u alt=%u IN EP=0x%02x MPS=%u",
             usb_ctx.intf_num, usb_ctx.alt_setting, usb_ctx.ep_in_addr, usb_ctx.ep_in_mps);

    size_t buf_sz = usb_ctx.ep_in_mps;
    if (buf_sz == 0 || buf_sz > sizeof(((usb_input_frame_t *)0)->data)) {
        buf_sz = sizeof(((usb_input_frame_t *)0)->data);
    }
    err = usb_host_transfer_alloc(buf_sz, 0, &usb_ctx.xfer_in);
    if (err != ESP_OK || !usb_ctx.xfer_in) {
        ESP_LOGE(TAG_USB_INPUT, "transfer_alloc failed: %d", err);
        usb_host_device_close(usb_ctx.client_hdl, dev_hdl);
        usb_ctx.dev_hdl = NULL;
        return;
    }
    usb_ctx.xfer_in->device_handle = dev_hdl;
    usb_ctx.xfer_in->bEndpointAddress = usb_ctx.ep_in_addr;
    usb_ctx.xfer_in->callback = usb_input_in_transfer_cb;
    usb_ctx.xfer_in->num_bytes = usb_ctx.xfer_in->data_buffer_size;
    err = usb_host_transfer_submit(usb_ctx.xfer_in);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_USB_INPUT, "transfer_submit failed: %d", err);
    }
}

static void handle_device_gone(usb_device_handle_t dev_hdl)
{
    if (!dev_hdl) return;
    ESP_LOGI(TAG_USB_INPUT, "Device removed");
    if (usb_ctx.xfer_in) {
        usb_host_transfer_free(usb_ctx.xfer_in);
        usb_ctx.xfer_in = NULL;
    }
    usb_host_device_close(usb_ctx.client_hdl, dev_hdl);
    usb_ctx.dev_hdl = NULL;
}

static void client_event_cb(const usb_host_client_event_msg_t *event_msg, void *arg)
{
    switch (event_msg->event) {
    case USB_HOST_CLIENT_EVENT_NEW_DEV:
        ESP_LOGI(TAG_USB_INPUT, "USB new device @ addr %u", event_msg->new_dev.address);
        handle_new_device(event_msg->new_dev.address);
        break;
    case USB_HOST_CLIENT_EVENT_DEV_GONE:
        handle_device_gone(event_msg->dev_gone.dev_hdl);
        break;
    default:
        break;
    }
}

static void usb_host_lib_task(void *arg)
{
    uint32_t event_flags;
    while (1) {
        esp_err_t err = usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        if (err == ESP_OK && (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE)) {
            ESP_LOGI(TAG_USB_INPUT, "usb_host: all resources freed");
        }
    }
}

static void usb_client_task(void *arg)
{
    usb_host_client_config_t cfg = {
        .is_synchronous = false,
        .max_num_event_msg = 5,
        .async = {
            .client_event_callback = client_event_cb,
            .callback_arg = NULL,
        },
    };
    ESP_ERROR_CHECK(usb_host_client_register(&cfg, &usb_ctx.client_hdl));
    while (1) {
        usb_host_client_handle_events(usb_ctx.client_hdl, portMAX_DELAY);
    }
}

static void usb_start(void)
{
#if defined(CONFIG_XEMUBOX_USB_SEL_GPIO) && (CONFIG_XEMUBOX_USB_SEL_GPIO >= 0)
    gpio_config_t sel = {
        .pin_bit_mask = (1ULL << CONFIG_XEMUBOX_USB_SEL_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&sel);
#if defined(CONFIG_XEMUBOX_USB_SEL_HOST_LEVEL_HIGH)
    const int host_level = CONFIG_XEMUBOX_USB_SEL_HOST_LEVEL_HIGH ? 1 : 0;
#else
    const int host_level = 1;
#endif
    gpio_set_level(CONFIG_XEMUBOX_USB_SEL_GPIO, host_level);
    ESP_LOGI(TAG_USB_INPUT, "USB_SEL GPIO%u set to %d (host path)", CONFIG_XEMUBOX_USB_SEL_GPIO, host_level);
#else
    ESP_LOGW(TAG_USB_INPUT, "USB_SEL GPIO not configured; ensure board routes host port");
#endif

    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .root_port_unpowered = false,
        .intr_flags = 0,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));
    (void)usb_host_lib_set_root_port_power(true);
    xTaskCreatePinnedToCore(usb_host_lib_task, "usb_lib", 4096, NULL, 5, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(usb_client_task,   "usb_cli", 4096, NULL, 5, NULL, tskNO_AFFINITY);
}

static void usb_input_in_transfer_cb(usb_transfer_t *xfer)
{
    if (!xfer) return;
    if (xfer->status == USB_TRANSFER_STATUS_COMPLETED && xfer->actual_num_bytes > 0) {
        usb_input_frame_t frame = {0};
        frame.len = (uint16_t)((xfer->actual_num_bytes <= sizeof(frame.data)) ? xfer->actual_num_bytes : sizeof(frame.data));
        memcpy(frame.data, xfer->data_buffer, frame.len);
        if (g_usb_input_queue) {
            (void)xQueueSend(g_usb_input_queue, &frame, 0);
        }
    }
    if (xfer->status == USB_TRANSFER_STATUS_COMPLETED || xfer->status == USB_TRANSFER_STATUS_TIMED_OUT) {
        xfer->num_bytes = xfer->data_buffer_size;
        (void)usb_host_transfer_submit(xfer);
    }
}

#endif // CONFIG_XEMUBOX_ENABLE_USB_HOST

void usb_input_host_init(void)
{
    if (!g_usb_input_queue) {
        g_usb_input_queue = xQueueCreate(8, sizeof(usb_input_frame_t));
    }
#if CONFIG_XEMUBOX_ENABLE_USB_HOST
    usb_start();
#else
    ESP_LOGW(TAG_USB_INPUT, "USB host disabled in Kconfig; running stub only");
#endif
}
