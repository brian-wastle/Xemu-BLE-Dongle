// USB host enumerator and input stub.
// When enabled via Kconfig, this sets up the ESP-IDF USB Host library and logs
// descriptors of any newly attached device. 

#include "usb_gip_host.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include <string.h>
#include <stdio.h>

QueueHandle_t g_gip_out_queue = NULL;

static const char *TAG_GIP = "USB_GIP";

#if CONFIG_XEMUBOX_ENABLE_USB_HOST

#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "usb/usb_host.h"
#include "usb/usb_types_ch9.h"
#include "usb/usb_helpers.h"
#include "driver/gpio.h"

typedef struct {
    usb_host_client_handle_t client_hdl;
    // Current device/interface being captured
    usb_device_handle_t dev_hdl;
    uint8_t intf_num;
    uint8_t alt_setting;
    uint8_t ep_in_addr;
    uint8_t ep_out_addr;
    uint16_t ep_in_mps;
    usb_transfer_t *xfer_in;
    usb_transfer_t *xfer_out;
    uint8_t out_seq;
    uint8_t handshake_stage;
} usb_host_ctx_t;

static usb_host_ctx_t usb_ctx = (usb_host_ctx_t){0};

// Log a device and configuration descriptor summary
static void log_device_summary(usb_device_handle_t dev_hdl)
{
    const usb_device_desc_t *dev_desc = NULL;
    const usb_config_desc_t *cfg = NULL;
    if (usb_host_get_device_descriptor(dev_hdl, &dev_desc) != ESP_OK || !dev_desc) {
        ESP_LOGW(TAG_GIP, "Failed to get device descriptor");
        return;
    }
    if (usb_host_get_active_config_descriptor(dev_hdl, &cfg) != ESP_OK || !cfg) {
        ESP_LOGW(TAG_GIP, "Failed to get active config descriptor");
        return;
    }
    ESP_LOGI(TAG_GIP, "Device: VID=0x%04x PID=0x%04x bcdUSB=0x%04x",
             dev_desc->idVendor, dev_desc->idProduct, dev_desc->bcdUSB);
    ESP_LOGI(TAG_GIP, " ConfigDesc: numIF=%u totalLen=%u", cfg->bNumInterfaces, cfg->wTotalLength);

    const uint8_t *p = (const uint8_t *)cfg;
    const uint8_t *end = p + cfg->wTotalLength;
    // Skip the configuration descriptor header itself
    if (end - p < cfg->bLength) return;
    p += cfg->bLength;
    while (end - p >= 2) {
        uint8_t len = p[0];
        uint8_t type = p[1];
        if (len < 2 || p + len > end) break;
        if (type == USB_B_DESCRIPTOR_TYPE_INTERFACE && len >= sizeof(usb_intf_desc_t)) {
            const usb_intf_desc_t *intf = (const usb_intf_desc_t *)p;
            ESP_LOGI(TAG_GIP, "  IF #%u alt=%u cls=0x%02x sub=0x%02x proto=0x%02x",
                     intf->bInterfaceNumber, intf->bAlternateSetting,
                     intf->bInterfaceClass, intf->bInterfaceSubClass, intf->bInterfaceProtocol);
        } else if (type == USB_B_DESCRIPTOR_TYPE_ENDPOINT && len >= sizeof(usb_ep_desc_t)) {
            const usb_ep_desc_t *ep = (const usb_ep_desc_t *)p;
            uint8_t ep_addr = ep->bEndpointAddress;
            uint8_t ep_dir_in = (ep_addr & USB_B_ENDPOINT_ADDRESS_EP_DIR_MASK) ? 1 : 0;
            uint8_t ep_num = ep_addr & USB_B_ENDPOINT_ADDRESS_EP_NUM_MASK;
            uint8_t attr = ep->bmAttributes & USB_BM_ATTRIBUTES_XFERTYPE_MASK;
            const char *type_str = (attr == USB_BM_ATTRIBUTES_XFER_INT) ? "INTR" :
                                   (attr == USB_BM_ATTRIBUTES_XFER_BULK) ? "BULK" :
                                   (attr == USB_BM_ATTRIBUTES_XFER_CONTROL) ? "CTRL" :
                                   (attr == USB_BM_ATTRIBUTES_XFER_ISOC) ? "ISOC" : "?";
            ESP_LOGI(TAG_GIP, "   EP %u %s %s mps=%u",
                     ep_num, ep_dir_in ? "IN" : "OUT", type_str, ep->wMaxPacketSize);
        }
        p += len;
    }
}

static void handle_new_device(uint8_t addr)
{
    usb_device_handle_t dev_hdl = NULL;
    esp_err_t err = usb_host_device_open(usb_ctx.client_hdl, addr, &dev_hdl);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_GIP, "usb_host_device_open(%u) failed: %d", addr, err);
        return;
    }
    ESP_LOGI(TAG_GIP, "Opened device @ addr %u", addr);
    log_device_summary(dev_hdl);

    // Parse active config and pick an interface to capture from (HID preferred, else vendor)
    const usb_config_desc_t *cfg = NULL;
    if (usb_host_get_active_config_descriptor(dev_hdl, &cfg) != ESP_OK || !cfg) {
        ESP_LOGW(TAG_GIP, "No active config; leaving device open but idle");
        return;
    }
    const uint8_t *p = (const uint8_t *)cfg;
    const uint8_t *end = p + cfg->wTotalLength;
    if ((end - p) < cfg->bLength) return;
    p += cfg->bLength;
    const usb_intf_desc_t *chosen_intf = NULL;
    const usb_ep_desc_t *chosen_ep_in = NULL;
    const usb_ep_desc_t *chosen_ep_out = NULL;
    // First pass: look for HID (cls 0x03)
    for (int pass = 0; pass < 2 && !chosen_ep_in; ++pass) {
        p = (const uint8_t *)cfg + cfg->bLength;
        while (end - p >= 2) {
            uint8_t len = p[0];
            uint8_t type = p[1];
            if (len < 2 || p + len > end) break;
            if (type == USB_B_DESCRIPTOR_TYPE_INTERFACE && len >= sizeof(usb_intf_desc_t)) {
                const usb_intf_desc_t *intf = (const usb_intf_desc_t *)p;
                bool class_ok = (pass == 0) ? (intf->bInterfaceClass == 0x03) : (intf->bInterfaceClass == 0xFF);
                if (class_ok) {
                    // Search following endpoints for interrupt EPs
                    int if_off_base = (int)((const uint8_t *)intf - (const uint8_t *)cfg);
                    for (int ep_idx = 0; ep_idx < intf->bNumEndpoints; ++ep_idx) {
                        int off = if_off_base; // reset to interface offset for each query
                        const usb_ep_desc_t *ep = usb_parse_endpoint_descriptor_by_index(intf, ep_idx, cfg->wTotalLength, &off);
                        if (!ep) break;
                        uint8_t xfer = USB_EP_DESC_GET_XFERTYPE(ep);
                        uint8_t dir_in = USB_EP_DESC_GET_EP_DIR(ep);
                        if (xfer == USB_TRANSFER_TYPE_INTR) {
                            if (dir_in && !chosen_ep_in) chosen_ep_in = ep;
                            if (!dir_in && !chosen_ep_out) chosen_ep_out = ep;
                        }
                    }
                }
            }
            p += len;
        }
    }
    if (!chosen_intf || !chosen_ep_in) {
        // Fallback: linear walk scoped to the interface, independent of helper
        p = (const uint8_t *)cfg + cfg->bLength;
        const usb_intf_desc_t *current_if = NULL;
        while (end - p >= 2) {
            uint8_t len = p[0];
            uint8_t type = p[1];
            if (len < 2 || p + len > end) break;
            if (type == USB_B_DESCRIPTOR_TYPE_INTERFACE && len >= sizeof(usb_intf_desc_t)) {
                current_if = (const usb_intf_desc_t *)p;
            } else if (type == USB_B_DESCRIPTOR_TYPE_ENDPOINT && current_if && len >= sizeof(usb_ep_desc_t)) {
                // HID or vendor 0xFF 0x47
                bool accept = (current_if->bInterfaceClass == 0x03) ||
                              (current_if->bInterfaceClass == 0xFF);
                if (accept) {
                    const usb_ep_desc_t *ep = (const usb_ep_desc_t *)p;
                    uint8_t xfer = USB_EP_DESC_GET_XFERTYPE(ep);
                    uint8_t dir_in = USB_EP_DESC_GET_EP_DIR(ep);
                    if (xfer == USB_TRANSFER_TYPE_INTR) {
                        if (!chosen_intf) chosen_intf = current_if;
                        if (dir_in && !chosen_ep_in) chosen_ep_in = ep;
                        if (!dir_in && !chosen_ep_out) chosen_ep_out = ep;
                    }
                }
            }
            p += len;
            if (chosen_intf && chosen_ep_in) break;
        }
        if (!chosen_intf || !chosen_ep_in) {
            ESP_LOGW(TAG_GIP, "No suitable interface/IN endpoint found (HID or vendor) — idle");
            return;
        }
    }

    // Claim interface
    err = usb_host_interface_claim(usb_ctx.client_hdl, dev_hdl, chosen_intf->bInterfaceNumber, chosen_intf->bAlternateSetting);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_GIP, "interface_claim IF=%u alt=%u failed: %d", chosen_intf->bInterfaceNumber, chosen_intf->bAlternateSetting, err);
        return;
    }
    usb_ctx.dev_hdl = dev_hdl;
    usb_ctx.intf_num = chosen_intf->bInterfaceNumber;
    usb_ctx.alt_setting = chosen_intf->bAlternateSetting;
    usb_ctx.ep_in_addr = chosen_ep_in->bEndpointAddress;
    usb_ctx.ep_in_mps = USB_EP_DESC_GET_MPS(chosen_ep_in);
    usb_ctx.ep_out_addr = chosen_ep_out ? chosen_ep_out->bEndpointAddress : 0;
    if (usb_ctx.ep_out_addr) {
        ESP_LOGI(TAG_GIP, "Capturing IF=%u alt=%u IN EP=0x%02x MPS=%u OUT EP=0x%02x",
                 usb_ctx.intf_num, usb_ctx.alt_setting, usb_ctx.ep_in_addr, usb_ctx.ep_in_mps, usb_ctx.ep_out_addr);
    } else {
        ESP_LOGI(TAG_GIP, "Capturing IF=%u alt=%u IN EP=0x%02x MPS=%u OUT EP=none",
                 usb_ctx.intf_num, usb_ctx.alt_setting, usb_ctx.ep_in_addr, usb_ctx.ep_in_mps);
    }

    // Allocate and submit first IN transfer
    size_t buf_sz = usb_ctx.ep_in_mps;
    if (buf_sz == 0 || buf_sz > sizeof(((gip_frame_t *)0)->data)) buf_sz = sizeof(((gip_frame_t *)0)->data);
    err = usb_host_transfer_alloc(buf_sz, 0, &usb_ctx.xfer_in);
    if (err != ESP_OK || !usb_ctx.xfer_in) {
        ESP_LOGE(TAG_GIP, "transfer_alloc failed: %d", err);
        return;
    }
    usb_ctx.xfer_in->device_handle = usb_ctx.dev_hdl;
    usb_ctx.xfer_in->bEndpointAddress = usb_ctx.ep_in_addr;
    extern void usb_gip_in_transfer_cb(usb_transfer_t *xfer); // forward decl
    usb_ctx.xfer_in->callback = usb_gip_in_transfer_cb;
    usb_ctx.xfer_in->num_bytes = usb_ctx.xfer_in->data_buffer_size; // multiple of MPS
    usb_ctx.xfer_in->context = NULL;
    err = usb_host_transfer_submit(usb_ctx.xfer_in);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_GIP, "transfer_submit failed: %d", err);
    }

#if CONFIG_XEMUBOX_GIP_ENABLE_HANDSHAKE
    // If OUT endpoint exists, attempt a minimal wake-up/subscribe sequence.
    if (usb_ctx.ep_out_addr) {
        size_t out_sz = 64; // allocate up to n MPS
        if (usb_host_transfer_alloc(out_sz, 0, &usb_ctx.xfer_out) == ESP_OK && usb_ctx.xfer_out) {
            usb_ctx.xfer_out->device_handle = usb_ctx.dev_hdl;
            usb_ctx.xfer_out->bEndpointAddress = usb_ctx.ep_out_addr;
            extern void usb_gip_out_transfer_cb(usb_transfer_t *x);
            usb_ctx.xfer_out->callback = usb_gip_out_transfer_cb;
            // Stage 1: initial wake packet
            uint8_t *out = usb_ctx.xfer_out->data_buffer;
            size_t out_len = 0;
            out[out_len++] = 0x05; // host->dev
            out[out_len++] = 0x20; // GIP 0x20 (keepalive/announce)
            out[out_len++] = ++usb_ctx.out_seq; // seq
            out[out_len++] = 0x00; // param
            usb_ctx.xfer_out->num_bytes = out_len;
            esp_err_t se = usb_host_transfer_submit(usb_ctx.xfer_out);
            if (se != ESP_OK) {
                ESP_LOGW(TAG_GIP, "GIP wake submit failed: %d", se);
            } else {
                ESP_LOGI(TAG_GIP, "GIP wake packet sent to EP=0x%02x", usb_ctx.ep_out_addr);
                usb_ctx.handshake_stage = 1;
            }
        }
    }
#endif
}

static void handle_device_gone(usb_device_handle_t dev_hdl)
{
    if (!dev_hdl) return;
    ESP_LOGI(TAG_GIP, "Device removed, closing handle");
    if (usb_ctx.xfer_in) {
        // Try to free transfer. If in-flight, the library will call back with NO_DEVICE and we can free later.
        (void)usb_host_transfer_free(usb_ctx.xfer_in);
        usb_ctx.xfer_in = NULL;
    }
    (void)usb_host_device_close(usb_ctx.client_hdl, dev_hdl);
    usb_ctx.dev_hdl = NULL;
}

static void client_event_cb(const usb_host_client_event_msg_t *event_msg, void *arg)
{
    switch (event_msg->event) {
    case USB_HOST_CLIENT_EVENT_NEW_DEV:
        ESP_LOGI(TAG_GIP, "USB new device: addr=%u", event_msg->new_dev.address);
        handle_new_device(event_msg->new_dev.address);
        break;
    case USB_HOST_CLIENT_EVENT_DEV_GONE:
        ESP_LOGI(TAG_GIP, "USB device gone");
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
        if (err == ESP_OK) {
            if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
                ESP_LOGD(TAG_GIP, "usb_host: no clients");
            }
            if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
                ESP_LOGI(TAG_GIP, "usb_host: all resources freed");
            }
        }
    }
}

static void usb_client_task(void *arg)
{
    usb_host_client_config_t client_config = {
        .is_synchronous = false,
        .max_num_event_msg = 5,
        .async = {
            .client_event_callback = client_event_cb,
            .callback_arg = NULL,
        },
    };
    ESP_ERROR_CHECK(usb_host_client_register(&client_config, &usb_ctx.client_hdl));
    ESP_LOGI(TAG_GIP, "USB client registered");
    while (1) {
        usb_host_client_handle_events(usb_ctx.client_hdl, portMAX_DELAY);
    }
}

static void usb_start(void)
{
    // Route D+/D- to the USB_HOST (Type-A female) connector if configured
#if defined(CONFIG_XEMUBOX_USB_SEL_GPIO)
#if (CONFIG_XEMUBOX_USB_SEL_GPIO >= 0)
    gpio_config_t sel = {
        .pin_bit_mask = (1ULL << CONFIG_XEMUBOX_USB_SEL_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&sel);
#if defined(CONFIG_XEMUBOX_USB_SEL_HOST_LEVEL_HIGH)
    int host_level = CONFIG_XEMUBOX_USB_SEL_HOST_LEVEL_HIGH ? 1 : 0;
#else
    int host_level = 1; // default assume HIGH for HOST
#endif
    gpio_set_level(CONFIG_XEMUBOX_USB_SEL_GPIO, host_level);
    ESP_LOGI(TAG_GIP, "USB_SEL GPIO%u set to %d (route to HOST)", CONFIG_XEMUBOX_USB_SEL_GPIO, host_level);
#else
    ESP_LOGW(TAG_GIP, "USB_SEL configured <0; not switching in SW");
#endif
#else
    ESP_LOGW(TAG_GIP, "USB_SEL Kconfig not present; ensure board routes to HOST");
#endif

    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .root_port_unpowered = false,
        .intr_flags = 0,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));
    ESP_LOGI(TAG_GIP, "USB host installed");
    // Ensure root port is powered (safe to call even if already ON)
    (void)usb_host_lib_set_root_port_power(true);
    xTaskCreatePinnedToCore(usb_host_lib_task, "usb_lib", 4096, NULL, 5, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(usb_client_task,   "usb_cli", 4096, NULL, 5, NULL, tskNO_AFFINITY);
}

#endif // CONFIG_XEMUBOX_ENABLE_USB_HOST

void usb_gip_host_init(void)
{
    if (!g_gip_out_queue) {
        g_gip_out_queue = xQueueCreate(8, sizeof(gip_frame_t));
    }
#if CONFIG_XEMUBOX_ENABLE_USB_HOST
    usb_start();
#else
    ESP_LOGW(TAG_GIP, "USB host disabled in Kconfig; running stub only");
#endif
}

#if CONFIG_XEMUBOX_ENABLE_USB_HOST
// Transfer completion callback for IN endpoint
void usb_gip_in_transfer_cb(usb_transfer_t *xfer)
{
    if (!xfer) return;
    if (xfer->status == USB_TRANSFER_STATUS_COMPLETED && xfer->actual_num_bytes > 0) {
#if CONFIG_XEMUBOX_GIP_VERBOSE_LOG
        int n = xfer->actual_num_bytes;
        if (n > 36) n = 36; // log more to find stick/button offsets
        char hex[160];
        int o = 0;
        for (int i = 0; i < n && o < (int)sizeof(hex)-3; ++i) {
            o += snprintf(hex+o, sizeof(hex)-o, "%02X ", xfer->data_buffer[i]);
        }
        ESP_LOGI(TAG_GIP, "IN %dB: %s", xfer->actual_num_bytes, hex);
#endif
        gip_frame_t f = {0};
        f.len = (uint16_t)((xfer->actual_num_bytes <= sizeof(f.data)) ? xfer->actual_num_bytes : sizeof(f.data));
        memcpy(f.data, xfer->data_buffer, f.len);
        if (g_gip_out_queue) {
            (void)xQueueSend(g_gip_out_queue, &f, 0);
        }
    }
    if (xfer->status == USB_TRANSFER_STATUS_COMPLETED || xfer->status == USB_TRANSFER_STATUS_TIMED_OUT) {
        xfer->num_bytes = xfer->data_buffer_size;
        (void)usb_host_transfer_submit(xfer);
    }
}
#endif

#if CONFIG_XEMUBOX_GIP_ENABLE_HANDSHAKE
// Transfer completion callback for OUT endpoint (handshake sequence)
void usb_gip_out_transfer_cb(usb_transfer_t *x)
{
    if (!x) return;
    ESP_LOGI(TAG_GIP, "OUT done: status=%d bytes=%d", (int)x->status, x->actual_num_bytes);
    if (!usb_ctx.xfer_out) return;
    if (usb_ctx.handshake_stage == 1) {
        uint8_t *out = usb_ctx.xfer_out->data_buffer;
        size_t out_len = 0;
        out[out_len++] = 0x05; // host->device
        out[out_len++] = 0x20; // GIP 0x20
        out[out_len++] = ++usb_ctx.out_seq;
        out[out_len++] = 0x01; // request enable
        usb_ctx.xfer_out->num_bytes = out_len;
        (void)usb_host_transfer_submit(usb_ctx.xfer_out);
        usb_ctx.handshake_stage = 2;
    }
}
#endif
