#include "power_manager.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"

static const char *TAG_PWR = "POWER";

// Provide safe defaults if new Kconfig options have not been realized yet
#ifndef CONFIG_XEMUBOX_LIMIT_EN_GPIO
#define CONFIG_XEMUBOX_LIMIT_EN_GPIO -1
#endif
#ifndef CONFIG_XEMUBOX_BOOST_EN_GPIO
#define CONFIG_XEMUBOX_BOOST_EN_GPIO -1
#endif
#ifndef CONFIG_XEMUBOX_OVER_CURRENT_GPIO
#define CONFIG_XEMUBOX_OVER_CURRENT_GPIO -1
#endif
#ifndef CONFIG_XEMUBOX_HOST_VOL_ADC_CH
#define CONFIG_XEMUBOX_HOST_VOL_ADC_CH -1
#endif
#ifndef CONFIG_XEMUBOX_BATT_VOL_ADC_CH
#define CONFIG_XEMUBOX_BATT_VOL_ADC_CH -1
#endif
#ifndef CONFIG_XEMUBOX_VBUS_SENSE_GPIO
#define CONFIG_XEMUBOX_VBUS_SENSE_GPIO -1
#endif
#ifndef CONFIG_XEMUBOX_BATT_MV_MIN_START_USB_HOST
#define CONFIG_XEMUBOX_BATT_MV_MIN_START_USB_HOST 3400
#endif
#ifndef CONFIG_XEMUBOX_BATT_MV_CUTOFF
#define CONFIG_XEMUBOX_BATT_MV_CUTOFF 3200
#endif
#ifndef CONFIG_XEMUBOX_DEV_VBUS_EN_GPIO
#define CONFIG_XEMUBOX_DEV_VBUS_EN_GPIO -1
#endif
#ifndef CONFIG_XEMUBOX_OC_ACTIVE_HIGH
#define CONFIG_XEMUBOX_OC_ACTIVE_HIGH 1
#endif
#ifndef CONFIG_XEMUBOX_IGNORE_OVER_CURRENT
#define CONFIG_XEMUBOX_IGNORE_OVER_CURRENT 0
#endif
static power_status_t power_state = {0};

static adc_oneshot_unit_handle_t adc1_handle = NULL;
static bool power_inited = false;

static inline void set_limit_enable(bool on)
{
#if (CONFIG_XEMUBOX_LIMIT_EN_GPIO >= 0)
    gpio_set_level(CONFIG_XEMUBOX_LIMIT_EN_GPIO, on ? 1 : 0);
#endif
}

static inline void set_boost_enable(bool on)
{
#if (CONFIG_XEMUBOX_BOOST_EN_GPIO >= 0)
    gpio_set_level(CONFIG_XEMUBOX_BOOST_EN_GPIO, on ? 1 : 0);
#endif
    power_state.boost_enabled = on;
}

static inline void set_dev_vbus_enable(bool on)
{
#if (CONFIG_XEMUBOX_DEV_VBUS_EN_GPIO >= 0)
    gpio_set_level(CONFIG_XEMUBOX_DEV_VBUS_EN_GPIO, on ? 1 : 0);
#endif
}

static int read_batt_mv(void)
{
#if (CONFIG_XEMUBOX_BATT_VOL_ADC_CH >= 0)
    if (!adc1_handle)
        return 0;
    int raw = 0;
    if (adc_oneshot_read(adc1_handle, CONFIG_XEMUBOX_BATT_VOL_ADC_CH, &raw) != ESP_OK)
        return 0;
    int pin_mv = (raw * 2500) / 4095;
    int vbatt_mv = pin_mv * 2;
    return vbatt_mv;
#else
    return 0;
#endif
}

static void power_task(void *arg)
{
    ESP_LOGI(TAG_PWR, "Power manager task running (stub)");
    power_inited = true;
    bool last_dev = false;
    bool last_boost = false;
    bool last_lim = false;
    int last_batt = -1;
    int oc_high_count = 0;
    const int oc_high_threshold = 2; // ~1s debounce at 500ms loop
    bool dev_power_armed = false;    // sequence DEV_VBUS_EN, LIMIT_EN next cycle
    while (1)
    {
        // Sense external 5V presence
        bool dev_vbus = false;
#if (CONFIG_XEMUBOX_HOST_VOL_ADC_CH >= 0)
        int raw = 0;
        if (adc_oneshot_read(adc1_handle, CONFIG_XEMUBOX_HOST_VOL_ADC_CH, &raw) == ESP_OK)
        {
            // Crude threshold: if > ~0.5V on divider, assume present
            dev_vbus = raw > 300; // tune as needed
        }
#elif (CONFIG_XEMUBOX_VBUS_SENSE_GPIO >= 0)
        dev_vbus = gpio_get_level(CONFIG_XEMUBOX_VBUS_SENSE_GPIO) ? true : false;
#endif
        power_state.dev_vbus_present = dev_vbus;

        // Read battery voltage (approx)
        power_state.batt_mv = read_batt_mv();

        // Decide 5V source policy for ESP32-S3-USB-OTG board:
        // - If DEV VBUS present > use DEV power, keep boost disabled (avoid backfeed)
        // - If DEV VBUS absent > enable boost only if battery is healthy
        bool want_boost = !dev_vbus;
        if (want_boost)
        {
            if (power_state.batt_mv > 0 && power_state.batt_mv < CONFIG_XEMUBOX_BATT_MV_MIN_START_USB_HOST)
            {
                // Battery too low to start boost for host
                want_boost = false;
            }
        }
        else
        {
            // External power available; do not run boost
            want_boost = false;
        }

        // Cutoff protection
        if (power_state.batt_mv > 0 && power_state.batt_mv < CONFIG_XEMUBOX_BATT_MV_CUTOFF)
        {
            want_boost = false;
        }

        // Sequence DEV power path to avoid false OC (UVLO/inrush):
        // On rising edge of dev_vbus, enable DEV_VBUS_EN first, then arm LIMIT_EN for next cycle
        if (dev_vbus && !last_dev)
        {
            set_dev_vbus_enable(true);
            dev_power_armed = true;
        }
        else if (!dev_vbus && last_dev)
        {
            set_dev_vbus_enable(false);
            dev_power_armed = false;
        }

        // Enable/disable boost and current limiter
        set_boost_enable(want_boost);

        bool over_current = false;
#if (CONFIG_XEMUBOX_OVER_CURRENT_GPIO >= 0)
        // Ignore OC only when external 5V is present (DEV VBUS) and the
        // Kconfig option is enabled. While on battery/boost, OC remains enforced.
        bool ignore_oc_now = (CONFIG_XEMUBOX_IGNORE_OVER_CURRENT != 0) && dev_vbus;
#endif
#if (CONFIG_XEMUBOX_OVER_CURRENT_GPIO >= 0)
        int oc_raw = gpio_get_level(CONFIG_XEMUBOX_OVER_CURRENT_GPIO) ? 1 : 0;
        bool oc_active = CONFIG_XEMUBOX_OC_ACTIVE_HIGH ? (oc_raw == 1) : (oc_raw == 0);
        if (oc_active)
            oc_high_count++;
        else
            oc_high_count = 0;
        over_current = (oc_high_count >= oc_high_threshold);
        if (over_current && !ignore_oc_now)
        {
            ESP_LOGW(TAG_PWR, "USB host overcurrent detected; disabling LIMIT_EN");
        }
#endif
        // Mask OC if ignoring on DEV power; enforce OC on battery
#if (CONFIG_XEMUBOX_OVER_CURRENT_GPIO >= 0)
        if (ignore_oc_now)
            over_current = false;
#endif
        bool want_limit = (dev_vbus || power_state.boost_enabled) && !over_current;
        bool lim_on = want_limit && !dev_power_armed;
        set_limit_enable(lim_on);
        dev_power_armed = false; // clear after one cycle

        // Report whether 5V is intended to be present on the host connector
        power_state.usb_host_5v_on = lim_on;

        if (dev_vbus != last_dev || power_state.boost_enabled != last_boost || lim_on != last_lim || (last_batt < 0) || (abs(power_state.batt_mv - last_batt) > 50))
        {
            ESP_LOGI(TAG_PWR, "PWR dev_vbus=%d boost=%d limit_en=%d batt=%dmV host5v=%d",
                     dev_vbus, power_state.boost_enabled, lim_on, power_state.batt_mv, power_state.usb_host_5v_on);
            last_dev = dev_vbus;
            last_boost = power_state.boost_enabled;
            last_lim = lim_on;
            last_batt = power_state.batt_mv;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void power_manager_init(void)
{
    ESP_LOGI(TAG_PWR, "Config pins: LIMIT_EN=%d BOOST_EN=%d OC=%d HOST_VOL_CH=%d BATT_VOL_CH=%d",
             (int)CONFIG_XEMUBOX_LIMIT_EN_GPIO, (int)CONFIG_XEMUBOX_BOOST_EN_GPIO,
             (int)CONFIG_XEMUBOX_OVER_CURRENT_GPIO, (int)CONFIG_XEMUBOX_HOST_VOL_ADC_CH,
             (int)CONFIG_XEMUBOX_BATT_VOL_ADC_CH);
// Configure LIMIT_EN (host current limiter)
#if (CONFIG_XEMUBOX_LIMIT_EN_GPIO >= 0)
    gpio_config_t io_lim = {
        .pin_bit_mask = (1ULL << CONFIG_XEMUBOX_LIMIT_EN_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_lim);
    gpio_set_level(CONFIG_XEMUBOX_LIMIT_EN_GPIO, 0);
#endif

// Configure BOOST_EN
#if (CONFIG_XEMUBOX_BOOST_EN_GPIO >= 0)
    gpio_config_t io_boost = {
        .pin_bit_mask = (1ULL << CONFIG_XEMUBOX_BOOST_EN_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_boost);
    gpio_set_level(CONFIG_XEMUBOX_BOOST_EN_GPIO, 0);
#endif

// Configure OVER_CURRENT input
#if (CONFIG_XEMUBOX_OVER_CURRENT_GPIO >= 0)
    gpio_config_t io_oc = {
        .pin_bit_mask = (1ULL << CONFIG_XEMUBOX_OVER_CURRENT_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_oc);
#endif

// Configure DEV_VBUS_EN
#if (CONFIG_XEMUBOX_DEV_VBUS_EN_GPIO >= 0)
    gpio_config_t io_dven = {
        .pin_bit_mask = (1ULL << CONFIG_XEMUBOX_DEV_VBUS_EN_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_dven);
    gpio_set_level(CONFIG_XEMUBOX_DEV_VBUS_EN_GPIO, 0);
#endif

// Init ADC for supply/battery sense (ADC1 oneshot)
#if ((CONFIG_XEMUBOX_BATT_VOL_ADC_CH >= 0) || (CONFIG_XEMUBOX_HOST_VOL_ADC_CH >= 0))
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    if (adc_oneshot_new_unit(&unit_cfg, &adc1_handle) == ESP_OK)
    {
        adc_oneshot_chan_cfg_t ch_cfg = {
            .bitwidth = ADC_BITWIDTH_DEFAULT,
            .atten = ADC_ATTEN_DB_12,
        };
#if (CONFIG_XEMUBOX_BATT_VOL_ADC_CH >= 0)
        (void)adc_oneshot_config_channel(adc1_handle, CONFIG_XEMUBOX_BATT_VOL_ADC_CH, &ch_cfg);
#endif
#if (CONFIG_XEMUBOX_HOST_VOL_ADC_CH >= 0)
        (void)adc_oneshot_config_channel(adc1_handle, CONFIG_XEMUBOX_HOST_VOL_ADC_CH, &ch_cfg);
#endif
    }
#endif
    xTaskCreate(power_task, "power_task", 4096, NULL, 4, NULL);
}

power_status_t power_manager_get(void)
{
    return power_state;
}
