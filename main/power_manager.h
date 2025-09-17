#pragma once

#include <stdbool.h>
#include <stdint.h>

// Optional: configure battery sense ADC and VBUS/5V enable pins here.
// You can later move these into Kconfig.
// #define PM_BAT_ADC_UNIT    ADC_UNIT_1
// #define PM_BAT_ADC_CH      ADC_CHANNEL_0   // e.g., GPIO36 on some chips
// #define PM_VBUS_EN_GPIO    GPIO_NUM_XX     // set to actual pin controlling 5V for USB host

typedef struct
{
    bool usb_host_5v_on;   // 5V present for USB host (from DEV or boost)
    bool boost_enabled;    // whether on-board boost is enabled via GPIO
    bool dev_vbus_present; // VBUS present on USB-DEV connector
    int batt_mv;           // battery voltage in millivolts (approx)
} power_status_t;

// Initialize power manager and start a monitoring task
void power_manager_init(void);

// Get last known power status (non-blocking)
power_status_t power_manager_get(void);
