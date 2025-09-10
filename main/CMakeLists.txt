set(srcs
    esp_hid_device_main.c
    esp_hid_gap.c
    hid_gamepad.c
    usb_gip_host.c
    input_mapper.c
    power_manager.c
)

set(include_dirs
    .
)

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS ${include_dirs}
    REQUIRES esp_hid
    PRIV_REQUIRES nvs_flash driver
)
