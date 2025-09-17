set(srcs
    esp_hid_device_main.c
    esp_hid_gap.c
    hid_gamepad.c
    usb_gip_host.c
    input_mapper.c
    power_manager.c
    ble_hid_task_startup_stub.c
)

set(include_dirs
    .
)

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS ${include_dirs}
    REQUIRES esp_hid
    PRIV_REQUIRES nvs_flash driver usb esp_adc
)

# ID overrides via environment variables at build time.
# Replace <VID> and <PID> with your licensed VID/PID:
#   XEMUBOX_IDENTITY=myPad idf.py build
#   XEMUBOX_VENDOR_ID=<VID> XEMUBOX_PRODUCT_ID=<PID> idf.py build
#   XEMUBOX_ADV_NAME="MyPad" idf.py build

if(DEFINED ENV{XEMUBOX_IDENTITY})
    if($ENV{XEMUBOX_IDENTITY} STREQUAL "xbox")
        target_compile_definitions(${COMPONENT_LIB} PRIVATE XEMUBOX_IDENTITY_PRIV=1)
        message(STATUS "XemuBox identity: Compatibility mode")
    endif()
endif()

if(DEFINED ENV{XEMUBOX_VENDOR_ID})
    target_compile_definitions(${COMPONENT_LIB} PRIVATE XEMUBOX_VENDOR_ID=$ENV{XEMUBOX_VENDOR_ID})
    message(STATUS "XemuBox HID VID override: $ENV{XEMUBOX_VENDOR_ID}")
endif()

if(DEFINED ENV{XEMUBOX_PRODUCT_ID})
    target_compile_definitions(${COMPONENT_LIB} PRIVATE XEMUBOX_PRODUCT_ID=$ENV{XEMUBOX_PRODUCT_ID})
    message(STATUS "XemuBox HID PID override: $ENV{XEMUBOX_PRODUCT_ID}")
endif()

if(DEFINED ENV{XEMUBOX_ADV_NAME})
    string(REPLACE "\"" "\\\"" XEMUBOX_ADV_NAME_ESC "$ENV{XEMUBOX_ADV_NAME}")
    target_compile_definitions(${COMPONENT_LIB} PRIVATE XEMUBOX_ADV_NAME_STR="\"${XEMUBOX_ADV_NAME_ESC}\"")
    message(STATUS "XemuBox advertised name: $ENV{XEMUBOX_ADV_NAME}")
endif()
