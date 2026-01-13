# ESP32-S3 BLE Gamepad Bridge

This project was born from the necessity that the XemuBox emulation miniPc build have a wireless controller solution that closely mimics the original hardware. The Hyperkin DuchesS is an officially licensed remake of the original Xbox's S Controller which uses a USB-C interface. This is ESP32‑S3 firmware that hosts a USB controller (MS-GIPUSB) and exposes a Bluetooth LE HID Gamepad to the host. Out-of-box, it's already tuned for use with Steam, with mappings for sticks, triggers, D‑pad hat, face/shoulder/menu buttons, and Guide menu button.

You will need to acquire up a hardware ID for your device, which is comprised of a Vendor ID (VID) and a Product ID (PID). These are typically licensed by the USB consortium. These can be very expensive. More details on configuration follow.

In order for this device to be advertised over BLE so that Xemu will map it correctly, your PC will need to recognize the device over Bluetooth as a Microsoft Xbox controller. This could only be accomplished by spoofing an official Microsoft device, so solutions for Xemu auto mapping are in the works. 

## Targets

- ESP-IDF 5.5.x
- ESP32-S3-USB-OTG DevKit or similar boards that support USB Host mode.
- Windows / Linux hosts for testing BLE HID peripherals.

## Directory Layout

- `main/esp_hid_device_main.c` — Application bootstrap (NVS, GAP, BLE HID init).
- `main/hid_gamepad.{c,h}` — BLE HID device implementation and report map.
- `main/usb_input_host.{c,h}` — Generic USB host enumerator that streams interrupt IN reports into a queue.
- `main/input_mapper.{c,h}` — Minimal example converting queued USB HID frames into `gamepad_state_t`.
- `main/esp_hid_gap.{c,h}` — GAP helpers (advertising, security callbacks).
- `main/power_manager.{c,h}` — Stubs for power/battery control on portable builds.

## Building / Flashing

1. Install ESP-IDF 5.5.x (VS Code extension or command-line).
2. Select target and optional config:
   - `idf.py set-target esp32s3`
   - Optional: `idf.py menuconfig` → “XemuBox Configuration” to set device name, GPIO routing, etc.
3. Build, flash, and monitor:
   - `idf.py -p <PORT> build flash monitor`

The BLE HID identity defaults to a neutral `CONFIG_XEMUBOX_DEVICE_NAME` value and 0xFFFF/0x0000 VID/PID. Override them with environment variables if you licensed your own IDs:

```
set XEMUBOX_VENDOR_ID=0x1209
set XEMUBOX_PRODUCT_ID=0x0001
set XEMUBOX_ADV_NAME="DemoPad"
idf.py build flash
```

## USB-to-BLE Flow

1. `usb_input_host_init()` powers the onboard USB Host PHY, registers a client, and listens for HID-class devices.
2. Interrupt IN packets are copied into `g_usb_input_queue` as `usb_input_frame_t` entries.
3. `input_mapper_task` consumes queue entries. For demonstration purposes it expects a very small, fully documented report structure (`buttons`, `hat`, axes, triggers) and forwards it to the BLE HID driver.
4. `hid_gamepad_send_state()` emits the BLE HID report, which any OS can pair with as a standard controller.

You can replace the simple mapper with your own logic (e.g., parsing a custom HID descriptor) without exposing any proprietary capture.

## Why This Is Safe to Publish

- No captured packets from commercial devices are shipped—only a toy `simple_hid_gamepad_report_t`.
- USB logging is generic (VID/PID are printed strictly for debugging).
- BLE identifiers remain neutral unless you deliberately override them at build time.
- `sdkconfig` is intentionally ignored; contributors generate their own local copies.

## Legal Reminder

This repository is for educational and portfolio use. If you adapt it for commercial hardware you must:

- Obtain your own USB Vendor/Product IDs and Bluetooth SIG listings.
- Respect any third-party controller licenses and regional regulations.
- Avoid impersonating trademarked device identities unless you have explicit permission.

## Acknowledgements

- Espressif’s ESP-IDF Bluetooth HID examples for the GAP scaffolding.
- The SDL community for open HID mapping references.
