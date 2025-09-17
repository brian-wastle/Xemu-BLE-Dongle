# XemuBox BT Dongle

This project was borne from the necessity that the XemuBox emulation miniPc build have a wireless controller solution that closely mimics the original hardware. The Hyperkin DuchesS is an officially licensed remake of the original Xbox's S Controller which uses a USB-C interface. This is ESP32‑S3 firmware that hosts a USB controller (MS-GIPUSB) and exposes a Bluetooth LE HID Gamepad to the host. Out-of-box, it's already tuned for use with Steam, with mappings for sticks, triggers, D‑pad hat, face/shoulder/menu buttons, and Guide menu button.

You will need to acquire up a hardware ID for your device, which is comprised of a Vendor ID (VID) and a Product ID (PID). These are typically licensed by the USB consortium. These can be very expensive. More details on configuration follow.

In order for this device to be advertised over BLE so that Xemu will map it correctly, your PC will need to recognize the device over Bluetooth as a Microsoft Xbox controller. This could, theoretically, only be accomplished by spoofing an official Microsoft device, so solutions for Xemu auto mapping are in the works. 

**Targets**

- Hyperkin DuchesS
- ESP‑IDF 5.5.x, Espressif ESP32‑S3-USB‑OTG dev board.
- Windows 11 and Linux Debian/Ubuntu
- 3.7V 1000mAh LiPo Battery

**Directory Layout**
- `main/esp_hid_device_main.c` — App bootstrap (NVS, GAP, BLE HID initialization)
- `main/hid_gamepad.{c,h}` — BLE HID device and report map
- `main/usb_gip_host.{c,h}` — USB host setup, IN/OUT transfers, GIP frame queue
- `main/input_mapper.{c,h}` — GIP > `gamepad_state_t` > BLE HID report
- `main/esp_hid_gap.{c,h}` — GAP helper (advertising, security callbacks)
- `main/power_manager.{c,h}` — Placeholders for power/battery control

### Set-up/Identity Control

You must set the HID identity for the device to be recognized. It can either be hardcoded in `hid_gamepad.c` or via environment variables at build time in CMakeLists.txt:

- Neutral defaults
  - Advertised name: `CONFIG_XEMUBOX_DEVICE_NAME` (default “XemuPad”)
  - VID/PID: 0xFFFF/0x0000 (unset)


## Pairing with Debian/Ubuntu (bluetoothctl)

- Pair:
  - `bluetoothctl` > `power on` > `agent on` > `default-agent` > `scan on`
  - Wait for device to appear
  - `pair <MAC>` > `trust <MAC>` > `connect <MAC>` > `scan off`
- Verify input mapping:
  - `jstest /dev/input/js0`

## Acknowledgements

- Built on Espressif ESP‑IDF bluetooth template example and GAP helpers.
- Thanks to the SDL community for the GameController mapping model.
