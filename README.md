# XemuBox BT Dongle (v0.0.1)

This project is based on Espressif’s HID device example (IDF v5.5.1). It targets an ESP32‑S3 USB‑OTG dev board acting as a Bluetooth HID Gamepad while reading inputs from a Hyperkin DuchesS controller over USB host (converts Xbox's GIP → HID mapping). This is designed to pair with my XemuBox emulator machine.


## Layout

- `main/esp_hid_device_main.c`: minimal app bootstrapping (NVS, GAP, BLE HID init, start tasks)
- `main/hid_gamepad.{c,h}`: BLE HID device with a simple Gamepad report (16 buttons + 2 sticks)
- `main/usb_gip_host.{c,h}`: USB host stub to read DuchesS frames (to be implemented)
- `main/input_mapper.{c,h}`: stub mapper from GIP frames → `gamepad_state_t` → HID report
- `main/power_manager.{c,h}`: stub for battery monitoring and 5V host power control
- `main/esp_hid_gap.{c,h}`: GAP helper from upstream example (kept for BLE advertising/init)

## Build & Flash

1) Select target (ESP32‑S3 recommended):
   `idf.py set-target esp32s3`
2) Build/flash/monitor:
   `idf.py -p PORT flash monitor`

After boot, the device advertises as `XemuPad` (BLE HID). No inputs are sent yet; USB host and mapping are placeholders.

## Next Steps

- USB Host: implement TinyUSB/USB Host class init and device enumeration for DuchesS.
- GIP Parser: parse incoming endpoints into a normalized controller state.
- Mapping: translate parsed GIP to `gamepad_state_t` and call `hid_gamepad_send_state`.
- Power: wire battery ADC and USB 5V enable GPIO; manage wall/battery routing for host mode.

Notes
- HID descriptor can be extended later (triggers, D‑pad/Hat, vendor usage).
- If using Bluedroid instead of NimBLE, the project still compiles; current focus is BLE HID.
