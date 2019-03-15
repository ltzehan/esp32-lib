# esp32

This library is based off the ESP8266 library found in mbed OS for interfacing with the ESP32 over UART using AT commands.

### AT Firmware

The firmware required for AT interfacing on the ESP32 can be found under `esp32-AT`. Run the command in `flash.txt` using [esptool](https://github.com/espressif/esptool) and replace `<PORT>` with the serial port connected to the ESP32.

### Library

This library was built on mbed OS v5.11 and is located in `esp32-lib`. The `lib-test` folder contains files for testing socket connection functionality over TCP and UDP.

