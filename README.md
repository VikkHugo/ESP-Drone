## ESP-Drone

* [中文](./README_cn.md)

### Introduction

**ESP-Drone** is an open source solution based on Espressif ESP32/ESP32-S2/ESP32-S3 Wi-Fi chip, which can be controlled by a mobile APP or gamepad over **Wi-Fi** connection. ESP-Drone comes with **simple hardware**, **clear and extensible code architecture**, and therefore this project can be used in **STEAM education** and other fields. The main code is ported from **Crazyflie** open source project with **GPL3.0** protocol.

This fork includes support for the **GY-87** sensor stack on the legacy flight pipeline:
* **MPU6050** for accelerometer and gyroscope
* **HMC5883L-compatible magnetometer** for yaw correction
* **BMP180** for pressure-based altitude estimation

The estimator combines gyro, accelerometer, and magnetometer data for a more stable yaw estimate while keeping the existing serial/log path intact.

> Currently support ESP32、ESP32S2、ESP32S3, please using ESP-IDF [release/v4.4](https://docs.espressif.com/projects/esp-idf/en/release-v4.4/esp32s2/get-started/index.html) branch as your develop environment

![ESP-Drone](./docs/_static/espdrone_s2_v1_2_2.png)

For more information, please check the sections below:
* **Getting Started**: [Getting Started](https://docs.espressif.com/projects/espressif-esp-drone/zh_CN/latest/gettingstarted.html)
* **Hardware Schematic**：[Hardware](https://docs.espressif.com/projects/espressif-esp-drone/zh_CN/latest/_static/ESP32_S2_Drone_V1_2/SCH_Mainboard_ESP32_S2_Drone_V1_2.pdf)
* **iOS APP Source code**: [ESP-Drone-iOS](https://github.com/EspressifApps/ESP-Drone-iOS)
* **Android APP Source code**: [ESP-Drone-Android](https://github.com/EspressifApps/ESP-Drone-Android)

### Features

1. Stabilize Mode
2. Height-hold Mode
3. Position-hold Mode
4. APP Control
5. CFclient Supported
6. GY-87 sensor support with fused attitude estimation
7. Existing telemetry/log output for roll, pitch, yaw, altitude, gyro and magnetometer values

### Recent Updates (This Fork)

The following updates were added and validated during bring-up on Lolin D32:

* Magnetometer auto-detection for both original and clone chips in GY-87:
	* `HMC5883L` (address `0x1E`)
	* `QMC5883L` clone fallback (address `0x0D`)
* Sensor startup no longer blocks `Ready to fly` when magnetometer probe/self-test fails.
	* The firmware now logs the failure and continues startup for initial flight validation.
* Added safe serial telemetry output for external microcontroller integration:
	* Data production is done in the stabilizer loop at 25 Hz.
	* Transmission is asynchronous via queue + dedicated low-priority UART task.
	* This avoids blocking the flight control loop.
* Serial telemetry task stability fix:
	* Increased SERIAL_TLM task stack budget.
	* Moved telemetry line buffer out of task local stack.
	* This prevents stack overflow resets while preserving non-blocking telemetry behavior.
* Flash size header alignment fix:
	* Firmware image header is now configured for `4MB` flash on ESP32 Lolin D32.
	* This removes boot warning about detected flash size (`4096k`) vs binary header (`2048k`).
* ADC driver modernization for ESP-IDF 5.x:
	* Migrated from legacy ADC driver APIs to `esp_adc/adc_oneshot.h` + calibration schemes.
	* This removes the deprecation warning for legacy ADC while preserving battery/voltage readings.

```text
acc_x:acc_y:acc_z:gyro_x:gyro_y:gyro_z:mag_x:mag_y:mag_z:altitude:pressao:roll:pitch:yaw
```

The stream is generated at 25 Hz and sent through UART1 in a non-blocking path.

### Active Validation Config

For the current `sdkconfig` used in this workspace:

* `MPU_PIN_INT = GPIO34`

Note: to implement Height-hold/Position-hold mode, extension boards are needed. For more information, see Hardware Reference.

### Sensor Stack

The active sensor implementation for the drone profile is:

* `MPU6050 + HMC5883L + BMP180`

The firmware probes the sensors on boot and exposes their values through the existing logging/telemetry groups:

* `stateEstimate.roll`, `stateEstimate.pitch`, `stateEstimate.yaw`
* `baro.asl`, `baro.temp`, `baro.pressure`
* `mag.x`, `mag.y`, `mag.z`
* `gyro.x`, `gyro.y`, `gyro.z`
* `acc.x`, `acc.y`, `acc.z`

The firmware now also supports a dedicated UART telemetry path for external MCU integration, while preserving the original Wi-Fi/CRTP control path.

### Serial Telemetry (ESP32 -> ESP8266)

This fork provides serial telemetry for an external controller (for example ESP8266) using UART1.

#### ESP32 (drone) side

* UART: `UART1`
* TX pin: `GPIO16`
* RX pin: `GPIO17`
* Baud rate: `115200`
* Frame: `8N1`
* Telemetry rate: `25 Hz`

#### ESP8266 side recommended settings

* Use hardware serial (`Serial`) at `115200` baud, `SERIAL_8N1`
* Ensure common ground between ESP32 and ESP8266
* Connect `ESP32 GPIO16 (TX)` -> `ESP8266 RX`
* Connect `ESP32 GPIO17 (RX)` <- `ESP8266 TX` (optional, if return channel is needed)
* Both devices are `3.3V` logic (do not use 5V UART levels)

#### Important integration notes

* Avoid using SoftwareSerial on ESP8266 for this stream if possible.
* If ESP8266 USB logging is needed, use a board/setup that keeps reliable hardware RX available.
* The telemetry queue keeps the latest frame; if receiver is slower, older frames are dropped to protect flight timing.

With the serial settings and wiring documented above, this implementation guarantees an appropriate telemetry communication path to ESP8266 while keeping flight-control timing independent of UART transmission (non-blocking telemetry task + queue overwrite policy).

### Pin Map

Default pin mapping is defined in `main/Kconfig.projbuild`. The sensor bus uses `I2C0`.

#### ESPlane V1

* `I2C0 SDA` = `GPIO 21`
* `I2C0 SCL` = `GPIO 22`
* `I2C1 SDA` = `GPIO 19`
* `I2C1 SCL` = `GPIO 2`
* `MPU INT` = `GPIO 35`

This frees `GPIO 16` and `GPIO 17` for UART1 wiring.

### Lolin D32 Adapted Pinout (ESP32)

This project is adapted for **Lolin D32** using the `ESPlane V1` target profile.

#### Interfaces

* `I2C0 (main IMU bus)` -> `SDA GPIO 21`, `SCL GPIO 22`
* `I2C1 (deck/aux bus)` -> `SDA GPIO 19`, `SCL GPIO 2`
* `UART1 (reserved user serial)` -> `TX GPIO 16`, `RX GPIO 17`
* `Console/flash serial` -> `UART0` (default ESP32 console path)

#### Motors (Lolin D32 profile)

* `M1` -> `GPIO 4`
* `M2` -> `GPIO 33`
* `M3` -> `GPIO 32`
* `M4` -> `GPIO 25`

#### Notes for this adaptation

* `I2C1` was moved away from `GPIO 16/17` to avoid conflict with UART1.
* `MPU INT` was remapped to `GPIO 35` to keep all active signals conflict-free.
* This keeps motor outputs unchanged and preserves the existing telemetry/control flow.

#### ESPlane V2 S2

* `I2C0 SDA` = `GPIO 11`
* `I2C0 SCL` = `GPIO 10`
* `I2C1 SDA` = `GPIO 40`
* `I2C1 SCL` = `GPIO 41`

#### ESP32_S2_DRONE_V1_2

* `I2C0 SDA` = `GPIO 11`
* `I2C0 SCL` = `GPIO 10`
* `I2C1 SDA` = `GPIO 40`
* `I2C1 SCL` = `GPIO 41`

If you wire the GY-87 to the main sensor bus, use `I2C0`. Keep the UART pins free for console/flash/debug use so telemetry is not affected.

### Build and Flash

This repository uses **ESP-IDF** and **CMake/Ninja**.

Recommended setup:

1. Install the ESP-IDF version required by your target branch.
2. Export the ESP-IDF environment.
3. Build from the repository root.
4. Flash the resulting image to the board.

Common commands:

```bash
idf.py build
idf.py -p COMx flash
idf.py -p COMx monitor
```

If you are publishing this fork to GitHub, keep the `build/` directory out of version control and commit only the source tree, configuration, and documentation changes.

### Suggested Repository Hygiene

Before pushing to GitHub:

* Remove generated artifacts from the working tree.
* Verify `git status` is clean except for intentional source changes.
* Keep the README aligned with the supported hardware and pinout.
* Include a short note in the commit history describing the GY-87 sensor support and yaw fusion update.

### Third Party Copyrighted Code

Additional third party copyrighted code is included under the following licenses.

| Component | License | Origin |Commit ID |
| :---:  | :---:  | :---:  |:---: |
| core/crazyflie | GPL3.0  |[Crazyflie](https://github.com/bitcraze/crazyflie-firmware) |tag_2021_01 b448553|
| lib/dsp_lib |  | [esp32-lin](https://github.com/whyengineer/esp32-lin/tree/master/components/dsp_lib) |6fa39f4c|

### Support Policy

From December 2022, we will offer limited support on this project, but Pull Request is still welcomed!

### THANKS

1. Thanks to Bitcraze for the great [Crazyflie project](https://www.bitcraze.io/%20).
2. Thanks to Espressif for the powerful [ESP-IDF framework](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s2/get-started/index.html).
3. Thanks to WhyEngineer for the useful [ESP-DSP lib](https://github.com/whyengineer/esp32-lin/tree/master/components/dsp_lib).