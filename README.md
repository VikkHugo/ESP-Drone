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

The serial communication path used by the firmware remains the same; the new sensor fusion only updates the state estimation source used by the control stack.

### Pin Map

Default pin mapping is defined in `main/Kconfig.projbuild`. The sensor bus uses `I2C0`.

#### ESPlane V1

* `I2C0 SDA` = `GPIO 21`
* `I2C0 SCL` = `GPIO 22`
* `I2C1 SDA` = `GPIO 16`
* `I2C1 SCL` = `GPIO 17`

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