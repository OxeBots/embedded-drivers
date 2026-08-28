# embedded-drivers

Reusable hardware libraries for ESP-IDF / PlatformIO projects: discrete chip
drivers plus I2C/SPI bus helpers. Each top-level folder is one PlatformIO
library; expose this folder through lib_extra_dirs so the Library Dependency
Finder scans it one level deep.

## Chip drivers

| Folder | Device |
| --- | --- |
| 74HC595 | 8-bit shift register |
| ADXL345 | 3-axis accelerometer (I2C) |
| AS5600 | Magnetic rotary encoder (I2C) |
| BL48250 | DC motor driver |
| CD4052B | Differential analog multiplexer |
| IMUGY85 | GY-85 IMU breakout (ADXL345 + ITG3200 + QMC5883L) |
| ITG3200 | 3-axis gyroscope (I2C) |
| NRF24L01 | 2.4 GHz radio (SPI, wraps esp-idf-mirf) |
| QMC5883L | 3-axis magnetometer (I2C) |
| VL53L5CX | 8x8 time-of-flight sensor (wraps VL53L5CX-Library) |

## Bus helpers

- I2Cdev: I2C transaction helpers used by the I2C drivers.
- SPIdev: SPI transaction helpers used by the SPI drivers.

## Vendor libraries (git submodules)

- esp-idf-mirf: Nordic nRF24L01 driver used by NRF24L01.
- VL53L5CX-Library: ST ULD API used by VL53L5CX.

They are built as ESP-IDF components, not PIO libraries, because their
Kconfig options (RF pins/channel, sensor result layout) must stay in the
host sdkconfig. In an ESP-IDF project, register them before including
project.cmake:

    set(EXTRA_COMPONENT_DIRS
        "${CMAKE_CURRENT_LIST_DIR}/components/embedded-drivers/esp-idf-mirf"
        "${CMAKE_CURRENT_LIST_DIR}/components/embedded-drivers/VL53L5CX-Library")

and keep the PlatformIO LDF from double-building them:

    lib_ignore =
        esp-idf-mirf
        VL53L5CX-Library

Clone with --recurse-submodules (or run git submodule update --init
--recursive after cloning).