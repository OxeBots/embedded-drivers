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

VL53L5CX and NRF24L01 expect the vendor libraries (VL53L5CX-Library,
esp-idf-mirf) to exist in the host project.
