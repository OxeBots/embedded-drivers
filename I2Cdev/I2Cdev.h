// I2Cdev library collection - Main I2C device class
// Abstracts bit and byte I2C R/W functions into a convenient class
// EFM32 stub port by Nicolas Baldeck <nicolas@pioupiou.fr>
// Ported to ESP-IDF i2c_master driver by Erick Suzart <ericksuzart@gmail.com>
// Based on Arduino's I2Cdev by Jeff Rowberg <jeff@rowberg.net>
//
// Changelog:
//      2015-01-02 - Initial release
//      2025-11-29 - Ported to ESP-IDF i2c_master driver

// ============================================
// I2Cdev device library code is placed under the MIT license
// Copyright (c) 2015 Jeff Rowberg, Nicolas Baldeck
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
// ===============================================
#ifndef _I2CDEV_H_
#define _I2CDEV_H_

#include <driver/i2c_master.h>
#include <esp_err.h>

#define I2CDEV_DEFAULT_READ_TIMEOUT 1000
#define I2CDEV_MAX_CACHED_DEVICES 8

typedef struct
{
    uint8_t addr;
    i2c_master_dev_handle_t handle;
} i2c_dev_cache_entry_t;

class I2Cdev
{
   public:
    I2Cdev();

    static void init(i2c_master_bus_handle_t bus_handle);
    static i2c_master_bus_handle_t get_bus_handle();

    // Get or create the cached device handle for a 7-bit address.
    // dev_addr: I2C slave device address (7-bit).
    // scl_speed_hz: bus clock for the device when first registered.
    // Returns the device handle, or NULL on failure.
    static i2c_master_dev_handle_t get_device_handle(uint8_t dev_addr,
                                                     uint32_t scl_speed_hz = 400000);

    static int8_t read_bit(uint8_t dev_addr,
                           uint8_t reg_addr,
                           uint8_t bit_num,
                           uint8_t * data,
                           uint16_t timeout = I2Cdev::read_timeout);
    static int8_t read_bits(uint8_t dev_addr,
                            uint8_t reg_addr,
                            uint8_t bit_start,
                            uint8_t length,
                            uint8_t * data,
                            uint16_t timeout = I2Cdev::read_timeout);
    static int8_t read_byte(uint8_t dev_addr,
                            uint8_t reg_addr,
                            uint8_t * data,
                            uint16_t timeout = I2Cdev::read_timeout);
    static int8_t read_word(uint8_t dev_addr,
                            uint8_t reg_addr,
                            uint16_t * data,
                            uint16_t timeout = I2Cdev::read_timeout);
    static int8_t read_bytes(uint8_t dev_addr,
                             uint8_t reg_addr,
                             uint8_t length,
                             uint8_t * data,
                             uint16_t timeout = I2Cdev::read_timeout);

    static bool write_bit(uint8_t dev_addr, uint8_t reg_addr, uint8_t bit_num, uint8_t data);
    static bool write_bits(
      uint8_t dev_addr, uint8_t reg_addr, uint8_t bit_start, uint8_t length, uint8_t data);
    static bool write_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data);
    static bool write_word(uint8_t dev_addr, uint8_t reg_addr, uint16_t data);
    static bool write_bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t length, uint8_t * data);

    static uint16_t read_timeout;

   private:
    static i2c_master_bus_handle_t global_bus_handle;

    static i2c_dev_cache_entry_t device_cache[I2CDEV_MAX_CACHED_DEVICES];
    static uint8_t cached_count;

    static esp_err_t perform_transaction(uint8_t dev_addr,
                                         uint8_t * write_buffer,
                                         size_t write_len,
                                         uint8_t * read_buffer,
                                         size_t read_len);
};

#endif  // _I2CDEV_H_
