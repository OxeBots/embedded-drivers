#ifndef _SPIDEV_H_
#define _SPIDEV_H_

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <stddef.h>
#include <stdint.h>

#define SPIDEV_MAX_CACHED_DEVICES 8

typedef struct
{
    uint8_t device_key;
    spi_device_handle_t handle;
    spi_device_interface_config_t config;
} spi_dev_cache_entry_t;

class SPIdev
{
   public:
    SPIdev() = default;

    // Creates the shared SPI bus from the sdkconfig pins and host selection,
    // and resets the device registry. Drivers only register devices on the
    // bus after this call. Mirrors I2Cdev::init() as the single bus owner.
    static void init();

    // Registers a device before its first transaction. The chip select is
    // part of the device: SPIdev wires cs_gpio into the peripheral config so
    // every transaction asserts it before the first bit and releases it
    // after the last one (the 74HC595 latch and the radio CSN both work this
    // way). Pass GPIO_NUM_NC for a device without a chip select.
    static bool register_device(uint8_t device_key,
                                gpio_num_t cs_gpio,
                                const spi_device_interface_config_t * dev_config);

    static bool is_initialized();

    static uint8_t shift(uint8_t device_key, uint8_t data);
    static bool read(uint8_t device_key, const uint8_t * tx_data, uint8_t * rx_data, size_t length);
    static bool write(uint8_t device_key, const uint8_t * tx_data, size_t length);

   private:
    static spi_host_device_t global_host_id;
    static bool initialized;
    static spi_dev_cache_entry_t device_cache[SPIDEV_MAX_CACHED_DEVICES];
    static uint8_t cached_count;
    static SemaphoreHandle_t transaction_mutex;

    static spi_device_handle_t get_device_handle(uint8_t device_key);
    static esp_err_t perform_transaction(uint8_t device_key,
                                         const uint8_t * tx_data,
                                         uint8_t * rx_data,
                                         size_t length);
};

#endif  // _SPIDEV_H_
