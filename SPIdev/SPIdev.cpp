/**
 * @file SPIdev.cpp
 * @brief Shared raw SPI transactions for devices on one ESP-IDF SPI bus.
 */

#include "SPIdev.h"

#include <esp_log.h>
#include <sdkconfig.h>
#include <stdlib.h>

static const char * TAG = "SPIdev";

#if CONFIG_SPI2_HOST
static constexpr spi_host_device_t SHARED_SPI_HOST = SPI2_HOST;
#elif CONFIG_SPI3_HOST
static constexpr spi_host_device_t SHARED_SPI_HOST = SPI3_HOST;
#else
#error "Select SPI2_HOST or SPI3_HOST in menuconfig"
#endif

spi_dev_cache_entry_t SPIdev::device_cache[SPIDEV_MAX_CACHED_DEVICES] = {};
uint8_t SPIdev::cached_count = 0;
SemaphoreHandle_t SPIdev::transaction_mutex = nullptr;

esp_err_t SPIdev::init()
{
    if (is_initialized())
        return ESP_OK;

    SemaphoreHandle_t mutex = xSemaphoreCreateMutex();
    if (mutex == nullptr)
    {
        ESP_LOGE(TAG, "Failed to create the SPI transaction mutex");
        return ESP_ERR_NO_MEM;
    }

    spi_bus_config_t config = {};
    config.mosi_io_num = CONFIG_MOSI_GPIO;
    config.miso_io_num = CONFIG_MISO_GPIO;
    config.sclk_io_num = CONFIG_SCLK_GPIO;
    config.quadwp_io_num = -1;
    config.quadhd_io_num = -1;
    const esp_err_t result = spi_bus_initialize(SHARED_SPI_HOST, &config, SPI_DMA_CH_AUTO);
    if (result != ESP_OK)
    {
        vSemaphoreDelete(mutex);
        return result;
    }

    transaction_mutex = mutex;
    return ESP_OK;
}

bool SPIdev::is_initialized()
{
    return transaction_mutex != nullptr;
}

bool SPIdev::register_device(uint8_t device_key,
                             gpio_num_t cs_gpio,
                             const spi_device_interface_config_t * dev_config)
{
    if (!is_initialized() || dev_config == nullptr)
        return false;

    spi_device_interface_config_t config = *dev_config;
    // The chip select belongs to the device: the peripheral asserts it before
    // the first bit and releases it after the last one, while the bus mutex
    // excludes every other bus user.
    config.spics_io_num = (cs_gpio == GPIO_NUM_NC) ? -1 : static_cast<int>(cs_gpio);

    for (uint8_t i = 0; i < cached_count; ++i)
    {
        if (device_cache[i].device_key != device_key)
            continue;

        if (device_cache[i].handle != nullptr)
            return false;

        device_cache[i].config = config;
        return true;
    }

    if (cached_count >= SPIDEV_MAX_CACHED_DEVICES)
    {
        ESP_LOGE(TAG, "SPI device cache is full");
        return false;
    }

    device_cache[cached_count].device_key = device_key;
    device_cache[cached_count].config = config;
    ++cached_count;
    return true;
}

spi_device_handle_t SPIdev::get_device_handle(uint8_t device_key)
{
    if (!is_initialized())
    {
        ESP_LOGE(TAG, "SPIdev is not initialized");
        return nullptr;
    }

    for (uint8_t i = 0; i < cached_count; ++i)
    {
        auto & entry = device_cache[i];
        if (entry.device_key != device_key)
            continue;

        if (entry.handle != nullptr)
            return entry.handle;

        spi_device_interface_config_t config = entry.config;
        if (config.queue_size == 0)
            config.queue_size = 1;

        esp_err_t err = spi_bus_add_device(SHARED_SPI_HOST, &config, &entry.handle);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to add device key %u: %s", device_key, esp_err_to_name(err));
            return nullptr;
        }
        return entry.handle;
    }

    ESP_LOGE(TAG, "Device key %u was never registered", device_key);
    return nullptr;
}

esp_err_t SPIdev::perform_transaction(uint8_t device_key,
                                      const uint8_t * tx_data,
                                      uint8_t * rx_data,
                                      size_t length)
{
    if (length == 0 || (tx_data == nullptr && rx_data == nullptr))
        return ESP_ERR_INVALID_ARG;

    if (transaction_mutex == nullptr)
        return ESP_ERR_INVALID_STATE;

    if (xSemaphoreTake(transaction_mutex, portMAX_DELAY) != pdTRUE)
        return ESP_ERR_TIMEOUT;

    spi_device_handle_t handle = get_device_handle(device_key);
    if (handle == nullptr)
    {
        xSemaphoreGive(transaction_mutex);
        return ESP_FAIL;
    }

    uint8_t * zero_tx = nullptr;
    if (tx_data == nullptr)
    {
        zero_tx = static_cast<uint8_t *>(calloc(length, sizeof(uint8_t)));
        if (zero_tx == nullptr)
        {
            xSemaphoreGive(transaction_mutex);
            return ESP_ERR_NO_MEM;
        }
        tx_data = zero_tx;
    }

    spi_transaction_t transaction = {};
    transaction.length = length * 8;
    transaction.tx_buffer = tx_data;
    transaction.rx_buffer = rx_data;

    esp_err_t err = spi_device_transmit(handle, &transaction);
    free(zero_tx);
    xSemaphoreGive(transaction_mutex);
    return err;
}

uint8_t SPIdev::shift(uint8_t device_key, uint8_t data)
{
    uint8_t received = 0;
    uint8_t transmitted = data;
    if (perform_transaction(device_key, &transmitted, &received, 1) != ESP_OK)
        return 0;
    return received;
}

bool SPIdev::read(uint8_t device_key, const uint8_t * tx_data, uint8_t * rx_data, size_t length)
{
    if (rx_data == nullptr || length == 0)
        return false;
    return perform_transaction(device_key, tx_data, rx_data, length) == ESP_OK;
}

bool SPIdev::write(uint8_t device_key, const uint8_t * tx_data, size_t length)
{
    if (tx_data == nullptr || length == 0)
        return false;
    return perform_transaction(device_key, tx_data, nullptr, length) == ESP_OK;
}
