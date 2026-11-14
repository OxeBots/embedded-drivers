/**
 * @file 74HC595.cpp
 * @brief Driver for the SPI-connected 74HC595 output shift register.
 *
 * The eight outputs are shared by independent tasks (motor direction from the
 * control task, encoder mux selection from the estimator task, and the radio
 * CE line from the receiver task), so every write and read-modify-write is
 * serialized with a mutex.
 */

#include "74HC595.h"

#include <driver/spi_master.h>
#include <esp_log.h>

#include "SPIdev.h"

static const char * TAG = "74HC595";

esp_err_t HC595::init(const Config & config, uint8_t initial_value)
{
    if (m_initialized)
        return ESP_ERR_INVALID_STATE;

    if (config.latch_gpio < 0 || config.latch_gpio >= GPIO_NUM_MAX)
        return ESP_ERR_INVALID_ARG;

    spi_device_interface_config_t device_config = {};
    device_config.clock_speed_hz = static_cast<int>(config.clock_hz);
    device_config.mode = 0;
    device_config.spics_io_num = -1;  // overridden: SPIdev wires the latch as chip select
    device_config.queue_size = 1;
    device_config.flags = SPI_DEVICE_NO_DUMMY;

    if (!SPIdev::register_device(config.device_key, config.latch_gpio, &device_config))
    {
        ESP_LOGE(TAG, "Failed to register 74HC595 on SPI device key %u", config.device_key);
        return ESP_FAIL;
    }

    m_mutex = xSemaphoreCreateMutex();
    if (m_mutex == nullptr)
    {
        ESP_LOGE(TAG, "Failed to create the output access mutex");
        return ESP_ERR_NO_MEM;
    }

    m_config = config;
    m_initialized = true;
    return write(initial_value);
}

esp_err_t HC595::write(uint8_t value)
{
    if (!m_initialized)
        return ESP_ERR_INVALID_STATE;

    if (xSemaphoreTake(m_mutex, portMAX_DELAY) != pdTRUE)
        return ESP_ERR_TIMEOUT;

    esp_err_t err = write_locked(value);
    xSemaphoreGive(m_mutex);
    return err;
}

esp_err_t HC595::update_mask(uint8_t mask, uint8_t value)
{
    if (!m_initialized)
        return ESP_ERR_INVALID_STATE;

    if (xSemaphoreTake(m_mutex, portMAX_DELAY) != pdTRUE)
        return ESP_ERR_TIMEOUT;

    const uint8_t next = static_cast<uint8_t>((m_output & ~mask) | (value & mask));
    esp_err_t err = write_locked(next);
    xSemaphoreGive(m_mutex);
    return err;
}

uint8_t HC595::get_output() const
{
    return m_output;
}

bool HC595::is_initialized() const
{
    return m_initialized;
}

/**
 * @brief Writes one output bit, keeping every other output untouched.
 *
 * This is the pin-write callback a driver uses to toggle a single output (the
 * radio CE line): the full byte is shifted again with only that bit changed.
 * @param context The Pin describing the output bit.
 * @param level 1 sets the bit, 0 clears it.
 */
void HC595::Pin::write(void * context, uint8_t level)
{
    auto * pin = static_cast<Pin *>(context);
    const uint8_t value = level ? pin->mask : 0;
    if (pin->shift_register->update_mask(pin->mask, value) != ESP_OK)
        ESP_LOGE(TAG, "Failed to write output bit 0x%02X", pin->mask);
}

esp_err_t HC595::write_locked(uint8_t value)
{
    // The latch is the device chip select: the peripheral holds it low while
    // the byte shifts in and raises it after the last bit, which updates the
    // storage register in one atomic bus transaction.
    if (!SPIdev::write(m_config.device_key, &value, 1))
        return ESP_FAIL;
    m_output = value;
    return ESP_OK;
}
