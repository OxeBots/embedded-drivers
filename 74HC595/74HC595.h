#ifndef _74HC595_H_
#define _74HC595_H_

#include <driver/gpio.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <stdint.h>

class HC595
{
   public:
    static constexpr uint8_t DEFAULT_DEVICE_KEY = 0xFE;

    struct Config
    {
        gpio_num_t latch_gpio;
        uint8_t device_key = DEFAULT_DEVICE_KEY;
        uint32_t clock_hz = 1000000;
    };

    // One output bit. Drivers that toggle a single pin (the radio CE line) call
    // write() with the Pin as context, so they do not depend on this class.
    struct Pin
    {
        HC595 * shift_register;
        uint8_t mask;
        static void write(void * context, uint8_t level);
    };

    HC595() = default;

    esp_err_t init(const Config & config, uint8_t initial_value = 0);
    esp_err_t write(uint8_t value);
    esp_err_t update_mask(uint8_t mask, uint8_t value);

    uint8_t get_output() const;
    bool is_initialized() const;

   private:
    Config m_config = {};
    uint8_t m_output = 0;
    bool m_initialized = false;
    // Motor direction, the encoder mux and the radio CE all write the output
    // byte from different tasks; the mutex keeps their updates atomic.
    SemaphoreHandle_t m_mutex = nullptr;

    // Shifts the full byte and pulses the latch. Caller must hold m_mutex.
    esp_err_t write_locked(uint8_t value);
};

#endif  // _74HC595_H_
