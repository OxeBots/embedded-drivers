#ifndef _VL53L5CX_H_
#define _VL53L5CX_H_

#include <stdint.h>

#include "I2Cdev.h"
#include "vl53l5cx_api.h"

class VL53L5CX
{
   public:
    static constexpr uint8_t DEFAULT_ADDRESS = 0x52;

    VL53L5CX(uint8_t address = DEFAULT_ADDRESS);
    ~VL53L5CX();

    esp_err_t init();
    bool test_connection();
    uint8_t set_resolution(uint8_t resolution);
    uint8_t set_ranging_frequency(uint8_t frequency_hz);
    uint8_t set_ranging_mode(uint8_t mode);
    uint8_t set_target_order(uint8_t order);
    uint8_t set_sharpener_percent(uint8_t percent);
    uint8_t start_ranging();
    uint8_t check_data_ready(uint8_t * is_ready);
    uint8_t get_ranging_data(VL53L5CX_ResultsData * results);
    uint8_t stop_ranging();

   private:
    uint8_t m_dev_addr;
    VL53L5CX_Configuration m_dev;
    bool m_initialized = false;
    bool m_ranging = false;
};

#endif  // _VL53L5CX_H_
