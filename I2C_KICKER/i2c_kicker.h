#ifndef I2C_KICKER_H
#define I2C_KICKER_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "driver/i2c_master.h"
#include "kicker_i2c_defs.h"
#include "I2Cdev.h"

#define KICKER_I2C_ADDRESS 0x27U

// Inicialização
bool kicker_init(i2c_master_bus_handle_t i2c);
bool kicker_is_initialized(void);

// Leitura de registradores
esp_err_t kicker_get_version(uint8_t *version);
esp_err_t kicker_get_status(uint8_t *status);
esp_err_t kicker_get_voltage(uint8_t *voltage);
esp_err_t kicker_get_setpoint_voltage(uint8_t *setpoint_voltage);
esp_err_t kicker_get_kick_duty(uint8_t *kick_duty);
esp_err_t kicker_get_kick_duration(uint8_t *kick_duration);
esp_err_t kicker_get_primary_voltage_1(uint8_t *primary_voltage_1);
esp_err_t kicker_get_primary_voltage_2(uint8_t *primary_voltage_2);

// Escrita de registradores
esp_err_t kicker_set_setpoint_voltage(uint8_t setpoint_voltage);
esp_err_t kicker_set_kick_duty(uint8_t kick_duty);
esp_err_t kicker_set_kick_duration(uint8_t kick_duration);

// Comandos
esp_err_t kicker_reset_board(void);
esp_err_t kicker_enable_charge(bool enable);
esp_err_t kicker_start_kick(void);
esp_err_t kicker_enable_bleeder(bool enable);

// Verificação de status
// Verificação dos bits de status
bool kicker_is_charging(uint8_t status);
bool kicker_is_kicking(uint8_t status);
bool kicker_is_bleeder_enabled(uint8_t status);
bool kicker_has_fault(uint8_t status);
bool kicker_has_hardware_ov_trip(uint8_t status);
bool kicker_has_ov_trip(uint8_t status);
bool kicker_has_i2c_reg_error(uint8_t status);
bool kicker_has_i2c_value_error(uint8_t status);


#endif // I2C_KICKER_H