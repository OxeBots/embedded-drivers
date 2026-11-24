#include "i2c_kicker.h"


// ============================================================
// Inicialização
// ============================================================

bool kicker_init(i2c_master_bus_handle_t bus)
{
    if (bus == nullptr)
        return false;

    const i2c_master_bus_handle_t current_bus = I2Cdev::get_bus_handle();

    if (current_bus == bus)
        return true;

    if (current_bus != nullptr)
        return false;

    I2Cdev::init(bus);
    return true;
}


bool kicker_is_initialized(void)
{
    return I2Cdev::get_bus_handle() != nullptr;
}


// ============================================================
// Funções auxiliares de leitura e escrita
// ============================================================

static esp_err_t kicker_read(uint8_t reg, uint8_t *value)
{
    if (value == nullptr)
        return ESP_ERR_INVALID_ARG;

    if (!kicker_is_initialized())
        return ESP_ERR_INVALID_STATE;

    return I2Cdev::readByte(
        KICKER_I2C_ADDRESS,
        reg,
        value
    ) == 1 ? ESP_OK : ESP_FAIL;
}


static esp_err_t kicker_write(uint8_t reg, uint8_t value)
{
    if (!kicker_is_initialized())
        return ESP_ERR_INVALID_STATE;

    return I2Cdev::writeByte(
        KICKER_I2C_ADDRESS,
        reg,
        value
    ) ? ESP_OK : ESP_FAIL;
}


// ============================================================
// Função auxiliar para verificar bits de status
// ============================================================

static bool kicker_check_status(uint8_t status, uint8_t mask)
{
    return (status & mask) != 0;
}


// ============================================================
// Registradores de leitura
// ============================================================

esp_err_t kicker_get_version(uint8_t *version)
{
    return kicker_read(KICKER_REG_VERSION, version);
}


esp_err_t kicker_get_status(uint8_t *status)
{
    return kicker_read(KICKER_REG_BOARD_STATUS, status);
}


esp_err_t kicker_get_voltage(uint8_t *voltage)
{
    return kicker_read(KICKER_REG_VSEC, voltage);
}


esp_err_t kicker_get_setpoint_voltage(uint8_t *setpoint_voltage)
{
    return kicker_read(KICKER_REG_VSEC_SET, setpoint_voltage);
}


esp_err_t kicker_get_kick_duty(uint8_t *kick_duty)
{
    return kicker_read(KICKER_REG_KICK_DUTY, kick_duty);
}


esp_err_t kicker_get_kick_duration(uint8_t *kick_duration)
{
    return kicker_read(KICKER_REG_KICK_DURATION, kick_duration);
}


esp_err_t kicker_get_primary_voltage_1(uint8_t *primary_voltage_1)
{
    return kicker_read(KICKER_REG_VPRI_1, primary_voltage_1);
}


esp_err_t kicker_get_primary_voltage_2(uint8_t *primary_voltage_2)
{
    return kicker_read(KICKER_REG_VPRI_2, primary_voltage_2);
}


// ============================================================
// Registradores de escrita
// ============================================================

esp_err_t kicker_set_setpoint_voltage(uint8_t setpoint_voltage)
{
    if (setpoint_voltage > KICKER_VSEC_MAX)
        return ESP_ERR_INVALID_ARG;

    return kicker_write(KICKER_REG_VSEC_SET, setpoint_voltage);
}


esp_err_t kicker_set_kick_duty(uint8_t kick_duty)
{
    if (kick_duty > KICKER_DUTY_MAX)
        return ESP_ERR_INVALID_ARG;

    return kicker_write(KICKER_REG_KICK_DUTY, kick_duty);
}


esp_err_t kicker_set_kick_duration(uint8_t kick_duration)
{
    return kicker_write(KICKER_REG_KICK_DURATION, kick_duration);
}


// ============================================================
// Registradores de comando
// ============================================================

esp_err_t kicker_reset_board(void)
{
    return kicker_write(
        KICKER_REG_BOARD_RESET,
        KICKER_COMMAND_START
    );
}


esp_err_t kicker_enable_charge(bool enable)
{
    return kicker_write(
        KICKER_REG_CHARGE_ENABLE,
        enable ? KICKER_COMMAND_START : KICKER_COMMAND_STOP
    );
}


esp_err_t kicker_start_kick(void)
{
    return kicker_write(
        KICKER_REG_KICK_START,
        KICKER_COMMAND_START
    );
}


esp_err_t kicker_enable_bleeder(bool enable)
{
    return kicker_write(
        KICKER_REG_BLEEDER_ENABLE,
        enable ? KICKER_COMMAND_START : KICKER_COMMAND_STOP
    );
}


// ============================================================
// Verificação dos bits de status
// ============================================================

bool kicker_is_charging(uint8_t status)
{
    return kicker_check_status(status, KICKER_BOARD_STATUS_CHARGE);
}

bool kicker_is_kicking(uint8_t status)
{
    return kicker_check_status(status, KICKER_BOARD_STATUS_KICK);
}

bool kicker_is_bleeder_enabled(uint8_t status)
{
    return kicker_check_status(status, KICKER_BOARD_STATUS_BLEEDER);
}

bool kicker_has_fault(uint8_t status)
{
    return kicker_check_status(status, KICKER_BOARD_STATUS_FAULT);
}

bool kicker_has_hardware_ov_trip(uint8_t status)
{
    return kicker_check_status(status, KICKER_BOARD_STATUS_HARDWARE_OV_TRIP);
}

bool kicker_has_ov_trip(uint8_t status)
{
    return kicker_check_status(status, KICKER_BOARD_STATUS_OV_TRIP);
}

bool kicker_has_i2c_reg_error(uint8_t status)
{
    return kicker_check_status(status, KICKER_BOARD_STATUS_I2C_LAST_REG_ERROR);
}

bool kicker_has_i2c_value_error(uint8_t status)
{
    return kicker_check_status(status, KICKER_BOARD_STATUS_I2C_LAST_VALUE_ERROR);
}