/**
 * @file AS5600.cpp
 * @brief Implementation of the AS5600 magnetic rotary encoder driver.
 */

#include "AS5600.h"

#include <algorithm>
#include <cmath>

#include "I2Cdev.h"
#include "NVSManager.h"

static const char * TAG = "AS5600";

namespace
{
/**
 * @brief Median of a small fixed-size window (insertion-sort copy).
 *
 * Used to filter the calibration tracking samples so one isolated dropout or
 * glitch sample cannot pin a range endpoint.
 */
template <typename T, size_t N>
T window_median(const std::array<T, N> & values)
{
    std::array<T, N> sorted = values;
    for (size_t i = 1; i < N; ++i)
    {
        const T value = sorted[i];
        size_t j = i;
        while (j > 0 && sorted[j - 1] > value)
        {
            sorted[j] = sorted[j - 1];
            --j;
        }
        sorted[j] = value;
    }
    return sorted[N / 2];
}

/**
 * @brief Extracts the output stage from a CONF_H register value.
 */
AS5600::OutputStage output_stage_from_config(uint16_t config)
{
    // Bits 5:4 of the configuration register.
    return static_cast<AS5600::OutputStage>((config >> 4) & 0x03);
}
}  // namespace

/**
 * @brief Constructor for AS5600.
 * @param channel ADC channel for analog reading.
 * @param cali_handle ADC calibration handle.
 * @param voltage_calibrated Whether the ADC is voltage-calibrated.
 * @param unit ADC unit (default ADC_UNIT_1).
 * @param bitwidth ADC bitwidth (default ADC_BITWIDTH_12).
 */
AS5600::AS5600(adc_channel_t channel,
               adc_cali_handle_t cali_handle,
               bool voltage_calibrated,
               adc_unit_t unit,
               adc_bitwidth_t bitwidth)
: m_bitwidth(bitwidth),
  m_channel(channel),
  m_cali_handle(cali_handle),
  m_is_voltage_calibrated(voltage_calibrated)
{
}

/**
 * @brief Initializes the AS5600 device via I2C. Validates connection.
 * @return ESP_OK on success, ESP_FAIL on failure, ESP_ERR_INVALID_STATE if already initialized.
 */
esp_err_t AS5600::init_i2c()
{
    if (m_i2c_initialized)
        return ESP_ERR_INVALID_STATE;

    uint16_t test_conf;
    if (read_config_register(&test_conf) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to communicate with AS5600 sensor via I2C.");
        return ESP_FAIL;
    }

    m_i2c_initialized = true;
    ESP_LOGI(TAG, "AS5600 I2C interface initialized successfully.");
    return ESP_OK;
}

bool AS5600::is_i2c_initialized() const
{
    return m_i2c_initialized;
}

/**
 * @brief Sets the analog output mode (full, reduced, PWM).
 * @param stage Selected OutputStage enum.
 * @return ESP_OK on success.
 */
esp_err_t AS5600::set_output_stage(OutputStage stage)
{
    if (!m_i2c_initialized)
        return ESP_ERR_INVALID_STATE;

    uint16_t config;
    esp_err_t ret = read_config_register(&config);
    if (ret != ESP_OK)
        return ret;

    config &= ~0x0030;  // Clear bits 4 and 5
    config |= (static_cast<uint8_t>(stage) << 4);

    return write_config_register(config);
}

/**
 * @brief Short readable name of an output stage.
 * @param stage Selected OutputStage enum.
 * @return "FULL", "REDUCED", "PWM", or "UNKNOWN" for a value outside the enum.
 */
const char * AS5600::stage_name(OutputStage stage)
{
    switch (stage)
    {
        case OutputStage::ANALOG_FULL:
            return "FULL";
        case OutputStage::ANALOG_REDUCED:
            return "REDUCED";
        case OutputStage::DIGITAL_PWM:
            return "PWM";
    }
    return "UNKNOWN";
}

/**
 * @brief Sets the internal slow filter step.
 * @param filter Selected SlowFilter enum.
 * @return ESP_OK on success.
 */
esp_err_t AS5600::set_slow_filter(SlowFilter filter)
{
    if (!m_i2c_initialized)
        return ESP_ERR_INVALID_STATE;

    uint16_t config;
    esp_err_t ret = read_config_register(&config);
    if (ret != ESP_OK)
        return ret;

    config &= ~0x0300;  // Clear bits 8 and 9
    config |= (static_cast<uint8_t>(filter) << 8);

    return write_config_register(config);
}

/**
 * @brief Sets the internal fast filter threshold.
 * @param threshold Selected FastFilter enum.
 * @return ESP_OK on success.
 */
esp_err_t AS5600::set_fast_filter(FastFilter threshold)
{
    if (!m_i2c_initialized)
        return ESP_ERR_INVALID_STATE;

    uint16_t config;
    esp_err_t ret = read_config_register(&config);
    if (ret != ESP_OK)
        return ret;

    config &= ~0x1C00;  // Clear bits 10, 11, and 12
    config |= (static_cast<uint8_t>(threshold) << 10);

    return write_config_register(config);
}

/**
 * @brief Permanently burns settings into the AS5600 OTP memory. Can only be performed 3 times per
 * chip.
 * @return ESP_OK on success.
 */
esp_err_t AS5600::burn_settings()
{
    if (!m_i2c_initialized)
        return ESP_ERR_INVALID_STATE;

    ESP_LOGW(TAG, "Burning settings to AS5600 OTP memory. This is permanent!");
    uint8_t cmd = static_cast<uint8_t>(Cmd::BURN);
    if (!I2Cdev::write_byte(AS5600_ADDR, static_cast<uint8_t>(Register::BURN), cmd))
        return ESP_FAIL;

    vTaskDelay(pdMS_TO_TICKS(10));
    return ESP_OK;
}

/**
 * @brief Retrieves the raw magnetic angle using I2C (instead of ADC).
 * @param angle Pointer to output variable.
 * @return ESP_OK on success.
 */
esp_err_t AS5600::get_i2c_raw_angle(uint16_t * angle)
{
    if (!m_i2c_initialized)
        return ESP_ERR_INVALID_STATE;

    return read_raw_angle(angle);
}

/**
 * @brief Reads the raw magnetic angle over I2C without per-instance state.
 *
 * The caller must have routed this encoder onto the bus first (for example by
 * selecting its CD4052B channel). Used by the wiring calibration, which polls
 * every encoder position while a motor spins.
 *
 * @param angle Output for the 12-bit raw angle (0..4095).
 * @return ESP_OK on success, ESP_FAIL when the read fails.
 */
esp_err_t AS5600::read_raw_angle(uint16_t * angle)
{
    if (angle == nullptr)
        return ESP_ERR_INVALID_ARG;

    if (I2Cdev::read_word(AS5600_ADDR, static_cast<uint8_t>(Register::RAW_ANGLE_H), angle) != 0)
        return ESP_FAIL;

    *angle &= 0x0FFF;
    return ESP_OK;
}

/**
 * @brief Converts one published raw ADC block mean into the absolute angle.
 *
 * The ADC stream owns the oversampling and hands over one block mean
 * per dispatch; this driver only reports the raw measurement and leaves the
 * higher level estimation (filtering, velocity) to the caller.
 *
 * @param raw_adc_value Raw block mean delivered by the ADC stream.
 * @param measured_angle_rad Output for the measured angle in radians.
 */
void AS5600::process_new_reading(uint16_t raw_adc_value, float * measured_angle_rad)
{
    m_last_avg_value = raw_adc_value;

    const uint16_t raw_min = m_min_raw.load();
    const uint16_t raw_max = m_max_raw.load();
    const bool use_raw_range = static_cast<int>(raw_max) - static_cast<int>(raw_min) >=
                               static_cast<int>(MIN_VALID_RAW_RANGE);
    const bool calibrating = m_is_calibrating.load();
    int current_voltage_mv = 0;
    if (m_is_voltage_calibrated && (calibrating || !use_raw_range))
        adc_cali_raw_to_voltage(m_cali_handle, m_last_avg_value, &current_voltage_mv);

    if (calibrating)
    {
        // Ignore rail/glitch block means for both ranges: the REDUCED output
        // band never reaches the ADC rails, so an intermittent analog input
        // reading 0 (or full scale) must not define a calibration endpoint.
        if (raw_adc_value > RAW_RAIL_LOW && raw_adc_value < RAW_RAIL_HIGH)
        {
            // Feed a small median window first: one isolated dropout/glitch
            // sample must not reach the endpoints. The endpoints only update
            // from the window median once the window is full.
            m_calib_raw_window[m_calib_window_head] = raw_adc_value;
            m_calib_mv_window[m_calib_window_head] = current_voltage_mv;
            m_calib_window_head = (m_calib_window_head + 1) % CALIBRATION_MEDIAN_WINDOW;
            if (m_calib_window_count < CALIBRATION_MEDIAN_WINDOW)
                ++m_calib_window_count;

            if (m_calib_window_count == CALIBRATION_MEDIAN_WINDOW)
            {
                const int median_raw = window_median(m_calib_raw_window);
                const int median_mv = window_median(m_calib_mv_window);
                // Track both the voltage range (legacy diagnostics) and the raw
                // ADC block-mean range: the raw counts are the authoritative
                // endpoints. These accumulators are not used for the mapping.
                const int track_min = m_track_min_mv.load();
                const int track_max = m_track_max_mv.load();
                if (median_mv < track_min)
                    m_track_min_mv.store(median_mv);
                if (median_mv > track_max)
                    m_track_max_mv.store(median_mv);
                if (median_raw < m_track_min_raw.load())
                    m_track_min_raw.store(static_cast<uint16_t>(median_raw));
                if (median_raw > m_track_max_raw.load())
                    m_track_max_raw.store(static_cast<uint16_t>(median_raw));
            }
        }
        else
        {
            ++m_rail_samples;
        }
    }

    *measured_angle_rad = 0.0f;
    if (use_raw_range)
    {
        // Prefer the raw range: map the raw block mean linearly so the stored
        // minimum is exactly 0 and the maximum is exactly 2*pi; values outside
        // the range clamp to the endpoints.
        const int clamped_raw = std::clamp(
          static_cast<int>(raw_adc_value), static_cast<int>(raw_min), static_cast<int>(raw_max));
        const float ratio =
          static_cast<float>(clamped_raw - raw_min) / static_cast<float>(raw_max - raw_min);
        *measured_angle_rad = ratio * 2.0f * static_cast<float>(M_PI);
        return;
    }
    const int cal_min = m_min_voltage_mv.load();
    const int cal_max = m_max_voltage_mv.load();
    if (cal_max > cal_min)
    {
        int clamped_mv = std::clamp(current_voltage_mv, cal_min, cal_max);
        float ratio =
          static_cast<float>(clamped_mv - cal_min) / static_cast<float>(cal_max - cal_min);
        *measured_angle_rad = ratio * 2.0f * static_cast<float>(M_PI);
    }
    else
    {
        // Fallback if not calibrated: map raw value to angle
        const float max_raw = static_cast<float>((1 << m_bitwidth) - 1);
        *measured_angle_rad =
          static_cast<float>(m_last_avg_value) * (2.0f * static_cast<float>(M_PI) / max_raw);
    }
}

/**
 * @brief Starts an interactive calibration routine determining the true min/max analog voltages.
 * @param duration_ms Time to sample min/max readings.
 * @return ESP_OK on success.
 */
esp_err_t AS5600::calibrate_range(uint32_t duration_ms)
{
    if (!m_is_voltage_calibrated)
    {
        ESP_LOGE(TAG, "Channel %d must be voltage-calibrated first.", m_channel);
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Starting range calibration for channel %d. Rotate sensor now...", m_channel);
    reset_calibration_min_max();
    m_is_calibrating.store(true);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    m_is_calibrating.store(false);

    const int track_min_mv = m_track_min_mv.load();
    const int track_max_mv = m_track_max_mv.load();
    const uint16_t track_min_raw = m_track_min_raw.load();
    const uint16_t track_max_raw = m_track_max_raw.load();
    const int track_mv_span = track_max_mv - track_min_mv;
    const int track_raw_span = static_cast<int>(track_max_raw) - static_cast<int>(track_min_raw);
    ESP_LOGI(TAG,
             "Channel %d calibrated. Raw: [%u..%u] (%d counts), mV: [%d..%d]",
             m_channel,
             static_cast<unsigned>(track_min_raw),
             static_cast<unsigned>(track_max_raw),
             track_raw_span,
             track_min_mv,
             track_max_mv);

    if (track_mv_span < MIN_VALID_VOLTAGE_RANGE_MV)
    {
        ESP_LOGE(
          TAG, "Error: Voltage range for channel %d is invalid (%d mV).", m_channel, track_mv_span);
        return ESP_ERR_INVALID_STATE;
    }

    // A successful validation applies the tracked range (the raw endpoints only
    // when they form a plausible full turn, so the millivolt fallback stays in
    // charge otherwise).
    m_min_voltage_mv.store(track_min_mv);
    m_max_voltage_mv.store(track_max_mv);
    if (track_raw_span >= static_cast<int>(MIN_VALID_RAW_RANGE) &&
        track_raw_span <= static_cast<int>(MAX_VALID_RAW_RANGE) &&
        track_min_raw >= MIN_PLAUSIBLE_RAW_MIN && track_max_raw <= MAX_PLAUSIBLE_RAW_MAX)
    {
        m_min_raw.store(track_min_raw);
        m_max_raw.store(track_max_raw);
    }

    return ESP_OK;
}

/**
 * @brief Safely saves the encoder's min/max calibration range to NVS via NVSManager.
 * @return ESP_OK on success.
 */
esp_err_t AS5600::save_calibration_to_nvs()
{
    char key_min[15], key_max[15];
    char key_rmin[15], key_rmax[15];
    snprintf(key_min, sizeof(key_min), "ch%d_min", static_cast<int>(m_channel));
    snprintf(key_max, sizeof(key_max), "ch%d_max", static_cast<int>(m_channel));
    snprintf(key_rmin, sizeof(key_rmin), "ch%d_rmin", static_cast<int>(m_channel));
    snprintf(key_rmax, sizeof(key_rmax), "ch%d_rmax", static_cast<int>(m_channel));

    // Validate the tracked endpoints before writing anything: a railed or
    // implausible range would map 0/360 to the wrong angles. On failure the
    // applied (stored) calibration is kept untouched.
    const int track_min_mv = m_track_min_mv.load();
    const int track_max_mv = m_track_max_mv.load();
    const uint16_t track_min_raw = m_track_min_raw.load();
    const uint16_t track_max_raw = m_track_max_raw.load();
    const int min_raw = static_cast<int>(track_min_raw);
    const int max_raw = static_cast<int>(track_max_raw);
    const int raw_span = max_raw - min_raw;
    const int mv_span = track_max_mv - track_min_mv;
    if (min_raw < static_cast<int>(MIN_PLAUSIBLE_RAW_MIN) ||
        max_raw > static_cast<int>(MAX_PLAUSIBLE_RAW_MAX) ||
        raw_span < static_cast<int>(MIN_VALID_RAW_RANGE) ||
        raw_span > static_cast<int>(MAX_VALID_RAW_RANGE))
    {
        ESP_LOGE(TAG,
                 "Channel %d raw range implausible: [%d..%d] (span %d); expected min >= %u, "
                 "max <= %u and span %u..%u. Keeping the stored calibration.",
                 m_channel,
                 min_raw,
                 max_raw,
                 raw_span,
                 static_cast<unsigned>(MIN_PLAUSIBLE_RAW_MIN),
                 static_cast<unsigned>(MAX_PLAUSIBLE_RAW_MAX),
                 static_cast<unsigned>(MIN_VALID_RAW_RANGE),
                 static_cast<unsigned>(MAX_VALID_RAW_RANGE));
        return ESP_FAIL;
    }
    if (mv_span < MIN_VALID_VOLTAGE_RANGE_MV)
    {
        ESP_LOGE(TAG,
                 "Channel %d millivolt range implausible (%d mV); keeping the stored "
                 "calibration.",
                 m_channel,
                 mv_span);
        return ESP_FAIL;
    }

    esp_err_t err = NVSManager::save_i32(NVS_NS, key_min, track_min_mv);

    if (err == ESP_OK)
        err = NVSManager::save_i32(NVS_NS, key_max, track_max_mv);

    // The raw block-mean endpoints are the authoritative mapping range; the
    // millivolt keys stay for compatibility and diagnostics (and as the
    // fallback mapping range).
    if (err == ESP_OK)
    {
        err = NVSManager::save_i32(NVS_NS, key_rmin, static_cast<int32_t>(min_raw));
        if (err == ESP_OK)
            err = NVSManager::save_i32(NVS_NS, key_rmax, static_cast<int32_t>(max_raw));
    }

    if (err == ESP_OK)
    {
        // Only a successful save promotes the tracked range to the applied
        // range: an aborted run (or a failed save) leaves the previous
        // calibration driving the mapping and the frame.
        m_min_voltage_mv.store(track_min_mv);
        m_max_voltage_mv.store(track_max_mv);
        m_min_raw.store(track_min_raw);
        m_max_raw.store(track_max_raw);
        ESP_LOGI(TAG,
                 "Saved calibration for Ch %d: raw [%u..%u], mV [%d..%d]",
                 static_cast<int>(m_channel),
                 static_cast<unsigned>(track_min_raw),
                 static_cast<unsigned>(track_max_raw),
                 track_min_mv,
                 track_max_mv);
    }

    return err;
}

/**
 * @brief Safely loads the encoder's min/max calibration range from NVS via NVSManager.
 * @return ESP_OK on success.
 */
esp_err_t AS5600::load_calibration_from_nvs()
{
    char key_min[15], key_max[15];
    char key_rmin[15], key_rmax[15];
    snprintf(key_min, sizeof(key_min), "ch%d_min", static_cast<int>(m_channel));
    snprintf(key_max, sizeof(key_max), "ch%d_max", static_cast<int>(m_channel));
    snprintf(key_rmin, sizeof(key_rmin), "ch%d_rmin", static_cast<int>(m_channel));
    snprintf(key_rmax, sizeof(key_rmax), "ch%d_rmax", static_cast<int>(m_channel));

    int32_t min_v = 0, max_v = 0;
    esp_err_t err_min = NVSManager::load_i32(NVS_NS, key_min, &min_v);
    esp_err_t err_max = NVSManager::load_i32(NVS_NS, key_max, &max_v);

    if (err_min == ESP_OK && err_max == ESP_OK)
    {
        set_calibration_range(static_cast<int>(min_v), static_cast<int>(max_v));

        // Raw endpoints are optional: calibrations stored before the raw range
        // existed only have the millivolt keys and use the millivolt fallback.
        int32_t raw_min_v = 0, raw_max_v = 0;
        const bool raw_loaded = NVSManager::load_i32(NVS_NS, key_rmin, &raw_min_v) == ESP_OK &&
                                NVSManager::load_i32(NVS_NS, key_rmax, &raw_max_v) == ESP_OK;
        const int32_t raw_span = raw_max_v - raw_min_v;
        // Apply the same plausibility rules as the save path: a range stored by
        // an earlier run with a railed endpoint (for example a 0-count analog
        // dropout) must not drive the angle mapping; fall back to millivolts.
        const bool raw_plausible = raw_loaded &&
                                   raw_min_v >= static_cast<int32_t>(MIN_PLAUSIBLE_RAW_MIN) &&
                                   raw_max_v <= static_cast<int32_t>(MAX_PLAUSIBLE_RAW_MAX) &&
                                   raw_span >= static_cast<int32_t>(MIN_VALID_RAW_RANGE) &&
                                   raw_span <= static_cast<int32_t>(MAX_VALID_RAW_RANGE);
        if (raw_plausible)
        {
            m_min_raw.store(static_cast<uint16_t>(raw_min_v));
            m_max_raw.store(static_cast<uint16_t>(raw_max_v));
        }
        else
        {
            m_min_raw.store(65535);
            m_max_raw.store(0);
            if (raw_loaded)
                ESP_LOGW(TAG,
                         "Stored raw range for Ch %d is implausible ([%d..%d]); using the "
                         "millivolt fallback.",
                         static_cast<int>(m_channel),
                         static_cast<int>(raw_min_v),
                         static_cast<int>(raw_max_v));
        }

        ESP_LOGI(TAG,
                 "Loaded calibration for Ch %d: raw [%u..%u], mV [%d..%d]",
                 static_cast<int>(m_channel),
                 static_cast<unsigned>(m_min_raw.load()),
                 static_cast<unsigned>(m_max_raw.load()),
                 m_min_voltage_mv.load(),
                 m_max_voltage_mv.load());
        return ESP_OK;
    }

    return ESP_ERR_NOT_FOUND;
}

/**
 * @brief Get the last sampled raw voltage.
 * @return Voltage in millivolts.
 */
int AS5600::get_last_voltage_mv() const
{
    if (m_is_voltage_calibrated)
    {
        int voltage = 0;
        adc_cali_raw_to_voltage(m_cali_handle, m_last_avg_value, &voltage);
        return voltage;
    }
    return -1;
}

/**
 * @brief Get the last 12-bit ADC raw read value.
 * @return Raw value (0-4095).
 */
uint16_t AS5600::get_last_raw_value() const
{
    return m_last_avg_value;
}

/**
 * @brief Internal helper to read the CONF_H I2C register.
 * @param config Output word pointer.
 * @return ESP_OK on success.
 */
esp_err_t AS5600::read_config_register(uint16_t * config)
{
    if (I2Cdev::read_word(AS5600_ADDR, static_cast<uint8_t>(Register::CONF_H), config) != 0)
        return ESP_FAIL;

    return ESP_OK;
}

/**
 * @brief Internal helper to write the CONF_H I2C register.
 * @param config Input word.
 * @return ESP_OK on success.
 */
esp_err_t AS5600::write_config_register(uint16_t config)
{
    if (!I2Cdev::write_word(AS5600_ADDR, static_cast<uint8_t>(Register::CONF_H), config))
        return ESP_FAIL;

    return ESP_OK;
}

/**
 * @brief Reads the currently active filter and stage configurations via I2C.
 * @param stage Pointer to receive output stage setting.
 * @param slow Pointer to receive slow filter setting.
 * @param fast Pointer to receive fast filter setting.
 * @return ESP_OK on success.
 */
esp_err_t AS5600::read_configuration(OutputStage * stage, SlowFilter * slow, FastFilter * fast)
{
    if (!m_i2c_initialized)
        return ESP_ERR_INVALID_STATE;

    uint16_t config_val;
    esp_err_t ret = read_config_register(&config_val);
    if (ret != ESP_OK)
        return ret;

    // Bit manipulation based on datasheet (Register 0x07)
    // Bits 5:4 = Output Stage
    if (stage)
        *stage = output_stage_from_config(config_val);

    // Bits 9:8 = Slow Filter
    if (slow)
        *slow = static_cast<SlowFilter>((config_val >> 8) & 0x03);

    // Bits 12:10 = Fast Filter
    if (fast)
        *fast = static_cast<FastFilter>((config_val >> 10) & 0x07);

    return ESP_OK;
}

/**
 * @brief Reads the output stage over I2C without per-instance state.
 *
 * Used by consumers that display the configured stage without owning a driver
 * instance. The caller must have routed this encoder onto the bus first (for
 * example by selecting its CD4052B channel).
 *
 * @param stage Output for the configured output stage.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG for a null output, ESP_FAIL
 *         when the read fails.
 */
esp_err_t AS5600::read_output_stage(OutputStage * stage)
{
    if (stage == nullptr)
        return ESP_ERR_INVALID_ARG;

    uint16_t config_val = 0;
    if (read_config_register(&config_val) != ESP_OK)
        return ESP_FAIL;
    *stage = output_stage_from_config(config_val);
    return ESP_OK;
}

/**
 * @brief Manually sets the upper and lower voltage bounds for rotation mapping.
 * @param min_mv Minimum recorded voltage at 0 degrees.
 * @param max_mv Maximum recorded voltage at 360 degrees.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if invalid range.
 */
esp_err_t AS5600::set_calibration_range(int min_mv, int max_mv)
{
    if (min_mv >= max_mv)
    {
        ESP_LOGE(TAG,
                 "Invalid calibration range: min_mv (%d) must be less than max_mv (%d)",
                 min_mv,
                 max_mv);
        return ESP_ERR_INVALID_ARG;
    }

    if ((min_mv < 0 || min_mv > 3300) || (max_mv < 0 || max_mv > 3300))
    {
        ESP_LOGE(TAG,
                 "Invalid calibration range: min_mv and max_mv must be non-negative and <= 3300 "
                 "mV. Received "
                 "min_mv=%d, max_mv=%d",
                 min_mv,
                 max_mv);
        return ESP_ERR_INVALID_ARG;
    }

    m_min_voltage_mv.store(min_mv);
    m_max_voltage_mv.store(max_mv);

    return ESP_OK;
}

/**
 * @brief Anchors the applied raw ADC range to externally measured endpoints.
 *
 * Used by the wiring calibration ADC-I2C alignment: the standstill fit yields
 * the raw counts at the I2C angle reference 0 and 4095, which replace the raw
 * endpoints captured during the (distorted) open-loop spin. The millivolt keys
 * are kept as diagnostics/fallback.
 *
 * @param min_raw Raw ADC count at the I2C angle 0.
 * @param max_raw Raw ADC count at the I2C angle 4095.
 * @return ESP_OK when the range is plausible and persisted; ESP_ERR_INVALID_ARG
 *         for an implausible range, or the NVS error when persistence fails.
 */
esp_err_t AS5600::set_raw_calibration_range(uint16_t min_raw, uint16_t max_raw)
{
    const int span = static_cast<int>(max_raw) - static_cast<int>(min_raw);
    if (min_raw < MIN_PLAUSIBLE_RAW_MIN || max_raw > MAX_PLAUSIBLE_RAW_MAX ||
        span < static_cast<int>(MIN_VALID_RAW_RANGE) ||
        span > static_cast<int>(MAX_VALID_RAW_RANGE))
    {
        ESP_LOGE(TAG,
                 "Channel %d anchored raw range invalid: [%u..%u] (span %d); expected min >= %u, "
                 "max <= %u and span %u..%u. Keeping the stored calibration.",
                 static_cast<int>(m_channel),
                 static_cast<unsigned>(min_raw),
                 static_cast<unsigned>(max_raw),
                 span,
                 static_cast<unsigned>(MIN_PLAUSIBLE_RAW_MIN),
                 static_cast<unsigned>(MAX_PLAUSIBLE_RAW_MAX),
                 static_cast<unsigned>(MIN_VALID_RAW_RANGE),
                 static_cast<unsigned>(MAX_VALID_RAW_RANGE));
        return ESP_ERR_INVALID_ARG;
    }

    char key_rmin[15], key_rmax[15];
    snprintf(key_rmin, sizeof(key_rmin), "ch%d_rmin", static_cast<int>(m_channel));
    snprintf(key_rmax, sizeof(key_rmax), "ch%d_rmax", static_cast<int>(m_channel));
    esp_err_t err = NVSManager::save_i32(NVS_NS, key_rmin, static_cast<int32_t>(min_raw));
    if (err == ESP_OK)
        err = NVSManager::save_i32(NVS_NS, key_rmax, static_cast<int32_t>(max_raw));
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG,
                 "Channel %d anchored raw range could not be persisted (%s); keeping the stored "
                 "calibration.",
                 static_cast<int>(m_channel),
                 esp_err_to_name(err));
        return err;
    }

    m_min_raw.store(min_raw);
    m_max_raw.store(max_raw);
    ESP_LOGI(TAG,
             "Channel %d applied raw range anchored: [%u..%u] (span %d).",
             static_cast<int>(m_channel),
             static_cast<unsigned>(min_raw),
             static_cast<unsigned>(max_raw),
             span);
    return ESP_OK;
}

/**
 * @brief Resets the calibration boundaries, preparing for a new tracking run.
 */
void AS5600::reset_calibration_min_max()
{
    // Resets only the tracking accumulators: the applied range stays untouched,
    // so an aborted or failed run keeps the stored calibration active.
    m_track_min_mv.store(5000);
    m_track_max_mv.store(0);
    m_track_min_raw.store(65535);
    m_track_max_raw.store(0);
    m_rail_samples.store(0);
    m_calib_window_count = 0;
    m_calib_window_head = 0;
}

/**
 * @brief Enables live tracking of minimum and maximum voltages during readings.
 */
void AS5600::start_calibration_mode()
{
    // Fresh tracking run; the applied range is not touched.
    m_track_min_mv.store(5000);
    m_track_max_mv.store(0);
    m_track_min_raw.store(65535);
    m_track_max_raw.store(0);
    m_calib_window_count = 0;
    m_calib_window_head = 0;
    m_is_calibrating.store(true);
}

/**
 * @brief Disables live tracking of min/max voltages.
 */
void AS5600::stop_calibration_mode()
{
    m_is_calibrating.store(false);
    const uint32_t dropped = m_rail_samples.exchange(0);
    if (dropped > 0)
        ESP_LOGW(TAG,
                 "Channel %d range tracking ignored %u rail/glitch samples.",
                 m_channel,
                 static_cast<unsigned>(dropped));
}

/**
 * @brief Get currently calibrated minimum voltage.
 * @return Millivolts.
 */
int AS5600::get_calib_min() const
{
    return m_min_voltage_mv.load();
}

/**
 * @brief Get currently calibrated maximum voltage.
 * @return Millivolts.
 */
int AS5600::get_calib_max() const
{
    return m_max_voltage_mv.load();
}

/**
 * @brief Get currently calibrated minimum raw ADC block mean.
 * @return Raw value (0-4095).
 */
uint16_t AS5600::get_calib_min_raw() const
{
    return m_min_raw.load();
}

/**
 * @brief Get currently calibrated maximum raw ADC block mean.
 * @return Raw value (0-4095).
 */
uint16_t AS5600::get_calib_max_raw() const
{
    return m_max_raw.load();
}

/**
 * @brief Get the tracked minimum voltage of the current calibration run.
 * @return Millivolts (not applied to the mapping until a successful save).
 */
int AS5600::get_track_min_mv() const
{
    return m_track_min_mv.load();
}

/**
 * @brief Get the tracked maximum voltage of the current calibration run.
 * @return Millivolts (not applied to the mapping until a successful save).
 */
int AS5600::get_track_max_mv() const
{
    return m_track_max_mv.load();
}

/**
 * @brief Get the tracked minimum raw ADC block mean of the current run.
 * @return Raw value (0-4095; not applied to the mapping until a successful save).
 */
uint16_t AS5600::get_track_min_raw() const
{
    return m_track_min_raw.load();
}

/**
 * @brief Get the tracked maximum raw ADC block mean of the current run.
 * @return Raw value (0-4095; not applied to the mapping until a successful save).
 */
uint16_t AS5600::get_track_max_raw() const
{
    return m_track_max_raw.load();
}
