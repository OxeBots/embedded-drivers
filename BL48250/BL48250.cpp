#include "BL48250.h"

#include <esp_timer.h>

#include "74HC595.h"
#include "helper_func.h"

using namespace config::driver;

static const char * TAG = "BL48250_driver";

BL48250 & BL48250::get_instance()
{
    static BL48250 instance;
    return instance;
}

BL48250::BL48250()
: timer_(LEDC_TIMER_0),
  speed_mode_(LEDC_LOW_SPEED_MODE),
  duty_resolution_(LEDC_TIMER_10_BIT),
  pwm_freq_(20000),
  motor_pwm_pins_{},
  motor_channels_{},
  direction_shift_register_(nullptr),
  direction_bits_{0, 1, 2, 3},
  direction_mask_(0),
  max_duty_(0),
  directions_{MOTOR_CW, MOTOR_CW, MOTOR_CW, MOTOR_CW},
  duty_cycles_{0, 0, 0, 0},
  m_configured(false),
  motor_fb_pins_{GPIO_NUM_NC, GPIO_NUM_NC, GPIO_NUM_NC, GPIO_NUM_NC},
  pcnt_units_{},
  pcnt_channels_{},
  m_pulse_ring_count_{},
  m_pulse_ring_head_{},
  m_last_ring_push_us_{},
  m_fb_configured(false),
  m_fb_mutex(xSemaphoreCreateMutex())
{
}

BL48250::~BL48250()
{
    deinit_pulse_feedback();
    if (m_fb_mutex)
    {
        vSemaphoreDelete(m_fb_mutex);
        m_fb_mutex = nullptr;
    }
    if (m_configured)
    {
        // Stop all motors
        for (size_t i = 0; i < 4; ++i)
        {
            ledc_set_duty(speed_mode_, motor_channels_[i], 0);
            ledc_update_duty(speed_mode_, motor_channels_[i]);
        }
        // Reset the GPIO pins to release them from the LEDC peripheral
        for (auto pin : motor_pwm_pins_)
        {
            gpio_reset_pin(pin);
        }
    }
}

/**
 * @brief Configure the motor driver with GPIO pins, PWM settings, and LEDC channels.
 *
 * Must be called once before any set_duties() calls. Initializes the LEDC timer,
 * configures 4 PWM channels, and sets up the direction outputs through the
 * shift register, which is required.
 *
 * @param config MotorDriverConfig struct with PWM frequency, pins, channels, etc.
 * @return ESP_OK on success, or an error from the LEDC, shift-register or
 *         pulse-feedback setup.
 */
esp_err_t BL48250::configure(const MotorDriverConfig & config)
{
    if (m_configured)
        return ESP_OK;

    timer_ = config.timer;
    speed_mode_ = config.speed_mode;
    duty_resolution_ = config.duty_resolution;
    pwm_freq_ = config.pwm_freq;
    motor_pwm_pins_ = config.motor_pwm_pins;
    motor_channels_ = config.motor_channels;
    motor_fb_pins_ = config.motor_fb_pins;
    direction_shift_register_ = config.direction_shift_register;
    direction_bits_ = config.direction_bits;

    if (direction_shift_register_ == nullptr)
    {
        ESP_LOGE(TAG, "No direction shift register configured");
        return ESP_ERR_INVALID_ARG;
    }

    direction_mask_ = 0;
    for (uint8_t bit : direction_bits_)
    {
        if (bit >= 8 || (direction_mask_ & (1U << bit)) != 0)
        {
            ESP_LOGE(TAG, "Invalid or duplicate motor direction bit: %u", bit);
            return ESP_ERR_INVALID_ARG;
        }
        direction_mask_ |= static_cast<uint8_t>(1U << bit);
    }

    if (!direction_shift_register_->is_initialized())
    {
        ESP_LOGE(TAG, "Direction shift register is not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    max_duty_ = (1 << static_cast<int>(duty_resolution_)) - 1;

    // Configure PWM timer
    ledc_timer_config_t timer_conf = {.speed_mode = speed_mode_,
                                      .duty_resolution = duty_resolution_,
                                      .timer_num = timer_,
                                      .freq_hz = pwm_freq_,
                                      .clk_cfg = LEDC_USE_APB_CLK,
                                      .deconfigure = false};
    esp_err_t err = ledc_timer_config(&timer_conf);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure LEDC timer: %s", esp_err_to_name(err));
        return err;
    }

    // Configure each motor's PWM channel
    for (size_t i = 0; i < 4; ++i)
    {
        ledc_channel_config_t channel_conf = {
          .gpio_num = motor_pwm_pins_[i],
          .speed_mode = speed_mode_,
          .channel = motor_channels_[i],
          .intr_type = LEDC_INTR_DISABLE,
          .timer_sel = timer_,
          .duty = 0,
          .hpoint = 0,
          .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
          .flags = {.output_invert = 0},
        };
        err = ledc_channel_config(&channel_conf);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to configure LEDC channel %zu: %s", i, esp_err_to_name(err));
            return err;
        }

        directions_[i] = MOTOR_CW;
        duty_cycles_[i] = 0;
        reversal_start_us_[i] = 0;
    }

    uint8_t direction_value = 0;
    for (uint8_t bit : direction_bits_) direction_value |= static_cast<uint8_t>(1U << bit);

    esp_err_t direction_err =
      direction_shift_register_->update_mask(direction_mask_, direction_value);

    if (direction_err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize motor direction outputs");
        return direction_err;
    }

    err = init_pulse_feedback();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure pulse feedback: %s", esp_err_to_name(err));
        return err;
    }

    m_configured = true;
    ESP_LOGI(TAG,
             "BL48250 configured. PWM: %luHz, %d-bit duty",
             (unsigned long)pwm_freq_,
             static_cast<int>(duty_resolution_));
    return ESP_OK;
}

/**
 * @brief Deinitialize the motor driver: stop all motors, reset GPIO pins, clear state.
 *
 * Safe to call multiple times. After deinit(), configure() can be called again.
 */
void BL48250::deinit()
{
    deinit_pulse_feedback();

    if (!m_configured)
        return;

    // Stop all motors
    for (size_t i = 0; i < 4; ++i)
    {
        ledc_set_duty(speed_mode_, motor_channels_[i], 0);
        ledc_update_duty(speed_mode_, motor_channels_[i]);
    }
    // Reset the GPIO pins to release them from the LEDC peripheral
    for (auto pin : motor_pwm_pins_)
    {
        gpio_reset_pin(pin);
    }

    m_configured = false;
    direction_shift_register_ = nullptr;
    ESP_LOGI(TAG, "BL48250 deinitialized.");
}

/**
 * @brief Apply open-loop outputs, waiting for any requested direction changes.
 */
void BL48250::set_duties(const std::array<uint32_t, 4> & duties,
                         const std::array<uint8_t, 4> & directions)
{
    update_duties(duties, directions);
    if (m_configured && directions != directions_)
    {
        vTaskDelay(pdMS_TO_TICKS(DIRECTION_CHANGE_DELAY_US / 1000) + 1);
        update_duties(duties, directions);
    }
}

/**
 * @brief Disable drive during the delay before latching a motor's new direction.
 */
void BL48250::update_duties(const std::array<uint32_t, 4> & duties,
                            const std::array<uint8_t, 4> & directions)
{
    if (!m_configured)
        return;

    const int64_t now_us = esp_timer_get_time();
    auto next_directions = directions_;
    uint8_t change_mask = 0;
    uint8_t change_value = 0;
    for (size_t i = 0; i < duties.size(); ++i)
    {
        if (directions[i] == directions_[i])
            continue;
        write_duty(i, 0);
        if (duty_cycles_[i] != 0)
            continue;
        if (reversal_start_us_[i] == 0)
        {
            reversal_start_us_[i] = esp_timer_get_time();
            continue;
        }
        if (now_us - reversal_start_us_[i] < DIRECTION_CHANGE_DELAY_US)
            continue;
        const uint8_t bit = static_cast<uint8_t>(1U << direction_bits_[i]);
        change_mask |= bit;
        if (directions[i] != 0)
            change_value |= bit;
        next_directions[i] = directions[i];
    }

    if (change_mask != 0)
    {
        if (direction_shift_register_->update_mask(change_mask, change_value) == ESP_OK)
        {
            xSemaphoreTake(m_fb_mutex, portMAX_DELAY);
            directions_ = next_directions;
            xSemaphoreGive(m_fb_mutex);
        }
        else
            ESP_LOGE(TAG, "Failed to update motor direction outputs");
    }

    for (size_t i = 0; i < duties.size(); ++i)
    {
        if (directions[i] == directions_[i])
        {
            reversal_start_us_[i] = 0;
            write_duty(i, std::min(duties[i], max_duty_));
        }
    }
}

/**
 * @brief Write changed duty to the hardware.
 */
void BL48250::write_duty(size_t motor, uint32_t duty)
{
    if (duty == duty_cycles_[motor])
        return;
    if (ledc_set_duty(speed_mode_, motor_channels_[motor], duty) != ESP_OK ||
        ledc_update_duty(speed_mode_, motor_channels_[motor]) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to update motor %zu duty", motor + 1);
        return;
    }
    duty_cycles_[motor] = duty;
}

std::array<int32_t, 4> BL48250::pulse_counts() const
{
    std::array<int32_t, 4> counts = {0, 0, 0, 0};

    for (size_t i = 0; i < 4; ++i)
    {
        if (pcnt_units_[i] == nullptr)
            continue;

        int count = 0;
        if (pcnt_unit_get_count(pcnt_units_[i], &count) == ESP_OK)
            counts[i] = static_cast<int32_t>(count);
    }

    return counts;
}

std::array<uint32_t, 4> BL48250::get_duty_cycles() const
{
    return duty_cycles_;
}

std::array<uint8_t, 4> BL48250::get_directions() const
{
    return directions_;
}

/**
 * @brief Create one hardware pulse counter (PCNT unit) per motor feedback pin.
 *
 * The counter increments on both edges of the feedback net, exactly like the
 * Arduino reference sketch. The high-limit watch point lets the driver
 * accumulate past the 16-bit hardware range (accum_count), so the reported
 * count never wraps as long as it is polled before 32767 edges accumulate
 * (about 49 s at the 4000 RPM motor limit).
 *
 * @return ESP_OK when every configured pin has a running counter.
 */
esp_err_t BL48250::init_pulse_feedback()
{
    if (m_fb_configured)
        return ESP_OK;

    if (m_fb_mutex == nullptr)
        return ESP_ERR_NO_MEM;

    xSemaphoreTake(m_fb_mutex, portMAX_DELAY);

    pcnt_units_.fill(nullptr);
    pcnt_channels_.fill(nullptr);
    for (size_t i = 0; i < 4; ++i)
    {
        m_pulse_ring_count_[i] = 0;
        m_pulse_ring_head_[i] = 0;
        m_last_ring_push_us_[i] = 0;
        m_pulse_read_error_logged_[i] = false;
    }

    esp_err_t err = ESP_OK;
    size_t created = 0;

    for (size_t i = 0; i < 4 && err == ESP_OK; ++i)
    {
        if (motor_fb_pins_[i] == GPIO_NUM_NC)
        {
            ESP_LOGW(TAG, "Motor %zu has no feedback pin configured", i);
            continue;
        }

        pcnt_unit_config_t unit_config = {.low_limit = -32767,
                                          .high_limit = 32767,
                                          .intr_priority = 0,
                                          .flags = {.accum_count = 1}};
        pcnt_unit_handle_t unit = nullptr;
        err = pcnt_new_unit(&unit_config, &unit);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "pcnt_new_unit failed for motor %zu: %s", i, esp_err_to_name(err));
            break;
        }

        pcnt_chan_config_t channel_config = {.edge_gpio_num = motor_fb_pins_[i],
                                             .level_gpio_num = -1,
                                             .flags = {
                                               .invert_edge_input = 0,
                                               .invert_level_input = 0,
                                               .virt_edge_io_level = 0,
                                               .virt_level_io_level = 0,
                                               .io_loop_back = 0,
                                             }};
        pcnt_channel_handle_t channel = nullptr;
        err = pcnt_new_channel(unit, &channel_config, &channel);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "pcnt_new_channel failed for motor %zu: %s", i, esp_err_to_name(err));
            break;
        }

        // The PCNT driver enables the internal pull-up, and the board's
        // divider then holds an undriven feedback line around mid-scale,
        // where crosstalk becomes phantom edges. Hold the inactive level low
        // so a missing or unpowered motor driver cannot add counts.
        gpio_pullup_dis(motor_fb_pins_[i]);
        gpio_pulldown_en(motor_fb_pins_[i]);

        // Count every edge of the feedback net (rising and falling).
        err = pcnt_channel_set_edge_action(
          channel, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "pcnt_channel_set_edge_action failed for motor %zu", i);
            break;
        }

        // Watch points at the limits arm the overflow interrupt that keeps
        // the accumulated count continuous.
        err = pcnt_unit_add_watch_point(unit, unit_config.high_limit);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "pcnt_unit_add_watch_point failed for motor %zu", i);
            break;
        }

        err = pcnt_unit_enable(unit);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "pcnt_unit_enable failed for motor %zu", i);
            break;
        }

        err = pcnt_unit_clear_count(unit);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "pcnt_unit_clear_count failed for motor %zu", i);
            break;
        }

        err = pcnt_unit_start(unit);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "pcnt_unit_start failed for motor %zu", i);
            break;
        }

        pcnt_units_[i] = unit;
        pcnt_channels_[i] = channel;
        ++created;
        ESP_LOGI(TAG, "Motor %zu pulse feedback on GPIO %d started", i, motor_fb_pins_[i]);
    }

    if (err != ESP_OK)
    {
        // Release whatever was already created; the give happens inside.
        xSemaphoreGive(m_fb_mutex);
        deinit_pulse_feedback();
        return err;
    }

    m_fb_configured = (created > 0);
    xSemaphoreGive(m_fb_mutex);
    if (m_fb_configured)
        ESP_LOGI(TAG, "Pulse feedback counters running.");
    return ESP_OK;
}

/**
 * @brief Stop and delete every pulse-feedback counter.
 *
 * Best-effort: individual failures are ignored so a partial configuration is
 * still cleaned up. Safe to call when nothing was configured.
 */
void BL48250::deinit_pulse_feedback()
{
    if (m_fb_mutex == nullptr)
        return;

    xSemaphoreTake(m_fb_mutex, portMAX_DELAY);

    for (size_t i = 0; i < 4; ++i)
    {
        if (pcnt_units_[i] != nullptr)
        {
            pcnt_unit_stop(pcnt_units_[i]);
            pcnt_unit_disable(pcnt_units_[i]);
            pcnt_del_channel(pcnt_channels_[i]);
            pcnt_del_unit(pcnt_units_[i]);
        }
        pcnt_units_[i] = nullptr;
        pcnt_channels_[i] = nullptr;
        m_pulse_ring_count_[i] = 0;
        m_pulse_ring_head_[i] = 0;
        m_last_ring_push_us_[i] = 0;
        m_pulse_read_error_logged_[i] = false;
    }

    m_fb_configured = false;
    xSemaphoreGive(m_fb_mutex);
    ESP_LOGI(TAG, "Pulse feedback counters released.");
}

bool BL48250::pulse_velocity_rad_s(size_t motor_idx, float * velocity, TickType_t wait_ticks)
{
    if (velocity == nullptr || motor_idx >= 4 || m_fb_mutex == nullptr)
        return false;

    if (xSemaphoreTake(m_fb_mutex, wait_ticks) != pdTRUE)
        return false;

    bool valid = false;

    if (pcnt_units_[motor_idx] != nullptr)
    {
        const int64_t now_us = static_cast<int64_t>(esp_timer_get_time());
        int count = 0;
        if (pcnt_unit_get_count(pcnt_units_[motor_idx], &count) != ESP_OK)
        {
            if (!m_pulse_read_error_logged_[motor_idx])
            {
                ESP_LOGE(TAG, "Failed to read pulse counter for motor %zu", motor_idx);
                m_pulse_read_error_logged_[motor_idx] = true;
            }
        }
        else
        {
            // Decimate the snapshots so the ring covers several windows; the
            // resolution of the window does not improve past this interval.
            if (now_us - m_last_ring_push_us_[motor_idx] >= PULSE_RING_PUSH_INTERVAL_US)
            {
                m_pulse_ring_[motor_idx][m_pulse_ring_head_[motor_idx]] = {
                  static_cast<int32_t>(count), now_us};
                m_pulse_ring_head_[motor_idx] =
                  (m_pulse_ring_head_[motor_idx] + 1) % PULSE_RING_SIZE;
                if (m_pulse_ring_count_[motor_idx] < PULSE_RING_SIZE)
                    ++m_pulse_ring_count_[motor_idx];
                m_last_ring_push_us_[motor_idx] = now_us;
            }

            // The oldest snapshot inside the window gives the longest (least
            // quantized) baseline. Without one - first call after boot or
            // after a pause longer than the window - there is no velocity to
            // report; the caller keeps the angle-only path.
            int64_t reference_time_us = now_us;
            int32_t reference_count = count;
            bool have_baseline = false;
            for (size_t sample = 0; sample < m_pulse_ring_count_[motor_idx]; ++sample)
            {
                const PulseSample & snapshot = m_pulse_ring_[motor_idx][sample];
                const bool inside_window =
                  (now_us - snapshot.time_us) <= static_cast<int64_t>(BL48250_FB_WINDOW_MS) * 1000;
                if (inside_window && snapshot.time_us < reference_time_us)
                {
                    reference_time_us = snapshot.time_us;
                    reference_count = snapshot.count;
                    have_baseline = true;
                }
            }

            const float dt_s = static_cast<float>(now_us - reference_time_us) / 1000000.0f;
            if (have_baseline &&
                dt_s >= static_cast<float>(PULSE_RING_PUSH_INTERVAL_US) / 1000000.0f)
            {
                const float delta = static_cast<float>(count - reference_count);
                // The counter cannot see direction; the commanded direction is
                // the best available sign. The result is wheel-shaft rad/s.
                const float direction = (directions_[motor_idx] == MOTOR_CW) ? 1.0f : -1.0f;
                *velocity = direction * delta * BL48250_FB_RAD_S_PER_COUNT / dt_s;
                valid = true;
            }
        }
    }

    xSemaphoreGive(m_fb_mutex);
    return valid;
}
