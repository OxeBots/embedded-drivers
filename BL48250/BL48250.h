#ifndef _BL48250_H_
#define _BL48250_H_

#include <driver/gpio.h>
#include <driver/ledc.h>
#include <driver/pulse_cnt.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <algorithm>
#include <array>
#include <cmath>

#include "constants.h"

class HC595;

namespace config
{
namespace driver
{
constexpr float BL48250_MAX_VEL_RPM = 4000.0;  // Motor shaft limit
// Gear reduction between the motor shaft and the wheel shaft
constexpr float WHEEL_REDUCTION = 3.0f;
constexpr float WHEEL_MAX_VEL_RPM = BL48250_MAX_VEL_RPM / WHEEL_REDUCTION;  // Wheel shaft
constexpr float WHEEL_MAX_VEL_RAD = WHEEL_MAX_VEL_RPM * RPM_TO_RAD_S;       // rad/s
// Radians per wheel-shaft second represented by one pulse-feedback count:
// the counter increments on both edges of the feedback net
// (CONFIG_MOTOR_FB_COUNTS_PER_REV counts per motor-shaft revolution) and the
// wheel turns WHEEL_REDUCTION times slower than the motor shaft.
constexpr float BL48250_FB_RAD_S_PER_COUNT =
  (2.0f * M_PI / static_cast<float>(CONFIG_MOTOR_FB_COUNTS_PER_REV)) / WHEEL_REDUCTION;
// Sliding window behind pulse_velocity_rad_s(): at the 4000 RPM motor limit
// 100 ms holds 67 counts (1.5% resolution) for about 50 ms of measurement lag.
constexpr uint32_t BL48250_FB_WINDOW_MS = 100;
constexpr float BL48250_MIN_VEL = 0.0;
constexpr uint8_t MOTOR_CW = 1;
constexpr uint8_t MOTOR_CCW = 0;

// Configuration struct for BL48250 singleton initialization.
struct MotorDriverConfig
{
    ledc_timer_t timer;
    ledc_mode_t speed_mode;
    ledc_timer_bit_t duty_resolution;
    uint32_t pwm_freq;
    std::array<gpio_num_t, 4> motor_pwm_pins;
    std::array<ledc_channel_t, 4> motor_channels;
    // Digital pulse-feedback inputs (one per motor, motor-shaft rate). Leave
    // an entry at GPIO_NUM_NC to disable the feedback counter for that motor.
    // Defaults to "no feedback pin" for every motor; gpio_num_t zero-value
    // is GPIO_NUM_0, so the pins must be set to GPIO_NUM_NC explicitly.
    std::array<gpio_num_t, 4> motor_fb_pins = {GPIO_NUM_NC, GPIO_NUM_NC, GPIO_NUM_NC, GPIO_NUM_NC};
    // Required: drives the four motor direction lines.
    HC595 * direction_shift_register = nullptr;
    std::array<uint8_t, 4> direction_bits = {0, 1, 2, 3};
};
}  // namespace driver
}  // namespace config

// Send the needed signals to the 4 BL48250 motors of the omnidirectional robot to move it.
class BL48250
{
   public:
    static BL48250 & get_instance();

    esp_err_t configure(const config::driver::MotorDriverConfig & config);

    void deinit();

    void set_duties(const std::array<uint32_t, 4> & duties,
                    const std::array<uint8_t, 4> & directions);

    // Wheel-shaft angular velocity (rad/s) of one motor from a sliding window
    // over its pulse counter, recomputed without consuming counts. Returns
    // false when that motor has no counter or no window baseline yet; the sign
    // follows the last commanded direction (positive means CW).
    bool pulse_velocity_rad_s(size_t motor_idx, float * velocity);

    // Snapshot of each counter total since it was started (diagnostics).
    std::array<int32_t, 4> pulse_counts() const;

    BL48250(const BL48250 &) = delete;
    BL48250 & operator=(const BL48250 &) = delete;

   private:
    BL48250();
    ~BL48250();

    ledc_timer_t timer_;
    ledc_mode_t speed_mode_;
    ledc_timer_bit_t duty_resolution_;
    uint32_t pwm_freq_;
    std::array<gpio_num_t, 4> motor_pwm_pins_;
    std::array<ledc_channel_t, 4> motor_channels_;
    HC595 * direction_shift_register_;
    std::array<uint8_t, 4> direction_bits_;
    uint8_t direction_mask_;
    uint32_t max_duty_;

    std::array<uint8_t, 4> directions_;
    std::array<uint32_t, 4> duty_cycles_;

    bool m_configured;

    // Pulse-feedback hardware (one PCNT unit per motor) plus the snapshot
    // ring that feeds the sliding-window velocity. m_fb_mutex serializes the
    // ring and the counter handles between readers and deinit().
    esp_err_t init_pulse_feedback();
    void deinit_pulse_feedback();

    struct PulseSample
    {
        int32_t count;
        int64_t time_us;
    };

    // Ring capacity in snapshots. Pushes are decimated to one per
    // PULSE_RING_PUSH_INTERVAL_US, so the capacity covers many window lengths.
    static constexpr size_t PULSE_RING_SIZE = 32;
    static constexpr int64_t PULSE_RING_PUSH_INTERVAL_US = 5000;

    std::array<gpio_num_t, 4> motor_fb_pins_;
    std::array<pcnt_unit_handle_t, 4> pcnt_units_;
    std::array<pcnt_channel_handle_t, 4> pcnt_channels_;
    std::array<std::array<PulseSample, PULSE_RING_SIZE>, 4> m_pulse_ring_;
    std::array<size_t, 4> m_pulse_ring_count_;
    std::array<size_t, 4> m_pulse_ring_head_;
    std::array<int64_t, 4> m_last_ring_push_us_;
    std::array<bool, 4> m_pulse_read_error_logged_ = {};
    bool m_fb_configured;
    SemaphoreHandle_t m_fb_mutex;
};

#endif  // _BL48250_H_
