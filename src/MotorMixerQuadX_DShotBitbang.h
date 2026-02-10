#pragma once

#include "ESC_DShotBitbang.h"
#include "MotorMixerQuadBase.h"
#include "RPM_Filters.h"


/*!
DShot Motor Mixer.

Hz is used for motor revolutions per second rather than RPS, since RPS is generally used for Radians Per Second.
*/
class MotorMixerQuadXDshotBitbang : public MotorMixerQuadBase {
public:
    MotorMixerQuadXDshotBitbang(uint32_t task_interval_microseconds, uint32_t output_to_motors_denominator, const stm32_motor_pins_t& pins, Debug& debug);
public:
    virtual void set_motor_config(const motor_config_t& motor_config) override;
    virtual void set_motors_reversed(bool motors_is_reversed) override;
    virtual void output_to_motors(motor_mixer_commands_t& commands, float delta_t, uint32_t tick_count) override;
    virtual void rpm_filter_set_frequency_hz_iteration_step() override;
    virtual RpmFilters* get_rpm_filters() override;
    virtual const RpmFilters* get_rpm_filters() const override;
    virtual const DynamicIdleController* get_dynamic_idle_controller() const override;
    virtual void set_dynamic_idler_controller_config(const DynamicIdleController::config_t& config) override;
    float calculate_slowest_motor_hz() const;
protected:
    static constexpr float SECONDS_PER_MINUTE = 60.0F;
    static constexpr float DEFAULT_MOTOR_POLE_COUNT = 14.0F; //!< number of poles the motor has, used to calculate RPM from telemetry data
    float _erpm_to_hz { 2.0F * (100.0F / SECONDS_PER_MINUTE) / DEFAULT_MOTOR_POLE_COUNT };

    DynamicIdleController _dynamic_idle_controller;
    RpmFilters _rpm_filters;
    size_t _rpm_filter_iteration_count {};

    EscDshotBitbang _esc_dshot {};
    std::array<float, MOTOR_COUNT> _motor_frequencies_hz {};
};
