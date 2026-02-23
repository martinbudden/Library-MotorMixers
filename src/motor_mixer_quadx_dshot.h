#pragma once

#include "esc_dshot.h"
#include "motor_mixer_quad_base.h"
#include "rpm_filters.h"

/*!
DShot Motor Mixer.

Hz is used for motor revolutions per second rather than RPS, since RPS is generally used for Radians Per Second.
*/
class MotorMixerQuadXDshot : public MotorMixerQuadBase {
public:
    MotorMixerQuadXDshot(uint32_t task_interval_microseconds, uint8_t output_to_motors_denominator, const motor_pins_t& pins);
public:
    virtual void set_motor_config(const motor_config_t& motor_config) override;
    virtual void set_motors_reversed(bool motors_is_reversed) override;
    virtual void output_to_motors(const motor_mixer_message_queue_item_t& queue_item, RpmFilters* rpm_filters, float delta_t, uint32_t tick_count, Debug& debug) override;
    virtual void rpm_filter_set_frequency_hz_iteration_step(RpmFilters* rpm_filters) override;
    virtual const dynamic_idle_controller_config_t* get_dynamic_idle_config() const override;
    virtual void set_dynamic_idle_controller_config(const dynamic_idle_controller_config_t& config) override;
    float calculate_slowest_motor_hz() const;
protected:
    DynamicIdleController _dynamic_idle_controller;
    size_t _rpm_filter_iteration_count {};
    std::array<EscDshot, MOTOR_COUNT> _motors;
};
