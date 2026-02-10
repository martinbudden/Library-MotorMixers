#include "DynamicIdleController.h"
#include "Mixers.h"
#include "MotorMixerQuadX_DShot.h"

#include <RPM_Filters.h>
#if (__cplusplus >= 202002L)
#include <ranges>
#endif


MotorMixerQuadXDshot::MotorMixerQuadXDshot(uint32_t task_interval_microseconds, uint32_t output_to_motors_denominator, const motor_pins_t& pins, Debug& debug) :
    MotorMixerQuadBase(QUAD_X, &debug),
    _dynamic_idle_controller(task_interval_microseconds/output_to_motors_denominator, debug),
    _rpm_filters(_motor_count, static_cast<float>(task_interval_microseconds) * 0.000001F)
{
    _motors[M0].init(pins.m0);
    _motors[M1].init(pins.m1);
    _motors[M2].init(pins.m2);
    _motors[M3].init(pins.m3);
    // There are a maximum of 12 rpmFilter iterations: 4 motors and up to 3 harmonics for each motor.
    // We want to complete all 12 iterations in less than 1000 microseconds.
    if (task_interval_microseconds >= 1000) {
        _rpm_filter_iteration_count =  12;
    } else if (task_interval_microseconds >= 500) {
        _rpm_filter_iteration_count =  6;
    } else if (task_interval_microseconds >= 250) {
        _rpm_filter_iteration_count =  3;
    } else {
        _rpm_filter_iteration_count =  2;
    }
}

float MotorMixerQuadXDshot::calculate_slowest_motor_hz() const
{
    float slowestMotorHz = _motors[M0].get_motor_hz();
    float motor_hz = _motors[M1].get_motor_hz();
    if (motor_hz < slowestMotorHz) {
        slowestMotorHz = motor_hz;
    }
    motor_hz = _motors[M2].get_motor_hz();
    if (motor_hz < slowestMotorHz) {
        slowestMotorHz = motor_hz;
    }
    motor_hz = _motors[M3].get_motor_hz();
    if (motor_hz < slowestMotorHz) {
        slowestMotorHz = motor_hz;
    }
    return slowestMotorHz;
}

void MotorMixerQuadXDshot::set_motor_config(const motor_config_t& motor_config)
{
    _motor_config = motor_config;
    for (auto& motor : _motors) {
        motor.set_motor_pole_count(motor_config.motor_pole_count);
    }
}

RpmFilters* MotorMixerQuadXDshot::get_rpm_filters()
{
    return &_rpm_filters;
}

const RpmFilters* MotorMixerQuadXDshot::get_rpm_filters() const
{
    return &_rpm_filters;
}

const DynamicIdleController* MotorMixerQuadXDshot::get_dynamic_idle_controller() const
{
    return &_dynamic_idle_controller;
}

void MotorMixerQuadXDshot::set_dynamic_idler_controller_config(const DynamicIdleController::config_t& config)
{
    _dynamic_idle_controller.set_config(config);
}

void MotorMixerQuadXDshot::set_motors_reversed(bool motors_is_reversed)
{
    _motors_is_reversed = motors_is_reversed;
}

void MotorMixerQuadXDshot::output_to_motors(motor_mixer_commands_t& commands, float delta_t, uint32_t tick_count)
{
    (void)tick_count;

    if (motors_is_on()) {
        const float throttleIncrease = (_dynamic_idle_controller.get_minimum_allowed_motor_hz() == 0.0F) ? 0.0F : _dynamic_idle_controller.calculateSpeedIncrease(calculate_slowest_motor_hz(), delta_t);
        commands.throttle += throttleIncrease;
        // set the throttle to value returned by the mixer
        commands.throttle = mix_quad_x(_outputs, commands, _mix_parameters);
    } else {
        _outputs = { 0.0F, 0.0F, 0.0F, 0.0F };
    }
    _throttle_command = commands.throttle;

    // Output to the motors, reading the motor RPM
    // Motor outputs are converted to DShot range [47,2047]
    _motors[M0].write(static_cast<uint16_t>(std::lroundf(2000.0F*std::clamp(_outputs[M0], _mix_parameters.motor_output_min, 1.0F)) + 47)),
    _motors[M0].read();
    _rpm_filters.set_frequency_hz_iteration_start(M0, _motors[M0].get_motor_hz());

    _motors[M1].write(static_cast<uint16_t>(std::lroundf(2000.0F*std::clamp(_outputs[M1], _mix_parameters.motor_output_min, 1.0F)) + 47)),
    _motors[M1].read();
    _rpm_filters.set_frequency_hz_iteration_start(M1, _motors[M1].get_motor_hz());

    _motors[M2].write(static_cast<uint16_t>(std::lroundf(2000.0F*std::clamp(_outputs[M2], _mix_parameters.motor_output_min, 1.0F)) + 47)),
    _motors[M2].read();
    _rpm_filters.set_frequency_hz_iteration_start(M2, _motors[M2].get_motor_hz());

    _motors[M3].write(static_cast<uint16_t>(std::lroundf(2000.0F*std::clamp(_outputs[M3], _mix_parameters.motor_output_min, 1.0F)) + 47)),
    _motors[M3].read();
    _rpm_filters.set_frequency_hz_iteration_start(M3, _motors[M3].get_motor_hz());
}

void MotorMixerQuadXDshot::rpm_filter_set_frequency_hz_iteration_step()
{
    // Perform an rpmFilter iteration step for each motor
    // Note that _rpm_filters.set_frequency_hz_iteration_step is an expensive calculation and runs off a state machine, setting one motor harmonic per iteration
    // so we want to call it even if we do not write to the motors
#if (__cplusplus >= 202002L)
    for ([[maybe_unused]] auto _ : std::views::iota(size_t{0}, _rpm_filter_iteration_count)) {
#else
    for (size_t ii = 0; ii < _rpm_filter_iteration_count; ++ii) {
#endif
        _rpm_filters.set_frequency_hz_iteration_step();
    }
}
