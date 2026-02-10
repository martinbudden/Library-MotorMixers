#include "DynamicIdleController.h"
#include "Mixers.h"
#include "MotorMixerQuadXDshotBitbang.h"

#include <RpmFilters.h>

#include <cmath>
#if (__cplusplus >= 202002L)
#include <ranges>
#endif


MotorMixerQuadXDshotBitbang::MotorMixerQuadXDshotBitbang(uint32_t task_interval_microseconds, uint32_t output_to_motors_denominator, const stm32_motor_pins_t& pins, Debug& debug) :
    MotorMixerQuadBase(QUAD_X, &debug),
    _dynamic_idle_controller(task_interval_microseconds/output_to_motors_denominator, debug),
    _rpm_filters(_motor_count, static_cast<float>(task_interval_microseconds) * 0.000001F)
{
    (void)pins; // !!TODO: set pins

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

float MotorMixerQuadXDshotBitbang::calculate_slowest_motor_hz() const
{
    float slowestMotorHz = _motor_frequencies_hz[M0];
    float motor_hz = _motor_frequencies_hz[M1];
    if (motor_hz < slowestMotorHz) {
        slowestMotorHz = motor_hz;
    }
    motor_hz = _motor_frequencies_hz[M2];
    if (motor_hz < slowestMotorHz) {
        slowestMotorHz = motor_hz;
    }
    motor_hz = _motor_frequencies_hz[M3];
    if (motor_hz < slowestMotorHz) {
        slowestMotorHz = motor_hz;
    }
    return slowestMotorHz;
}

void MotorMixerQuadXDshotBitbang::set_motor_config(const motor_config_t& motor_config)
{
    _motor_config = motor_config;
    _erpm_to_hz = 2.0F * (100.0F / SECONDS_PER_MINUTE) / static_cast<float>(motor_config.motor_pole_count);
}

RpmFilters* MotorMixerQuadXDshotBitbang::get_rpm_filters()
{
    return &_rpm_filters;
}

const RpmFilters* MotorMixerQuadXDshotBitbang::get_rpm_filters() const
{
    return &_rpm_filters;
}

const DynamicIdleController* MotorMixerQuadXDshotBitbang::get_dynamic_idle_controller() const
{
    return &_dynamic_idle_controller;
}

void MotorMixerQuadXDshotBitbang::set_dynamic_idler_controller_config(const DynamicIdleController::config_t& config)
{
    _dynamic_idle_controller.set_config(config);
}

void MotorMixerQuadXDshotBitbang::set_motors_reversed(bool motors_is_reversed)
{
    _motors_is_reversed = motors_is_reversed;
}

void MotorMixerQuadXDshotBitbang::output_to_motors(motor_mixer_commands_t& commands, float delta_t, uint32_t tick_count)
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

    // convert motor output to DShot range [47, 2047]
    _esc_dshot.output_to_motors(
        static_cast<uint16_t>(std::lroundf(2000.0F*std::clamp(_outputs[M0], _mix_parameters.motor_output_min, 1.0F)) + 47),
        static_cast<uint16_t>(std::lroundf(2000.0F*std::clamp(_outputs[M1], _mix_parameters.motor_output_min, 1.0F)) + 47),
        static_cast<uint16_t>(std::lroundf(2000.0F*std::clamp(_outputs[M2], _mix_parameters.motor_output_min, 1.0F)) + 47),
        static_cast<uint16_t>(std::lroundf(2000.0F*std::clamp(_outputs[M3], _mix_parameters.motor_output_min, 1.0F)) + 47)
    );

    // read the motor RPMs
    _motor_frequencies_hz[M0] = static_cast<float>(_esc_dshot.get_motor_erpm(M0))*_erpm_to_hz;
    _motor_frequencies_hz[M1] = static_cast<float>(_esc_dshot.get_motor_erpm(M1))*_erpm_to_hz;
    _motor_frequencies_hz[M2] = static_cast<float>(_esc_dshot.get_motor_erpm(M2))*_erpm_to_hz;
    _motor_frequencies_hz[M3] = static_cast<float>(_esc_dshot.get_motor_erpm(M3))*_erpm_to_hz;

    _rpm_filters.set_frequency_hz_iteration_start(M0, _motor_frequencies_hz[M0]);
    _rpm_filters.set_frequency_hz_iteration_start(M1, _motor_frequencies_hz[M1]);
    _rpm_filters.set_frequency_hz_iteration_start(M1, _motor_frequencies_hz[M2]);
    _rpm_filters.set_frequency_hz_iteration_start(M3, _motor_frequencies_hz[M3]);

}

void MotorMixerQuadXDshotBitbang::rpm_filter_set_frequency_hz_iteration_step()
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
