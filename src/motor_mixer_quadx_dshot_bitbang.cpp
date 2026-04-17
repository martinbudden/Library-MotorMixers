#include "dynamic_idle_controller.h"
#include "mixers.h"
#include "motor_commands.h"
#include "motor_mixer_quadx_dshot_bitbang.h"
#include "rpm_filters.h"

#include <algorithm>
#include <cmath>

#if (__cplusplus >= 202002L)
#include <ranges>
#endif


MotorMixerQuadXDshotBitbang::MotorMixerQuadXDshotBitbang(uint32_t task_interval_microseconds, uint8_t output_to_motors_denominator, const stm32_motor_pins_t& pins) :
    MotorMixerQuadBase(QUAD_X, output_to_motors_denominator),
    _dynamic_idle_controller(task_interval_microseconds/output_to_motors_denominator)
{
    (void)pins; // !!TODO: set pins

    // There are a maximum of 12 rpm_filter iterations: 4 motors and up to 3 harmonics for each motor.
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
    float slowest_motor_hz = _motor_frequencies_hz[M0];
    float motor_hz = _motor_frequencies_hz[M1];
    if (motor_hz < slowest_motor_hz) {
        slowest_motor_hz = motor_hz;
    }
    motor_hz = _motor_frequencies_hz[M2];
    if (motor_hz < slowest_motor_hz) {
        slowest_motor_hz = motor_hz;
    }
    motor_hz = _motor_frequencies_hz[M3];
    if (motor_hz < slowest_motor_hz) {
        slowest_motor_hz = motor_hz;
    }
    return slowest_motor_hz;
}

void MotorMixerQuadXDshotBitbang::set_motor_config(const motor_config_t& motor_config)
{
    _motor_config = motor_config;
    _erpm_to_hz = 2.0F * (100.0F / SECONDS_PER_MINUTE) / static_cast<float>(motor_config.motor_pole_count);
}

const dynamic_idle_controller_config_t* MotorMixerQuadXDshotBitbang::get_dynamic_idle_config() const
{
    return &_dynamic_idle_controller.get_config();
}

void MotorMixerQuadXDshotBitbang::set_dynamic_idle_controller_config(const dynamic_idle_controller_config_t& config)
{
    _dynamic_idle_controller.set_config(config);
}

void MotorMixerQuadXDshotBitbang::set_motors_reversed(bool motors_is_reversed)
{
    _motors_is_reversed = motors_is_reversed;
}

void MotorMixerQuadXDshotBitbang::output_to_motors(const motor_commands_t& motor_commands, RpmFilters* rpm_filters, float delta_t, uint32_t tick_count, Debug& debug)
{
    (void)tick_count;

    // Output to motors every _output_to_motors_denominator times output_to_motors is called.
    ++_output_to_mixer_count;
    if (_output_to_mixer_count >= _output_to_motors_denominator || !motors_is_on()) {
        _output_to_mixer_count = 0;

        motor_mixer_commands_t commands {
            .throttle  = motor_commands.throttle,
            // scale roll, pitch, and yaw from DPS range to [-1.0F, 1.0F]
            .roll   = motor_commands.roll_dps * MIXER_OUTPUT_SCALE_FACTOR,
            .pitch  = motor_commands.pitch_dps * MIXER_OUTPUT_SCALE_FACTOR,
            .yaw    = motor_commands.yaw_dps * MIXER_OUTPUT_SCALE_FACTOR
        };
        if (motors_is_on()) {
            const float throttle_increase = (_dynamic_idle_controller.get_minimum_allowed_motor_hz() == 0.0F) ? 0.0F : _dynamic_idle_controller.calculate_speed_increase(calculate_slowest_motor_hz(), delta_t, debug);
            commands.throttle += throttle_increase;
            // set the throttle to value returned by the mixer
            _outputs = mix_quad_x(commands, _mix_parameters);
        } else {
            _outputs = { 0.0F, 0.0F, 0.0F, 0.0F };
        }

        // convert motor output to DShot range [47, 2047]
        _esc_dshot.output_to_motors(
            static_cast<uint16_t>(std::lroundf(2000.0F*clamp(_outputs[M0], _mix_parameters.motor_output_min, 1.0F)) + 47),
            static_cast<uint16_t>(std::lroundf(2000.0F*clamp(_outputs[M1], _mix_parameters.motor_output_min, 1.0F)) + 47),
            static_cast<uint16_t>(std::lroundf(2000.0F*clamp(_outputs[M2], _mix_parameters.motor_output_min, 1.0F)) + 47),
            static_cast<uint16_t>(std::lroundf(2000.0F*clamp(_outputs[M3], _mix_parameters.motor_output_min, 1.0F)) + 47)
        );

        // read the motor RPMs
        _motor_frequencies_hz[M0] = static_cast<float>(_esc_dshot.get_motor_erpm(M0))*_erpm_to_hz;
        _motor_frequencies_hz[M1] = static_cast<float>(_esc_dshot.get_motor_erpm(M1))*_erpm_to_hz;
        _motor_frequencies_hz[M2] = static_cast<float>(_esc_dshot.get_motor_erpm(M2))*_erpm_to_hz;
        _motor_frequencies_hz[M3] = static_cast<float>(_esc_dshot.get_motor_erpm(M3))*_erpm_to_hz;

        rpm_filters->set_frequency_hz_iteration_start(M0, _motor_frequencies_hz[M0]);
        rpm_filters->set_frequency_hz_iteration_start(M1, _motor_frequencies_hz[M1]);
        rpm_filters->set_frequency_hz_iteration_start(M1, _motor_frequencies_hz[M2]);
        rpm_filters->set_frequency_hz_iteration_start(M3, _motor_frequencies_hz[M3]);
    }

    rpm_filter_set_frequency_hz_iteration_step(rpm_filters);
}

void MotorMixerQuadXDshotBitbang::rpm_filter_set_frequency_hz_iteration_step(RpmFilters* rpm_filters)
{
    // Perform an rpm_filter iteration step for each motor
    // Note that rpm_filters->set_frequency_hz_iteration_step is an expensive calculation and runs off a state machine, setting one motor harmonic per iteration
    // so we want to call it even if we do not write to the motors
#if (__cplusplus >= 202002L)
    for ([[maybe_unused]] auto _ : std::views::iota(size_t{0}, _rpm_filter_iteration_count)) {
#else
    for (size_t ii = 0; ii < _rpm_filter_iteration_count; ++ii) {
#endif
        rpm_filters->set_frequency_hz_iteration_step();
    }
}
