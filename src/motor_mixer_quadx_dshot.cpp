#include "dynamic_idle_controller.h"
#include "mixers.h"
#include "motor_mixer_message_queue.h"
#include "motor_mixer_quadx_dshot.h"
#include "rpm_filters.h"

#if (__cplusplus >= 202002L)
#include <ranges>
#endif


MotorMixerQuadXDshot::MotorMixerQuadXDshot(uint32_t task_interval_microseconds, uint8_t output_to_motors_denominator, const motor_pins_t& pins) :
    MotorMixerQuadBase(QUAD_X, output_to_motors_denominator),
    _dynamic_idle_controller(task_interval_microseconds/output_to_motors_denominator)
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
    float slowest_motor_hz = _motors[M0].get_motor_hz();
    float motor_hz = _motors[M1].get_motor_hz();
    if (motor_hz < slowest_motor_hz) {
        slowest_motor_hz = motor_hz;
    }
    motor_hz = _motors[M2].get_motor_hz();
    if (motor_hz < slowest_motor_hz) {
        slowest_motor_hz = motor_hz;
    }
    motor_hz = _motors[M3].get_motor_hz();
    if (motor_hz < slowest_motor_hz) {
        slowest_motor_hz = motor_hz;
    }
    return slowest_motor_hz;
}

void MotorMixerQuadXDshot::set_motor_config(const motor_config_t& motor_config)
{
    _motor_config = motor_config;
    for (auto& motor : _motors) {
        motor.set_motor_pole_count(motor_config.motor_pole_count);
    }
}

const dynamic_idle_controller_config_t* MotorMixerQuadXDshot::get_dynamic_idle_config() const
{
    return &_dynamic_idle_controller.get_config();
}

void MotorMixerQuadXDshot::set_dynamic_idle_controller_config(const dynamic_idle_controller_config_t& config)
{
    _dynamic_idle_controller.set_config(config);
}

void MotorMixerQuadXDshot::set_motors_reversed(bool motors_is_reversed)
{
    _motors_is_reversed = motors_is_reversed;
}

void MotorMixerQuadXDshot::output_to_motors(const motor_mixer_message_queue_item_t& queue_item, RpmFilters* rpm_filters, float delta_t, uint32_t tick_count, Debug& debug)
{
    (void)tick_count;
    (void)debug;

    // Output to motors every _output_to_motors_denominator times output_to_motors is called.
    ++_output_to_mixer_count;
    if (_output_to_mixer_count >= _output_to_motors_denominator || !motors_is_on()) {
        _output_to_mixer_count = 0;

        motor_mixer_commands_t commands {
            .throttle  = queue_item.throttle,
            // scale roll, pitch, and yaw from DPS range to [-1.0F, 1.0F]
            .roll   = queue_item.roll_dps * MIXER_OUTPUT_SCALE_FACTOR,
            .pitch  = queue_item.pitch_dps * MIXER_OUTPUT_SCALE_FACTOR,
            .yaw    = queue_item.yaw_dps * MIXER_OUTPUT_SCALE_FACTOR
        };

        if (motors_is_on()) {
            const float throttle_increase = (_dynamic_idle_controller.get_minimum_allowed_motor_hz() == 0.0F) 
                ? 0.0F
                : _dynamic_idle_controller.calculate_speed_increase(calculate_slowest_motor_hz(), delta_t, debug);
            commands.throttle += throttle_increase;
            // set the throttle to value returned by the mixer
            _throttle_command = mix_quad_x(_outputs, commands, _mix_parameters);
        } else {
            _outputs = { 0.0F, 0.0F, 0.0F, 0.0F };
        }

        // Output to the motors, reading the motor RPM
        // Motor outputs are converted to DShot range [47,2047]
        _motors[M0].write(static_cast<uint16_t>(std::lroundf(2000.0F*clamp(_outputs[M0], _mix_parameters.motor_output_min, 1.0F)) + 47)),
        _motors[M0].read();
        rpm_filters->set_frequency_hz_iteration_start(M0, _motors[M0].get_motor_hz());

        _motors[M1].write(static_cast<uint16_t>(std::lroundf(2000.0F*clamp(_outputs[M1], _mix_parameters.motor_output_min, 1.0F)) + 47)),
        _motors[M1].read();
        rpm_filters->set_frequency_hz_iteration_start(M1, _motors[M1].get_motor_hz());

        _motors[M2].write(static_cast<uint16_t>(std::lroundf(2000.0F*clamp(_outputs[M2], _mix_parameters.motor_output_min, 1.0F)) + 47)),
        _motors[M2].read();
        rpm_filters->set_frequency_hz_iteration_start(M2, _motors[M2].get_motor_hz());

        _motors[M3].write(static_cast<uint16_t>(std::lroundf(2000.0F*clamp(_outputs[M3], _mix_parameters.motor_output_min, 1.0F)) + 47)),
        _motors[M3].read();
        rpm_filters->set_frequency_hz_iteration_start(M3, _motors[M3].get_motor_hz());
    }

    // perform an RPM filter iteration step, even if we have not output to the motors
    rpm_filter_set_frequency_hz_iteration_step(rpm_filters);
}

void MotorMixerQuadXDshot::rpm_filter_set_frequency_hz_iteration_step(RpmFilters* rpm_filters)
{
    // Perform an rpmFilter iteration step for each motor
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
