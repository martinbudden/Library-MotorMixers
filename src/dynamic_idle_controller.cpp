#include "dynamic_idle_controller.h"

#include <algorithm>
#include <debug.h>


inline float clamp(float value, float min, float max)
{
#if (__cplusplus >= 202002L)
    return std::clamp(value, min, max);
#else
    return (value < min) ? min : (value > max) ? max : value;
#endif
}


DynamicIdleController::DynamicIdleController(uint32_t task_interval_microseconds) :
    _task_interval_microseconds(task_interval_microseconds)
{
}

void DynamicIdleController::set_config(const dynamic_idle_controller_config_t& config)
{
    _config = config;

    _max_increase = static_cast<float>(config.dyn_idle_max_increase) * 0.001F;

    _minimum_allowed_motor_hz = static_cast<float>(config.dyn_idle_min_rpm_100) * 100.0F / 60.0F;
    _pid.set_setpoint(_minimum_allowed_motor_hz);

    // use Betaflight multiplier for compatibility with Betaflight Configurator
    _pid.set_p(static_cast<float>(config.dyn_idle_p_gain) * 0.00015F);

    const float delta_t = static_cast<float>(_task_interval_microseconds) * 0.000001F;

    _pid.set_i(static_cast<float>(config.dyn_idle_i_gain) * 0.01F * delta_t);
    // limit Iterm to range [0, _max_increase]
    _pid.set_integral_max(_max_increase);
    _pid.set_integral_min(0.0F);

    _pid.set_d(static_cast<float>(config.dyn_idle_i_gain) * 0.0000003F / delta_t);
    _dterm_filter.init(800.0F * delta_t / 20.0F); //approx 20ms D delay, arbitrarily suits many motors
}

void DynamicIdleController::set_minimum_allowed_motor_hz(float minimum_allowed_motor_hz)
{
    _minimum_allowed_motor_hz = minimum_allowed_motor_hz;
    _pid.set_setpoint(_minimum_allowed_motor_hz);
}

float DynamicIdleController::calculate_speed_increase(float slowestMotorHz, float delta_t, Debug& debug)
{
    if (_minimum_allowed_motor_hz == 0.0F) {
        // if motors are allowed to stop, then no speed increase is needed
        return  0.0F;
    }

    const float slowestMotorHzDeltaFiltered = _dterm_filter.filter(slowestMotorHz - _pid.get_previous_measurement());
    float speed_increase = _pid.update_delta(slowestMotorHz, slowestMotorHzDeltaFiltered, delta_t);

    speed_increase = clamp(speed_increase, 0.0F, _max_increase);

    if (debug.getMode() == DEBUG_DYN_IDLE) {
        const pid_error_t error = _pid.get_error();
        debug.set(0, static_cast<int16_t>(std::max(-1000L, std::lroundf(error.p * 10000))));
        debug.set(1, static_cast<int16_t>(std::lroundf(error.i * 10000)));
        debug.set(2, static_cast<int16_t>(std::lroundf(error.d * 10000)));
    }

    return speed_increase;
}

void DynamicIdleController::reset_pid()
{
    _pid.reset_integral();
}
