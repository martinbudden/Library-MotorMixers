#include "DynamicIdleController.h"
#include "Debug.h"

#include <algorithm>


inline float clamp(float value, float min, float max)
{
#if (__cplusplus >= 202002L)
    return std::clamp(value, min, max);
#else
    return (value < min) ? min : (value > max) ? max : value;
#endif
}


DynamicIdleController::DynamicIdleController(uint32_t task_interval_microseconds, Debug& debug) :
    _task_interval_microseconds(task_interval_microseconds),
    _debug(debug)
{
}

void DynamicIdleController::set_config(const dynamic_idle_controller_config_t& config)
{
    _config = config;

    _max_increase = static_cast<float>(config.dyn_idle_max_increase) * 0.001F;

    _minimum_allowed_motor_hz = static_cast<float>(config.dyn_idle_min_rpm_100) * 100.0F / 60.0F;
    _PID.setSetpoint(_minimum_allowed_motor_hz);

    // use Betaflight multiplier for compatibility with Betaflight Configurator
    _PID.setP(static_cast<float>(config.dyn_idle_p_gain) * 0.00015F);

    const float delta_t = static_cast<float>(_task_interval_microseconds) * 0.000001F;

    _PID.setI(static_cast<float>(config.dyn_idle_i_gain) * 0.01F * delta_t);
    // limit I-term to range [0, _max_increase]
    _PID.setIntegralMax(_max_increase);
    _PID.setIntegralMin(0.0F);

    _PID.setD(static_cast<float>(config.dyn_idle_i_gain) * 0.0000003F / delta_t);
    _DtermFilter.init(800.0F * delta_t / 20.0F); //approx 20ms D delay, arbitrarily suits many motors
}

void DynamicIdleController::set_minimum_allowed_motor_hz(float minimum_allowed_motor_hz)
{
    _minimum_allowed_motor_hz = minimum_allowed_motor_hz;
    _PID.setSetpoint(_minimum_allowed_motor_hz);
}

float DynamicIdleController::calculateSpeedIncrease(float slowestMotorHz, float delta_t)
{
    if (_minimum_allowed_motor_hz == 0.0F) {
        // if motors are allowed to stop, then no speed increase is needed
        return  0.0F;
    }

    const float slowestMotorHzDeltaFiltered = _DtermFilter.filter(slowestMotorHz - _PID.getPreviousMeasurement());
    float speedIncrease = _PID.updateDelta(slowestMotorHz, slowestMotorHzDeltaFiltered, delta_t);

    speedIncrease = clamp(speedIncrease, 0.0F, _max_increase);

    if (_debug.getMode() == DEBUG_DYN_IDLE) {
        const PIDF::error_t error = _PID.getError();
        _debug.set(0, static_cast<int16_t>(std::max(-1000L, std::lroundf(error.P * 10000))));
        _debug.set(1, static_cast<int16_t>(std::lroundf(error.I * 10000)));
        _debug.set(2, static_cast<int16_t>(std::lroundf(error.D * 10000)));
    }

    return speedIncrease;
}

void DynamicIdleController::resetPID()
{
    _PID.resetIntegral();
}
