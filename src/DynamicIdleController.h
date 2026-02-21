#pragma once

#include <Filters.h>
#include <pid_controller.h>

class Debug;


struct dynamic_idle_controller_config_t {
    uint8_t dyn_idle_min_rpm_100; // multiply this by 100 to get the actual min RPM
    uint8_t dyn_idle_p_gain;
    uint8_t dyn_idle_i_gain;
    uint8_t dyn_idle_d_gain;
    uint8_t dyn_idle_max_increase;
};

/*!
Dynamic Idle: use PID controller to boost motor speeds so that slowest motor does not go below minimum allowed RPM

A minimum RPM is required because the ESC will desynchronize if the motors turn too slowly (since they won't generate
enough back EMF for the ESC know the position of the rotor relative to the windings).

Note that a simple minimum output value is not sufficient: consider the case where the throttle is cut while hovering,
the quad will start to fall and this falling will generate a reverse torque on the motors which will eventually
overcome the fixed output value. Many types of maneuver can generate this reverse torque.

Instead we have a PID controller that increases output to the motors as the slowest motor nears the minimum allowed RPM.
*/
class DynamicIdleController {
public:
public:
    DynamicIdleController(uint32_t task_interval_microseconds, Debug& debug);
    void set_config(const dynamic_idle_controller_config_t& config);
    const dynamic_idle_controller_config_t& get_config() const { return _config; }
    void set_minimum_allowed_motor_hz(float minimum_allowed_motor_hz);
    float get_minimum_allowed_motor_hz() const { return _minimum_allowed_motor_hz; }
    void set_max_increase(float max_increase) { _max_increase = max_increase; }
    float get_max_increase()const { return _max_increase; }
    float calculateSpeedIncrease(float slowestMotorHz, float delta_t);
    void reset_pid(); //!< for test code
private:
    uint32_t _task_interval_microseconds;
    Debug& _debug;
    float _minimum_allowed_motor_hz {}; // minimum motor Hz, dynamically controlled
    float _max_increase {};
    //float _dynamicIdleMaxIncreaseDelayK {};
    PidController _pid {}; // PID to dynamic idle, ie to ensure slowest motor does not go below min RPS
    PowerTransferFilter1 _DtermFilter {};
    dynamic_idle_controller_config_t _config {};
};
