#pragma once

#include "DynamicIdleController.h"
#include "RpmFilters.h"
#include <cstddef>

class Debug;
class DynamicIdleController;

struct motor_mixer_commands_t {
    float throttle;
    float roll;
    float pitch;
    float yaw;
};

struct motor_mixer_parameters_t {
    //! minimum motor output, typically set to 5.5% to avoid ESC desynchronization,
    //! may be set to zero if using dynamic idle control or brushed motors
    float motor_output_min;
    float motor_output_max;
    float max_servo_angle_radians; //! used by tricopter
    float undershoot; //! used by test code
    float overshoot; //! used by test code
};

struct mixer_config_t {
    uint8_t type;
    uint8_t yaw_motors_reversed;
};

struct motor_device_config_t {
    uint16_t motor_pwm_rate;          // The update rate of motor outputs (50-498Hz)
    uint8_t  motor_protocol;
    uint8_t  motor_inversion;        // Active-High vs Active-Low. Useful for brushed FCs converted for brushless operation
    uint8_t  use_continuous_update;
    uint8_t  use_burst_dshot;
    uint8_t  use_dshot_telemetry;
    uint8_t  use_dshot_edt;
};

struct motor_config_t {
    motor_device_config_t device;
    uint16_t motor_idle;     // percentage of the motor range added to the disarmed value to give the idle value
    uint16_t max_throttle;   // value of throttle at full power, can be set up to 2000
    uint16_t min_command;    // value for ESCs when they are not armed. For some specific ESCs this value must be lowered to 900
    uint16_t kv;            // Motor constant estimate RPM under no load
    uint8_t motor_pole_count; // Number of motor poles, used to calculate actual RPM from eRPM
};

struct servo_device_config_t {
    // PWM values, in milliseconds, common range is 1000-2000 (1ms to 2ms)
    uint16_t servo_center_pulse;  // This is the value for servos when they should be in the middle. e.g. 1500.
    uint16_t servo_pwm_Rate;      // The update rate of servo outputs (50-498Hz)
};

struct servo_config_t {
    servo_device_config_t device;
    uint16_t servo_lowpass_freq;            // lowpass servo filter frequency selection; 1/1000ths of loop freq
    uint8_t tri_unarmed_servo;              // send tail servo correction pulses even when unarmed
    uint8_t channelForwardingStartChannel;
};

class MotorMixerBase {
public:
    // constants compatible with Betaflight mixerMode_e enums.
    static constexpr uint8_t TRICOPTER = 1;
    static constexpr uint8_t QUAD_P = 2; 
    static constexpr uint8_t QUAD_X = 3;
    static constexpr uint8_t BICOPTER = 4;
    static constexpr uint8_t GIMBAL = 5;
    static constexpr uint8_t Y6 = 6;
    static constexpr uint8_t HEX_P = 7;
    static constexpr uint8_t FLYING_WING_SINGLE_PROPELLER = 8;
    static constexpr uint8_t Y4 = 9;
    static constexpr uint8_t HEX_X = 10;
    static constexpr uint8_t OCTO_QUAD_X = 11;
    static constexpr uint8_t OCTO_FLAT_P = 12;
    static constexpr uint8_t OCTO_FLAT_X = 13;
    static constexpr uint8_t AIRPLANE_SINGLE_PROPELLER = 14;
    static constexpr uint8_t HELI_120_CCPM = 15;
    static constexpr uint8_t HELI_90_DEG = 16;
    static constexpr uint8_t VTAIL4 = 17;
    static constexpr uint8_t HEX_H = 18;
    static constexpr uint8_t PPM_TO_SERVO = 19; // PPM -> servo relay
    static constexpr uint8_t DUALCOPTER = 20;
    static constexpr uint8_t SINGLECOPTER = 21;
    static constexpr uint8_t ATAIL4 = 22;
    static constexpr uint8_t CUSTOM = 23;
    static constexpr uint8_t CUSTOM_AIRPLANE = 24;
    static constexpr uint8_t CUSTOM_TRI = 25;
    static constexpr uint8_t QUAD_X_1234 = 26;
    static constexpr uint8_t OCTO_XP = 27;
public:
    virtual ~MotorMixerBase() = default;
    MotorMixerBase(uint8_t type, size_t motor_count, size_t servo_count, Debug* debug) :
        _type(type),
        _motor_count(motor_count),
        _servo_count(servo_count),
        _debug(debug),
        _mixer_config { .type = type, .yaw_motors_reversed = true }
    {}
    MotorMixerBase(uint8_t type, size_t motor_count, size_t servo_count) : MotorMixerBase(type, motor_count, servo_count, nullptr) {}
public:
    static constexpr uint8_t PROTOCOL_FAMILY_UNKNOWN = 0;
    static constexpr uint8_t PROTOCOL_FAMILY_PWM = 1;
    static constexpr uint8_t PROTOCOL_FAMILY_DSHOT = 2;

    static constexpr uint8_t MOTOR_PROTOCOL_PWM = 0;
    static constexpr uint8_t MOTOR_PROTOCOL_ONESHOT125 = 1;
    static constexpr uint8_t MOTOR_PROTOCOL_ONESHOT42 = 2;
    static constexpr uint8_t MOTOR_PROTOCOL_MULTISHOT = 3;
    static constexpr uint8_t MOTOR_PROTOCOL_BRUSHED = 4;
    static constexpr uint8_t MOTOR_PROTOCOL_DSHOT150 = 5;
    static constexpr uint8_t MOTOR_PROTOCOL_DSHOT300 = 6;
    static constexpr uint8_t MOTOR_PROTOCOL_DSHOT600 = 7;
    static constexpr uint8_t MOTOR_PROTOCOL_PROSHOT1000 = 8;
    static constexpr uint8_t MOTOR_PROTOCOL_DISABLED = 9;
    static constexpr uint8_t MOTOR_PROTOCOL_COUNT = 10;

    static constexpr float RPM_TO_DPS { 360.0F / 60.0F };
    static constexpr float DPS_TO_RPM { 60.0F / 360.0F };

    // Betaflight compatible mixer output scale factor: scales roll, pitch, and yaw from DPS range to [-1.0F, 1.0F]
    static constexpr float MIXER_OUTPUT_SCALE_FACTOR = 0.001F;

    //! parameters to mix function
    struct stm32_motor_pin_t {
        uint8_t port;
        uint8_t pin;
        uint8_t timer;
        uint8_t channel;
    };
public:
    uint8_t get_type() const { return _type; }
    size_t get_motor_count() const { return _motor_count; }
    size_t get_servo_count() const { return _servo_count; }
    bool motors_is_on() const { return _motors_is_on; }
    void motors_switch_on() { _motors_is_on = true; }
    void motors_switch_off() { _motors_is_on = false; }
    bool motors_is_disabled() const { return _motors_is_disabled; }
    bool motors_is_reversed() const { return _motors_is_reversed; }

    virtual void set_mixer_config(const mixer_config_t& mixer_config) { _mixer_config.type = mixer_config.type; }
    const mixer_config_t& get_mixer_config() const { return _mixer_config; }

    virtual void set_motor_config(const motor_config_t& motor_config) { _motor_config = motor_config; }
    const motor_config_t& get_motor_config() const { return _motor_config; }

    void set_motor_output_min(float motor_output_min) { _mix_parameters.motor_output_min = motor_output_min; }
    float get_motor_output_min() const { return _mix_parameters.motor_output_min; }

    virtual void set_motors_reversed(bool motors_is_reversed) { _motors_is_reversed = motors_is_reversed; }
    virtual void output_to_motors(motor_mixer_commands_t& commands, RpmFilters* rpm_filters, float delta_t, uint32_t tick_count) { (void)commands; (void)rpm_filters; (void)delta_t; (void)tick_count; }
    virtual float get_motor_output(size_t motor_index) const { (void)motor_index; return 0.0F; }

    virtual bool can_report_position(size_t motor_index) const { (void)motor_index; return false; }
    virtual void reset_all_encoders() {}
    virtual void read_all_encoders() {}
    virtual void read_encoder(size_t motor_index) { (void)motor_index; }
    virtual int32_t get_encoder(size_t motor_index) const { (void)motor_index; return 0; }
    virtual uint32_t get_steps_per_revolution(size_t motor_index) const { (void)motor_index; return 0; }

    virtual bool can_report_speed(size_t motor_index) const { (void)motor_index; return false; }
    virtual int32_t get_motor_rpm(size_t motor_index) const { (void)motor_index; return 0; }
    virtual float get_motor_speed_dps(size_t motor_index) const { return static_cast<float>(get_motor_rpm(motor_index)) * RPM_TO_DPS; }
    virtual float get_motor_frequency_hz(size_t motor_index) const { (void)motor_index; return 0; }

    float get_throttle_command() const { return _throttle_command; } // for blackbox recording

    virtual void rpm_filter_set_frequency_hz_iteration_step(RpmFilters* rpm_filters) { (void)rpm_filters; };

    virtual const dynamic_idle_controller_config_t* get_dynamic_idle_config() const { return nullptr; }
    virtual void set_dynamic_idle_controller_config(const dynamic_idle_controller_config_t& config) { (void)config; }
protected:
    const uint8_t _type;
    const size_t _motor_count;
    const size_t _servo_count;
    Debug* _debug;
    mixer_config_t _mixer_config;
    motor_config_t _motor_config {
        .device = {
            .motor_pwm_rate = 480, // 16000 for brushed
            .motor_protocol = MOTOR_PROTOCOL_DSHOT300,
            .motor_inversion = false,
            .use_continuous_update = true,
            .use_burst_dshot = false,
            .use_dshot_telemetry = false,
            .use_dshot_edt = false,
        },
        .motor_idle = 550, // 700 for brushed
        .max_throttle = 2000,
        .min_command = 1000,
        .kv = 1960,
        .motor_pole_count = 14
    };
    motor_mixer_parameters_t _mix_parameters {
        .motor_output_min = 0.0F,
        .motor_output_max = 1.0F,
        .max_servo_angle_radians = 0.0F,
        .undershoot = 0.0F,
        .overshoot = 0.0F,
    };
    float _throttle_command {0.0F}; //!< used for blackbox recording
    bool _motors_is_on {false};
    bool _motors_is_disabled {false};
    bool _motors_is_reversed {false}; //!< reversed motors typically used to flip multi-rotor after a crash
};
