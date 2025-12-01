#pragma once

#include "DynamicIdleController.h"
#include "RPM_Filters.h"
#include <cstddef>
#include <cstdint>

class Debug;
class DynamicIdleController;

class MotorMixerBase {
public:
    // ordering compatible with Betaflight mixerMode_e enums.
    enum type_e {
        TRICOPTER = 1,
        QUAD_P = 2, QUAD_X = 3,
        BICOPTER = 4,
        GIMBAL = 5,
        Y6 = 6, HEX_P = 7,
        FLYING_WING_SINGLE_PROPELLER = 8,
        Y4 = 9, HEX_X = 10,
        OCTO_QUAD_X = 11, OCTO_FLAT_P = 12, OCTO_FLAT_X = 13,
        AIRPLANE_SINGLE_PROPELLER = 14,
        HELI_120_CCPM = 15,
        HELI_90_DEG = 16,
        VTAIL4 = 17,
        HEX_H = 18,
        PPM_TO_SERVO = 19, // PPM -> servo relay
        DUALCOPTER = 20,
        SINGLECOPTER = 21,
        ATAIL4 = 22,
        CUSTOM = 23,
        CUSTOM_AIRPLANE = 24,
        CUSTOM_TRI = 25,
        QUAD_X_1234 = 26,
        OCTO_XP = 27
    };
    enum protocol_family_e {
        PROTOCOL_FAMILY_UNKNOWN = 0,
        PROTOCOL_FAMILY_PWM,
        PROTOCOL_FAMILY_DSHOT,
    };
    enum motor_protocol_e {
        MOTOR_PROTOCOL_PWM = 0,
        MOTOR_PROTOCOL_ONESHOT125,
        MOTOR_PROTOCOL_ONESHOT42,
        MOTOR_PROTOCOL_MULTISHOT,
        MOTOR_PROTOCOL_BRUSHED,
        MOTOR_PROTOCOL_DSHOT150,
        MOTOR_PROTOCOL_DSHOT300,
        MOTOR_PROTOCOL_DSHOT600,
        MOTOR_PROTOCOL_PROSHOT1000,
        MOTOR_PROTOCOL_DISABLED,
        MOTOR_PROTOCOL_COUNT
    };
    static constexpr float RPMtoDPS { 360.0F / 60.0F };
    static constexpr float DPStoRPM { 60.0F / 360.0F };
    struct commands_t {
        float throttle;
        float roll;
        float pitch;
        float yaw;
    };
    //! parameters to mix function
    struct parameters_t {
        //! minimum motor output, typically set to 5.5% to avoid ESC desynchronization, 
        //! may be set to zero if using dynamic idle control or brushed motors
        float motorOutputMin;
        float motorOutputMax;
        float maxServoAngleRadians; //! used by tricopter
        float undershoot; //! used by test code
        float overshoot; //! used by test code
    };
    struct stm32_motor_pin_t {
        uint8_t port;
        uint8_t pin;
        uint8_t timer;
        uint8_t channel;
    };
    struct mixer_config_t {
        uint8_t type;
        uint8_t yaw_motors_reversed;
    };
    struct motor_device_config_t {
        uint16_t motorPWM_Rate;          // The update rate of motor outputs (50-498Hz)
        uint8_t  motorProtocol;
        uint8_t  motorInversion;        // Active-High vs Active-Low. Useful for brushed FCs converted for brushless operation
        uint8_t  useContinuousUpdate;
        uint8_t  useBurstDshot;
        uint8_t  useDshotTelemetry;
        uint8_t  useDshotEDT;
    };
    struct motor_config_t {
        motor_device_config_t device;
        uint16_t motorIdle;     // percentage of the motor range added to the disarmed value to give the idle value
        uint16_t maxthrottle;   // value of throttle at full power, can be set up to 2000
        uint16_t mincommand;    // value for ESCs when they are not armed. For some specific ESCs this value must be lowered to 900
        uint16_t kv;            // Motor constant estimate RPM under no load
        uint8_t motorPoleCount; // Number of motor poles, used to calculate actual RPM from eRPM
    };
    struct servo_device_config_t {
        // PWM values, in milliseconds, common range is 1000-2000 (1ms to 2ms)
        uint16_t servoCenterPulse;  // This is the value for servos when they should be in the middle. e.g. 1500.
        uint16_t servoPWM_Rate;      // The update rate of servo outputs (50-498Hz)
    };
    typedef struct servo_config_t {
        servo_device_config_t device;
        uint16_t servo_lowpass_freq;            // lowpass servo filter frequency selection; 1/1000ths of loop freq
        uint8_t tri_unarmed_servo;              // send tail servo correction pulses even when unarmed
        uint8_t channelForwardingStartChannel;
    } servoConfig_t;

public:
    virtual ~MotorMixerBase() = default;
    MotorMixerBase(type_e type, size_t motorCount, size_t servoCount, Debug* debug) :
        _type(type),
        _motorCount(motorCount),
        _servoCount(servoCount), 
        _debug(debug),
        _mixerConfig { .type = type, .yaw_motors_reversed = true }
    {}
    MotorMixerBase(type_e type, size_t motorCount, size_t servoCount) : MotorMixerBase(type, motorCount, servoCount, nullptr) {}

    inline type_e getType() const { return _type; }
    inline size_t getMotorCount() const { return _motorCount; }
    inline size_t getServoCount() const { return _servoCount; }
    inline bool motorsIsOn() const { return _motorsIsOn; }
    inline void motorsSwitchOn() { _motorsIsOn = true; }
    inline void motorsSwitchOff() { _motorsIsOn = false; }
    inline bool motorsIsDisabled() const { return _motorsIsDisabled; }

    virtual void setMixerConfig(const mixer_config_t& mixerConfig) { _mixerConfig.type = mixerConfig.type; }
    const mixer_config_t& getMixerConfig() const { return _mixerConfig; }

    virtual void setMotorConfig(const motor_config_t& motorConfig) { _motorConfig = motorConfig; }
    const motor_config_t& getMotorConfig() const { return _motorConfig; }

    inline void setMotorOutputMin(float motorOutputMin) { _mixParameters.motorOutputMin = motorOutputMin; }
    inline float getMotorOutputMin() const { return _mixParameters.motorOutputMin; }

    virtual void outputToMotors(commands_t& commands, float deltaT, uint32_t tickCount) { (void)commands; (void)deltaT; (void)tickCount; }
    virtual float getMotorOutput(size_t motorIndex) const { (void)motorIndex; return 0.0F; }

    virtual bool canReportPosition(size_t motorIndex) const { (void)motorIndex; return false; }
    virtual void resetAllEncoders() {}
    virtual void readEncoder(size_t motorIndex) { (void)motorIndex; }
    virtual int32_t getEncoder(size_t motorIndex) const { (void)motorIndex; return 0; }
    virtual uint32_t getStepsPerRevolution(size_t motorIndex) const { (void)motorIndex; return 0; }

    virtual bool canReportSpeed(size_t motorIndex) const { (void)motorIndex; return false; }
    virtual int32_t getMotorRPM(size_t motorIndex) const { (void)motorIndex; return 0; }
    virtual float getMotorSpeedDPS(size_t motorIndex) const { return static_cast<float>(getMotorRPM(motorIndex)) * RPMtoDPS; }
    virtual float getMotorFrequencyHz(size_t motorIndex) const { (void)motorIndex; return 0; }

    virtual float getMixerThrottleCommand() const { return 0.0F; } // for blackbox recording

    virtual void rpmFilterSetFrequencyHzIterationStep() {};
    virtual RPM_Filters* getRPM_Filters() { return nullptr; }
    virtual const RPM_Filters* getRPM_Filters() const { return nullptr; }

    virtual const DynamicIdleController* getDynamicIdleController() const { return nullptr; }
    virtual void setDynamicIdlerControllerConfig(const DynamicIdleController::config_t& config) { (void)config; }
protected:
    const type_e _type;
    const size_t _motorCount;
    const size_t _servoCount;
    Debug* _debug;
    mixer_config_t _mixerConfig;
    motor_config_t _motorConfig {
        .device = {
            .motorPWM_Rate = 480, // 16000 for brushed
            .motorProtocol = MOTOR_PROTOCOL_DSHOT300,
            .motorInversion = false,
            .useContinuousUpdate = true,
            .useBurstDshot = false,
            .useDshotTelemetry = false,
            .useDshotEDT = false,
        },
        .motorIdle = 550, // 700 for brushed
        .maxthrottle = 2000, 
        .mincommand = 1000, 
        .kv = 1960, 
        .motorPoleCount = 14
    };
    parameters_t _mixParameters {
        .motorOutputMin = 0.0F,
        .motorOutputMax = 1.0F,
        .maxServoAngleRadians = 0.0F,
        .undershoot = 0.0F,
        .overshoot = 0.0F,
    };
    bool _motorsIsOn {false};
    bool _motorsIsDisabled {false};
};
