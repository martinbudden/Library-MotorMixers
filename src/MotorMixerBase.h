#pragma once

#include "DynamicIdleController.h"
#include "RPM_Filters.h"
#include <cstddef>
#include <cstdint>

class Debug;
class DynamicIdleController;

class MotorMixerBase {
public:
    enum type_e { BICOPTER, TRICOPTER, QUAD_X, HEX_X, OCTO_X, FLYING_WING_SINGLE_PROPELLER, AIRPLANE_SINGLE_PROPELLER };
    struct commands_t {
        float throttle;
        float roll;
        float pitch;
        float yaw;
    };
    struct stm32_motor_pin_t {
        uint8_t port;
        uint8_t pin;
        uint8_t timer;
        uint8_t channel;
    };
    struct motorConfig_t {
        uint16_t motorIdle;     // percentage of the motor range added to the disarmed value to give the idle value
        uint16_t maxthrottle;   // value of throttle at full power, can be set up to 2000
        uint16_t mincommand;    // value for ESCs when they are not armed. For some specific ESCs this value must be lowered to 900
        uint16_t kv;            // Motor constant estimate RPM under no load
        uint8_t motorPoleCount; // Number of motor poles, used to calculate actual RPM from eRPM
    };
public:
    virtual ~MotorMixerBase() = default;
    MotorMixerBase(type_e type, size_t motorCount, size_t servoCount, Debug& debug) :
        _type(type), _motorCount(motorCount), _servoCount(servoCount), _debug(debug) {}

    inline size_t getMotorCount() const { return _motorCount; }
    inline size_t getServoCount() const { return _servoCount; }
    inline bool motorsIsOn() const { return _motorsIsOn; }
    inline void motorsSwitchOn() { _motorsIsOn = true; }
    inline void motorsSwitchOff() { _motorsIsOn = false; }
    inline bool motorsIsDisabled() const { return _motorsIsDisabled; }

    virtual void setMotorConfig(const motorConfig_t& motorConfig) { _motorConfig = motorConfig; }
    const motorConfig_t& getMotorConfig() const { return _motorConfig; }

    inline void setMotorOutputMin(float motorOutputMin) { _motorOutputMin = motorOutputMin; }
    inline float getMotorOutputMin() const { return _motorOutputMin; }

    virtual void outputToMotors(commands_t& commands, float deltaT, uint32_t tickCount) { (void)commands; (void)deltaT; (void)tickCount; }
    virtual float getMotorOutput(size_t motorIndex) const { (void)motorIndex; return 0.0F; }

    virtual void resetEncoderToZero(size_t motorIndex) { (void)motorIndex; }
    virtual void readEncoder(size_t motorIndex) { (void)motorIndex; }
    virtual int32_t getEncoder(size_t motorIndex) const { (void)motorIndex; return 0; }
    virtual uint32_t getStepsPerRevolution(size_t motorIndex) const { (void)motorIndex; return 0; }

    virtual int32_t getMotorRPM(size_t motorIndex) const { (void)motorIndex; return 0; }
    virtual float getMotorFrequencyHz(size_t motorIndex) const { (void)motorIndex; return 0; }
    //bool canAccuratelyEstimateSpeed(size_t motorIndex) const;
    //float getSpeed(size_t motorIndex) const;

    virtual void rpmFilterSetFrequencyHzIterationStep() {};
    virtual RPM_Filters* getRPM_Filters() { return nullptr; }
    virtual const RPM_Filters* getRPM_Filters() const { return nullptr; }

    virtual const DynamicIdleController* getDynamicIdleController() const { return nullptr; }
    virtual void setDynamicIdlerControllerConfig(const DynamicIdleController::config_t& config) { (void)config; }
protected:
    const type_e _type;
    const size_t _motorCount;
    const size_t _servoCount;
    Debug& _debug;
    float _motorOutputMin {0.0F}; //!< minimum motor output, typically set to 5.5% to avoid ESC desynchronization, may be set to zero if using dynamic idle control
    motorConfig_t _motorConfig { .motorIdle = 550, .maxthrottle = 2000, .mincommand = 1000, .kv = 1960, .motorPoleCount = 14 };
    bool _motorsIsOn {false};
    bool _motorsIsDisabled {false};
};
