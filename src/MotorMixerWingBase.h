#pragma once

#include <MotorMixerBase.h>
#include <array>


class MotorMixerWingBase : public MotorMixerBase {
public:
    enum { M0=0, S0=1, S1=2, MOTOR_COUNT=1, SERVO_COUNT=2 };
    explicit MotorMixerWingBase(Debug& debug) : MotorMixerBase(FLYING_WING_SINGLE_PROPELLER, MOTOR_COUNT, SERVO_COUNT, debug) {}
    virtual float getMotorOutput(size_t motorIndex) const override { return _outputs[motorIndex]; } // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
public:
    struct motor_pins_t {
        uint8_t m0;
        uint8_t s0;
        uint8_t s1;
    };
    struct stm32_motor_pins_t {
        stm32_motor_pin_t m0;
        stm32_motor_pin_t s0;
        stm32_motor_pin_t s1;
    };
protected:
    std::array<float, MOTOR_COUNT + SERVO_COUNT> _outputs {};
};
