#pragma once

#include <array>
#include <motor_mixer_base.h>


class MotorMixerWingBase : public MotorMixerBase {
public:
    explicit MotorMixerWingBase(uint8_t output_to_motors_denominator, Debug* debug) : 
        MotorMixerBase(FLYING_WING_SINGLE_PROPELLER, output_to_motors_denominator, MOTOR_COUNT, SERVO_COUNT, debug) {}
public:
    enum { M0=0, S0=1, S1=2, MOTOR_COUNT=1, SERVO_COUNT=2 };
    virtual float get_motor_output(size_t motor_index) const override { return _outputs[motor_index]; } // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
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
