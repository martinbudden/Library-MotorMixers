#pragma once

#include <array>
#include <motor_mixer_base.h>


class MotorMixerQuadBase : public MotorMixerBase {
public:
    enum { M0=0, M1=1, M2=2, M3=3, MOTOR_COUNT=4, SERVO_COUNT = 0 };

    explicit MotorMixerQuadBase(uint8_t type, uint8_t output_to_motors_denominator) : 
        MotorMixerBase(type, output_to_motors_denominator, MOTOR_COUNT, SERVO_COUNT) {}
    virtual float get_motor_output(size_t motor_index) const override { return _outputs[motor_index]; } // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
public:
    struct motor_pins_t {
        uint8_t m0;
        uint8_t m1;
        uint8_t m2;
        uint8_t m3;
    };
    struct stm32_motor_pins_t {
        stm32_motor_pin_t m0;
        stm32_motor_pin_t m1;
        stm32_motor_pin_t m2;
        stm32_motor_pin_t m3;
    };
protected:
    std::array<float, MOTOR_COUNT> _outputs {};
};
