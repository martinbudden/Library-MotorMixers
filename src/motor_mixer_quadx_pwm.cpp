#include "mixers.h"
#include "motor_mixer_message_queue.h"
#include "motor_mixer_quadx_pwm.h"

#include <cmath>

#if defined(FRAMEWORK_RPI_PICO)
#include <hardware/gpio.h>
#include <hardware/pwm.h>
#elif defined(FRAMEWORK_ESPIDF)
#include <driver/ledc.h>
#include <esp32-hal-ledc.h>
#elif defined(FRAMEWORK_STM32_CUBE)
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO
#if defined(FRAMEWORK_ARDUINO_ESP32)
#include <esp32-hal-ledc.h>
#else
#include <Arduino.h>
#endif
#endif // FRAMEWORK


/*
https://docs.espressif.com/projects/arduino-esp32/en/latest/migration_guides/2.x_to_3.0.html

Removed:
ledcSetup
ledcAttachPin

Added:
ledcAttach used to set up the LEDC pin (merged ledcSetup and ledcAttachPin functions).
*/


MotorMixerQuadXPwm::MotorMixerQuadXPwm(const stm32_motor_pins_t& pins, uint8_t output_to_motors_denominator) :
    MotorMixerQuadBase(QUAD_X, output_to_motors_denominator)
{
#if defined(FRAMEWORK_STM32_CUBE) && !defined(FRAMEWORK_ARDUINO_STM32)
    if (pins.m0.pin != 0xFF) {
        _pins[M0].htim = &_htims[M0];
        _pins[M0].channel = pins.m0.channel;
        _pins[M0].pin = pins.m0.pin;
        HAL_TIM_PWM_Start(_pins[M0].htim, _pins[M0].channel);
    }
    if (pins.m1.pin != 0xFF) {
        _pins[M1].htim = &_htims[M1];
        _pins[M1].channel = pins.m1.channel;
        _pins[M1].pin = pins.m1.pin;
        HAL_TIM_PWM_Start(_pins[M1].htim, _pins[M1].channel);
    }
    if (pins.m2.pin != 0xFF) {
        _pins[M2].htim = &_htims[M2];
        _pins[M2].channel = pins.m2.channel;
        _pins[M2].pin = pins.m2.pin;
        HAL_TIM_PWM_Start(_pins[M2].htim, _pins[M2].channel);
    }
    if (pins.m3.pin != 0xFF) {
        _pins[M3].htim = &_htims[M3];
        _pins[M3].channel = pins.m3.channel;
        _pins[M3].pin = pins.m3.pin;
        HAL_TIM_PWM_Start(_pins[M3].htim, _pins[M3].channel);
    }
#else
    (void)pins;
#endif
}

MotorMixerQuadXPwm::MotorMixerQuadXPwm(const motor_pins_t& pins, uint8_t output_to_motors_denominator) :
    MotorMixerQuadBase(QUAD_X, output_to_motors_denominator)
#if !defined(FRAMEWORK_STM32_CUBE)
    ,_pins({pins.m0,pins.m1,pins.m2,pins.m3})
#endif
{
#if defined(FRAMEWORK_RPI_PICO)

    _pwm_scale = 65535.0F; // NOLINT(cppcoreguidelines-prefer-member-initializer)
    if (pins.m0 != 0xFF) {
        gpio_set_function(pins.m0, GPIO_FUNC_PWM);
    }
    if (pins.m1 != 0xFF) {
        gpio_set_function(pins.m1, GPIO_FUNC_PWM);
    }
    if (pins.m2 != 0xFF) {
        gpio_set_function(pins.m2, GPIO_FUNC_PWM);
    }
    if (pins.m3 != 0xFF) {
        gpio_set_function(pins.m3, GPIO_FUNC_PWM);
    }

#elif defined(FRAMEWORK_ESPIDF)

    static constexpr int frequency_hz = 150000; // Motor PWM Frequency
    static constexpr int resolutionBits = 8; // PWM Resolution
    if (pins.m0 != 0xFF) {
        ledcAttach(pins.m0, frequency_hz, resolutionBits);
    }
    if (pins.m1 != 0xFF) {
        ledcAttach(pins.m1, frequency_hz, resolutionBits);
    }
    if (pins.m2 != 0xFF) {
        ledcAttach(pins.m2, frequency_hz, resolutionBits);
    }
    if (pins.m3 != 0xFF) {
        ledcAttach(pins.m3, frequency_hz, resolutionBits);
    }

#elif defined(FRAMEWORK_STM32_CUBE)

    (void)pins;

#elif defined(FRAMEWORK_TEST)

#else // defaults to FRAMEWORK_ARDUINO
#if defined(FRAMEWORK_ARDUINO_ESP32)

    static constexpr int frequency_hz = 150000; // Motor PWM Frequency
    static constexpr int resolutionBits = 8; // PWM Resolution
#if defined(FRAMEWORK_ARDUINO_ESP32_V2)
    if (pins.m0 != 0xFF) {
        ledcSetup(M0, frequency_hz, resolutionBits);
        ledcAttachPin(pins.m0, M0);
    }
    if (pins.m1 != 0xFF) {
        ledcSetup(M1, frequency_hz, resolutionBits);
        ledcAttachPin(pins.m1, M1);
    }
    if (pins.m2 != 0xFF) {
        ledcSetup(M2, frequency_hz, resolutionBits);
        ledcAttachPin(pins.m2, M2);
    }
    if (pins.m3 != 0xFF) {
        ledcSetup(M3, frequency_hz, resolutionBits);
        ledcAttachPin(pins.m3, M3);
    }
#else
    if (pins.m0 != 0xFF) {
        ledcAttach(pins.m0, frequency_hz, resolutionBits);
    }
    if (pins.m1 != 0xFF) {
        ledcAttach(pins.m1, frequency_hz, resolutionBits);
    }
    if (pins.m2 != 0xFF) {
        ledcAttach(pins.m2, frequency_hz, resolutionBits);
    }
    if (pins.m3 != 0xFF) {
        ledcAttach(pins.m3, frequency_hz, resolutionBits);
    }
#endif

#else // defaults to FRAMEWORK_ARDUINO

    if (pins.m0 != 0xFF) {
        pinMode(pins.m0, OUTPUT);
    }
    if (pins.m1 != 0xFF) {
        pinMode(pins.m1, OUTPUT);
    }
    if (pins.m2 != 0xFF) {
        pinMode(pins.m2, OUTPUT);
    }
    if (pins.m3 != 0xFF) {
        pinMode(pins.m3, OUTPUT);
    }

#endif

#endif // FRAMEWORK
}

void MotorMixerQuadXPwm::write_motor(uint8_t motor_index, float motor_output) // NOLINT(readability-make-member-function-const_
{
    const pwm_pin_t& pin = _pins[motor_index];
    if (pin.pin == 0xFF) {
        return;
    }
    // scale motor output to GPIO range (normally [0,255] or [0, 65535])
    const auto output = static_cast<uint16_t>(roundf(_pwm_scale*clamp(motor_output, 0.0F, 1.0F)));
#if defined(FRAMEWORK_RPI_PICO)
    pwm_set_gpio_level(pin.pin, output);
#elif defined(FRAMEWORK_ESPIDF)
    ledcWrite(pin.pin, output);
#elif defined(FRAMEWORK_STM32_CUBE)
    __HAL_TIM_SET_COMPARE(pin.htim, pin.channel, output);
#elif defined(FRAMEWORK_TEST)
    (void)output;
#else // defaults to FRAMEWORK_ARDUINO
#if defined(FRAMEWORK_ARDUINO_ESP32)
#if defined(FRAMEWORK_ARDUINO_ESP32_V2)
    ledcWrite(motor_index, output);
#else
    ledcWrite(pin.pin, output);
#endif
#else
    analogWrite(pin.pin, output);
#endif
#endif // FRAMEWORK
}

/*!
Calculate and output motor mix.
*/
void MotorMixerQuadXPwm::output_to_motors(const motor_commands_t& motor_commands, RpmFilters* rpm_filters, float delta_t, uint32_t tick_count, Debug& debug)
{
    (void)rpm_filters;
    (void)delta_t;
    (void)tick_count;
    (void)debug;

    // Output to motors every _output_to_motors_denominator times output_to_motors is called.
    ++_output_to_mixer_count;
    if (_output_to_mixer_count < _output_to_motors_denominator && motors_is_on()) {
        return;
    }

    _output_to_mixer_count = 0;
    if (motors_is_on()) {
        const motor_mixer_commands_t commands {
            .throttle  = motor_commands.throttle,
            // scale roll, pitch, and yaw from DPS range to [-1.0F, 1.0F]
            .roll   = motor_commands.roll_dps * MIXER_OUTPUT_SCALE_FACTOR,
            .pitch  = motor_commands.pitch_dps * MIXER_OUTPUT_SCALE_FACTOR,
            .yaw    = motor_commands.yaw_dps * MIXER_OUTPUT_SCALE_FACTOR
        };
        _outputs  = mix_quad_x(commands, _mix_parameters);
    } else {
        _outputs = { 0.0F, 0.0F, 0.0F, 0.0F };
    }

    write_motor(M0, _outputs[M0]);
    write_motor(M1, _outputs[M1]);
    write_motor(M2, _outputs[M2]);
    write_motor(M3, _outputs[M3]);
}
