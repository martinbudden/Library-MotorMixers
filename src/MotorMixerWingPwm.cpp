#include "Mixers.h"
#include "MotorMixerWingPwm.h"
#include "motor_mixer_message_queue.h"

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
#include <Arduino.h>
#if defined(FRAMEWORK_ARDUINO_ESP32)
#include <esp32-hal-ledc.h>
#endif
#endif // FRAMEWORK


MotorMixerWingPwm::MotorMixerWingPwm(const stm32_motor_pins_t& pins, uint8_t output_to_motors_denominator, Debug* debug) :
    MotorMixerWingBase(output_to_motors_denominator, debug)
{
#if defined(FRAMEWORK_STM32_CUBE) && !defined(FRAMEWORK_ARDUINO_STM32)
    if (pins.m0.pin != 0xFF) {
        _pins[M0].htim = &_htims[M0];
        _pins[M0].channel = pins.m0.channel;
        _pins[M0].pin = pins.m0.pin;
        HAL_TIM_PWM_Start(_pins[M0].htim, _pins[M0].channel);
    }
    if (pins.s0.pin != 0xFF) {
        _pins[S0].htim = &_htims[S0];
        _pins[S0].channel = pins.s0.channel;
        _pins[S0].pin = pins.s0.pin;
        HAL_TIM_PWM_Start(_pins[S0].htim, _pins[S0].channel);
    }
    if (pins.s1.pin != 0xFF) {
        _pins[S1].htim = &_htims[S1];
        _pins[S1].channel = pins.s1.channel;
        _pins[S1].pin = pins.s1.pin;
        HAL_TIM_PWM_Start(_pins[S1].htim, _pins[S1].channel);
    }
#else
    (void)pins;
#endif
}

MotorMixerWingPwm::MotorMixerWingPwm(const motor_pins_t& pins, uint8_t output_to_motors_denominator, Debug* debug) :
    MotorMixerWingBase(output_to_motors_denominator, debug)
#if !defined(FRAMEWORK_STM32_CUBE)
    ,_pins({pins.m0,pins.s0,pins.s1})
#endif
{
#if defined(FRAMEWORK_RPI_PICO)

    _pwm_scale = 65535.0F; // NOLINT(cppcoreguidelines-prefer-member-initializer)
    if (pins.m0 != 0xFF) {
        gpio_set_function(pins.m0, GPIO_FUNC_PWM);
    }
    if (pins.s0 != 0xFF) {
        gpio_set_function(pins.s0, GPIO_FUNC_PWM);
    }
    if (pins.s1 != 0xFF) {
        gpio_set_function(pins.s1, GPIO_FUNC_PWM);
    }

#elif defined(FRAMEWORK_ESPIDF)

    static constexpr int frequency_hz = 150000; // Motor PWM Frequency
    static constexpr int resolutionBits = 8; // PWM Resolution
    if (pins.m0 != 0xFF) {
        ledcAttach(pins.m0, frequency_hz, resolutionBits);
    }
    if (pins.s0 != 0xFF) {
        ledcAttach(pins.s0, frequency_hz, resolutionBits);
    }
    if (pins.s1 != 0xFF) {
        ledcAttach(pins.s1, frequency_hz, resolutionBits);
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
    if (pins.s0 != 0xFF) {
        ledcSetup(S0, frequency_hz, resolutionBits);
        ledcAttachPin(pins.s0, S0);
    }
    if (pins.s1 != 0xFF) {
        ledcSetup(S1, frequency_hz, resolutionBits);
        ledcAttachPin(pins.s1, S1);
    }
#else
    if (pins.m0 != 0xFF) {
        ledcAttach(pins.m0, frequency_hz, resolutionBits);
    }
    if (pins.s0 != 0xFF) {
        ledcAttach(pins.s0, frequency_hz, resolutionBits);
    }
    if (pins.s1 != 0xFF) {
        ledcAttach(pins.s1, frequency_hz, resolutionBits);
    }
#endif
#else // defaults to FRAMEWORK_ARDUINO
    if (pins.m0 != 0xFF) {
        pinMode(pins.m0, OUTPUT);
    }
    if (pins.s0 != 0xFF) {
        pinMode(pins.s0, OUTPUT);
    }
    if (pins.s1 != 0xFF) {
        pinMode(pins.s1, OUTPUT);
    }
#endif

#endif // FRAMEWORK
}

void MotorMixerWingPwm::write_motor(uint8_t motor_index, float motorOutput) // NOLINT(readability-make-member-function-const_
{
    const pwm_pin_t& pin = _pins[motor_index];
    if (pin.pin == 0xFF) {
        return;
    }
    // scale motor output to GPIO range (normally [0,255] or [0, 65535])
    const auto output = static_cast<uint16_t>(roundf(_pwm_scale*std::clamp(motorOutput, 0.0F, 1.0F)));
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
void MotorMixerWingPwm::output_to_motors(const motor_mixer_message_queue_item_t& queue_item, RpmFilters* rpm_filters, float delta_t, uint32_t tick_count)
{
    (void)rpm_filters;
    (void)delta_t;
    (void)tick_count;

    ++_output_to_mixer_count;
    if (_output_to_mixer_count < _output_to_motors_denominator && motors_is_on()) {
        return;
    }

    if (motors_is_on()) {
        // set the throttle to value returned by the mixer
        const motor_mixer_commands_t commands {
            .throttle  = queue_item.throttle,
            // scale roll, pitch, and yaw from DPS range to [-1.0F, 1.0F]
            .roll   = queue_item.roll_dps * MIXER_OUTPUT_SCALE_FACTOR,
            .pitch  = queue_item.pitch_dps * MIXER_OUTPUT_SCALE_FACTOR,
            .yaw    = queue_item.yaw_dps * MIXER_OUTPUT_SCALE_FACTOR
        };
        _throttle_command = mix_wing(_outputs, commands, _mix_parameters);
    } else {
        _outputs = { 0.0F, 0.0F, 0.0F };
    }

    write_motor(M0, _outputs[M0]);
    write_motor(S0, _outputs[S0]);
    write_motor(S1, _outputs[S1]);
}
