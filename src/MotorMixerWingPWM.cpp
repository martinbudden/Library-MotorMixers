#include "Mixers.h"
#include "MotorMixerWingPWM.h"
#include <cmath>

#if defined(FRAMEWORK_RPI_PICO)
#include <hardware/gpio.h>
#include <hardware/pwm.h>
#elif defined(FRAMEWORK_ESPIDF)
#include <driver/ledc.h>
#elif defined(FRAMEWORK_STM32_CUBE)
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO
#include <Arduino.h>
#if defined(FRAMEWORK_ARDUINO_ESP32)
#include <esp32-hal-ledc.h>
#endif
#endif // FRAMEWORK


MotorMixerWingPWM::MotorMixerWingPWM(Debug& debug, const stm32_motor_pins_t& pins) :
    MotorMixerWingBase(debug)
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

MotorMixerWingPWM::MotorMixerWingPWM(Debug& debug, const motor_pins_t& pins) :
    MotorMixerWingBase(debug)
#if !defined(FRAMEWORK_STM32_CUBE)
    ,_pins({pins.m0,pins.s0,pins.s1})
#endif
{
#if defined(FRAMEWORK_RPI_PICO)

    _pwmScale = 65535.0F; // NOLINT(cppcoreguidelines-prefer-member-initializer)
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

#elif defined(FRAMEWORK_STM32_CUBE)

    (void)pins;

#elif defined(FRAMEWORK_TEST)

#else // defaults to FRAMEWORK_ARDUINO
#if defined(FRAMEWORK_ARDUINO_ESP32)
    // Motor PWM Frequency
    static constexpr int frequency = 150000;
    // PWM Resolution
    static constexpr int resolution = 8;
#if defined(ESPRESSIF32_6_11_0)
    if (pins.m0 != 0xFF) {
        ledcSetup(M0, frequency, resolution);
        ledcAttachPin(pins.m0, M0);
    }
    if (pins.s0 != 0xFF) {
        ledcSetup(S0, frequency, resolution);
        ledcAttachPin(pins.s0, S0);
    }
    if (pins.s1 != 0xFF) {
        ledcSetup(S1, frequency, resolution);
        ledcAttachPin(pins.s1, S1);
    }
#else
    if (pins.m0 != 0xFF) {
        ledcAttach(pins.m0, frequency, resolution);
    }
    if (pins.s0 != 0xFF) {
        ledcAttach(pins.s0, frequency, resolution);
    }
    if (pins.s1 != 0xFF) {
        ledcAttach(pins.s1, frequency, resolution);
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

void MotorMixerWingPWM::writeMotor(uint8_t motorIndex, float motorOutput) // NOLINT(readability-make-member-function-const_
{
    const pwm_pin_t& pin = _pins[motorIndex];
    // scale motor output to GPIO range (normally [0,255] or [0, 65535])
    const auto output = static_cast<uint16_t>(roundf(_pwmScale*std::clamp(motorOutput, 0.0F, 1.0F)));
#if defined(FRAMEWORK_RPI_PICO)
    if (pin.pin != 0xFF) {
        pwm_set_gpio_level(pin.pin, output);
    }
#elif defined(FRAMEWORK_ESPIDF)
    (void)pin;
    (void)output;
#elif defined(FRAMEWORK_STM32_CUBE)
    if (pin.pin != 0xFF) {
        __HAL_TIM_SET_COMPARE(pin.htim, pin.channel, output);
    }
#elif defined(FRAMEWORK_TEST)
    (void)pin;
    (void)output;
#else // defaults to FRAMEWORK_ARDUINO
#if defined(FRAMEWORK_ARDUINO_ESP32)
    if (pin.pin != 0xFF) {
#if defined(ESPRESSIF32_6_11_0)
        ledcWrite(motorIndex, output);
#else
        ledcWrite(pin.pin, output);
#endif
    }
#else
    if (pin.pin != 0xFF) {
        analogWrite(pin.pin, output);
    }
#endif
#endif // FRAMEWORK
}

/*!
Calculate and output motor mix.
*/
void MotorMixerWingPWM::outputToMotors(commands_t& commands, float deltaT, uint32_t tickCount)
{
    (void)deltaT;
    (void)tickCount;

    if (motorsIsOn()) {
        // set the throttle to value returned by the mixer
        commands.throttle = mixWing(_outputs, commands, _mixParameters);
    } else {
        _outputs = { 0.0F, 0.0F, 0.0F };
    }

    writeMotor(M0, _outputs[M0]);
    writeMotor(S0, _outputs[S0]);
    writeMotor(S1, _outputs[S1]);
}
