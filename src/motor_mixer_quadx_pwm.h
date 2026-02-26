#pragma once

#include <motor_mixer_quad_base.h>

#if defined(FRAMEWORK_STM32_CUBE)

#if defined(FRAMEWORK_STM32_CUBE_F1)
#include <stm32f1xx_hal.h>
#include <stm32f1xx_hal_tim.h>
#elif defined(FRAMEWORK_STM32_CUBE_F3)
#include <stm32f3xx_hal.h>
#include <stm32f3xx_hal_tim.h>
#elif defined(FRAMEWORK_STM32_CUBE_F4)
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_tim.h>
#elif defined(FRAMEWORK_STM32_CUBE_F7)
#include <stm32f7xx_hal.h>
#include <stm32f7xx_hal_tim.h>
#endif

#endif


class MotorMixerQuadXPwm : public MotorMixerQuadBase {
public:
    MotorMixerQuadXPwm(const motor_pins_t& pins, uint8_t output_to_motors_denominator);
    MotorMixerQuadXPwm(const stm32_motor_pins_t& pins, uint8_t output_to_motors_denominator);
public:
#if defined(FRAMEWORK_STM32_CUBE)
    struct pwm_pin_t {
        TIM_HandleTypeDef* htim;
        uint8_t pin;
        uint8_t channel;
    };
#else
    struct pwm_pin_t {
        uint8_t pin;
    };
#endif
public:
    virtual void output_to_motors(const motor_mixer_message_queue_item_t& queue_item, RpmFilters* rpm_filters, float delta_t, uint32_t tick_count, Debug& debug) override;
    void write_motor(uint8_t motor_index, float motor_output);
protected:
    float _pwm_scale {255.0F};
    std::array<pwm_pin_t, MOTOR_COUNT> _pins {};
#if defined(FRAMEWORK_STM32_CUBE)
    std::array<TIM_HandleTypeDef, MOTOR_COUNT> _htims {};
#endif
};
