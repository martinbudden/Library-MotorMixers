#pragma once

#include "motor_commands.h"

#include <array>
#include <cstdint>

#if defined(FRAMEWORK_USE_FREERTOS)
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#else
#if defined(FRAMEWORK_ARDUINO_STM32)
#include <STM32FreeRTOS.h>
#endif
#include <FreeRTOS.h>
#include <queue.h>
#endif
#endif


class MotorMixerMessageQueue {
public:
#if defined(FRAMEWORK_USE_FREERTOS)
    MotorMixerMessageQueue()
        : _queue_handle(xQueueCreateStatic(QUEUE_LENGTH, sizeof(motor_commands_t), &_queue_storage_area[0], &_queue_static))
    {}
    inline int32_t WAIT(motor_commands_t& motor_commands) { return xQueueReceive(_queue_handle, &motor_commands, portMAX_DELAY); }
    inline void SIGNAL(const motor_commands_t& motor_commands) { xQueueOverwrite(_queue_handle, &motor_commands); }
private:
    enum { QUEUE_LENGTH = 1 };
    std::array<uint8_t, QUEUE_LENGTH * sizeof(motor_commands_t)> _queue_storage_area {};
    StaticQueue_t _queue_static {};
    QueueHandle_t _queue_handle {};
#else
    MotorMixerMessageQueue() = default;
    inline int32_t WAIT(motor_commands_t& motor_commands) { motor_commands = _motor_commands; return true; }
    inline void SIGNAL(const motor_commands_t& motor_commands) { _motor_commands = motor_commands; }
    inline const motor_commands_t& getQueueItem() const { return _motor_commands; }
private:
    motor_commands_t _motor_commands {};
#endif // FRAMEWORK_USE_FREERTOS
};
