#pragma once

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


struct motor_mixer_message_queue_item_t {
    float throttle;
    float roll_dps;
    float pitch_dps;
    float yaw_dps;
};

class MotorMixerMessageQueue {
public:
#if defined(FRAMEWORK_USE_FREERTOS)
    MotorMixerMessageQueue()
        : _queue_handle(xQueueCreateStatic(QUEUE_LENGTH, sizeof(motor_mixer_message_queue_item_t), &_queue_storage_area[0], &_queue_static))
    {}
    inline int32_t WAIT(motor_mixer_message_queue_item_t& queue_item) { return xQueueReceive(_queue_handle, &queue_item, portMAX_DELAY); }
    inline void SIGNAL(const motor_mixer_message_queue_item_t& queue_item) { xQueueOverwrite(_queue_handle, &queue_item); }
private:
    enum { QUEUE_LENGTH = 1 };
    std::array<uint8_t, QUEUE_LENGTH * sizeof(motor_mixer_message_queue_item_t)> _queue_storage_area {};
    StaticQueue_t _queue_static {};
    QueueHandle_t _queue_handle {};
#else
    MotorMixerMessageQueue() = default;
    inline int32_t WAIT(motor_mixer_message_queue_item_t& queue_item) { queue_item = _queue_item; return true; }
    inline void SIGNAL(const motor_mixer_message_queue_item_t& queue_item) { _queue_item = queue_item; }
    inline const motor_mixer_message_queue_item_t& getQueueItem() const { return _queue_item; }
private:
    motor_mixer_message_queue_item_t _queue_item {};
#endif // FRAMEWORK_USE_FREERTOS
};
