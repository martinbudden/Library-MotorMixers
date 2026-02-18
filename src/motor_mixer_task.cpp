#include "MotorMixerBase.h"
#include "motor_mixer_message_queue.h"
#include "motor_mixer_task.h"

#include <TimeMicroseconds.h>

#if defined(FRAMEWORK_USE_FREERTOS)
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#else
#if defined(FRAMEWORK_ARDUINO_STM32)
#include <STM32FreeRTOS.h>
#endif
#include <FreeRTOS.h>
#include <task.h>
#endif
#endif


MotorMixerTask::MotorMixerTask(const motor_mixer_task_parameters_t& parameters) :
    _task(parameters)
{
}

/*!
Task function for the VehicleController.
*/
[[noreturn]] void MotorMixerTask::task()
{
#if defined(FRAMEWORK_USE_FREERTOS)
    motor_mixer_message_queue_item_t queue_item {};
    while (true) {
        _task.motor_mixer_message_queue.WAIT(queue_item);

        // calculate timings for instrumentation
        const TickType_t tick_count = xTaskGetTickCount();
        _tickCountDelta = tick_count - _tickCountPrevious;
        _tickCountPrevious = tick_count;

        const float delta_t = static_cast<float>(_tickCountDelta) * 0.001F;
        _task.motor_mixer.output_to_motors(queue_item, _task.rpm_filters, delta_t, tick_count);
    }
#else
    while (true) {}
#endif // FRAMEWORK_USE_FREERTOS
}

/*!
Wrapper function for VehicleController::Task with the correct signature to be used in xTaskCreate.
*/
[[noreturn]] void MotorMixerTask::task_static(void* arg)
{
    const TaskBase::parameters_t* parameters = static_cast<TaskBase::parameters_t*>(arg);

    static_cast<MotorMixerTask*>(parameters->task)->task(); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast)
}
