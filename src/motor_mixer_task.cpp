#include "motor_mixer_interface.h"
#include "motor_mixer_message_queue.h"
#include "motor_mixer_task.h"

#include <time_microseconds.h>

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


MotorMixerTask::MotorMixerTask(MotorMixerInterface& motor_mixer, const motor_mixer_context_t& context) :
    _motor_mixer(motor_mixer),
    _context(context)
{
}

/*!
Task function for the VehicleController.
*/
[[noreturn]] void MotorMixerTask::task()
{
#if defined(FRAMEWORK_USE_FREERTOS)
    motor_commands_t motor_commands {};
    while (true) {
        _context.motor_mixer_message_queue.WAIT(motor_commands);

        // calculate timings for instrumentation
        const TickType_t tick_count = xTaskGetTickCount();
        _tick_count_delta = tick_count - _tick_count_previous;
        _tick_count_previous = tick_count;

        const float delta_t = static_cast<float>(_tick_count_delta) * 0.001F;
        _motor_mixer.output_to_motors(motor_commands, _context.rpm_filters, delta_t, tick_count, _context.debug);
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
