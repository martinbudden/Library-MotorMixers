#include "motor_mixer_task.h"

#include <array>
#include <cassert>
#include <cstring>

#if defined(FRAMEWORK_USE_FREERTOS)
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
#include <freertos/FreeRTOS.h>
#include <freertos/FreeRTOSConfig.h>
#include <freertos/task.h>
#else
#if defined(FRAMEWORK_ARDUINO_STM32)
#include <STM32FreeRTOS.h>
#endif
#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <task.h>
#endif
#endif


MotorMixerTask* MotorMixerTask::create_task(MotorMixerMessageQueue& motor_mixer_message_queue, MotorMixerBase& motor_mixer, RpmFilters* rpm_filters, uint8_t priority, uint32_t core)
{
    task_info_t task_info {};
    return create_task(task_info, motor_mixer_message_queue, motor_mixer, rpm_filters, priority, core);
}

MotorMixerTask* MotorMixerTask::create_task(task_info_t& task_info, MotorMixerMessageQueue& motor_mixer_message_queue, MotorMixerBase& motor_mixer, RpmFilters* rpm_filters, uint8_t priority, uint32_t core)
{
    static MotorMixerTask motor_mixer_task(motor_mixer_message_queue, motor_mixer, rpm_filters);

    static TaskBase::parameters_t task_parameters { // NOLINT(misc-const-correctness) false positive
        .task = &motor_mixer_task
    };
#if !defined(VEHICLE_CONTROLLER_TASK_STACK_DEPTH_BYTES)
    enum { VEHICLE_CONTROLLER_TASK_STACK_DEPTH_BYTES = 4096 };
#endif
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32) || !defined(FRAMEWORK_USE_FREERTOS)
    static std::array<uint8_t, VEHICLE_CONTROLLER_TASK_STACK_DEPTH_BYTES> stack;
#else
    static std::array <StackType_t, VEHICLE_CONTROLLER_TASK_STACK_DEPTH_BYTES / sizeof(StackType_t)> stack;
#endif
    task_info = {
        .taskHandle = nullptr,
        .name = "MotorMixerTask", // max length 16, including zero terminator
        .stackDepthBytes = VEHICLE_CONTROLLER_TASK_STACK_DEPTH_BYTES,
        .stackBuffer = reinterpret_cast<uint8_t*>(&stack[0]), // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
        .priority = priority,
        .core = core,
        .taskIntervalMicroseconds = 0,
    };

#if defined(FRAMEWORK_USE_FREERTOS)
    assert(std::strlen(task_info.name) < configMAX_TASK_NAME_LEN);
    assert(task_info.priority < configMAX_PRIORITIES);

    static StaticTask_t taskBuffer;
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
    task_info.taskHandle = xTaskCreateStaticPinnedToCore(
        MotorMixerTask::task_static,
        task_info.name,
        task_info.stackDepthBytes / sizeof(StackType_t),
        &task_parameters,
        task_info.priority,
        &stack[0],
        &taskBuffer,
        task_info.core
    );
    assert(task_info.taskHandle != nullptr && "Unable to create MotorMixerTask");
#elif defined(FRAMEWORK_RPI_PICO) || defined(FRAMEWORK_ARDUINO_RPI_PICO)
    task_info.taskHandle = xTaskCreateStaticAffinitySet(
        MotorMixerTask::task_static,
        task_info.name,
        task_info.stackDepthBytes / sizeof(StackType_t),
        &task_parameters,
        task_info.priority,
        &stack[0],
        &taskBuffer,
        task_info.core
    );
    assert(task_info.taskHandle != nullptr && "Unable to create MotorMixerTask");
#else
    task_info.taskHandle = xTaskCreateStatic(
        MotorMixerTask::task_static,
        task_info.name,
        task_info.stackDepthBytes / sizeof(StackType_t),
        &task_parameters,
        task_info.priority,
        &stack[0],
        &taskBuffer
    );
    assert(task_info.taskHandle != nullptr && "Unable to create MotorMixerTask");
    // vTaskCoreAffinitySet(task_info.taskHandle, task_info.core);
#endif
#else
    (void)task_parameters;
#endif // FRAMEWORK_USE_FREERTOS
    return &motor_mixer_task;
}
