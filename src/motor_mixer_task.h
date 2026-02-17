#pragma once

#include <TaskBase.h>

class MotorMixerMessageQueue;
class MotorMixerBase;
class RpmFilters;


class MotorMixerTask : public TaskBase {
public:
    MotorMixerTask(MotorMixerMessageQueue& motor_mixer_message_queue, MotorMixerBase& motor_mixer, RpmFilters* rpm_filters);
public:
    static MotorMixerTask* create_task(task_info_t& task_info, MotorMixerMessageQueue& motor_mixer_message_queue, MotorMixerBase& motor_mixer, RpmFilters* rpm_filters, uint8_t priority, uint32_t core);
    static MotorMixerTask* create_task(MotorMixerMessageQueue& motor_mixer_message_queue, MotorMixerBase& motor_mixer, RpmFilters* rpm_filters, uint8_t priority, uint32_t core);
public:
    [[noreturn]] static void task_static(void* arg);
private:
    [[noreturn]] void task();
private:
    MotorMixerMessageQueue& _motor_mixer_message_queue;
    MotorMixerBase& _motor_mixer;
    RpmFilters* _rpm_filters;
};
