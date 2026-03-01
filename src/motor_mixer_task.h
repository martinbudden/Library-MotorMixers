#pragma once

#include <task_base.h>

class Debug;
class MotorMixerInterface;
class MotorMixerMessageQueue;
class RpmFilters;


struct motor_mixer_context_t {
    MotorMixerMessageQueue& motor_mixer_message_queue;
    RpmFilters* rpm_filters;
    Debug& debug;
};

class MotorMixerTask : public TaskBase {
public:
    MotorMixerTask(MotorMixerInterface& motor_mixer, const motor_mixer_context_t& context);
public:
    static MotorMixerTask* create_task(task_info_t& task_info, MotorMixerInterface& motor_mixer, const motor_mixer_context_t& context, uint8_t priority, uint32_t core);
    static MotorMixerTask* create_task(MotorMixerInterface& motor_mixer, const motor_mixer_context_t& context, uint8_t priority, uint32_t core);
public:
    [[noreturn]] static void task_static(void* arg);
private:
    [[noreturn]] void task();
private:
    MotorMixerInterface& _motor_mixer;
    motor_mixer_context_t _context;
};
