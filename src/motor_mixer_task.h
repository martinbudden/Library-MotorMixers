#pragma once

#include <task_base.h>

class Debug;
class MotorMixerMessageQueue;
class MotorMixerInterface;
class RpmFilters;


struct motor_mixer_context_t {
    MotorMixerMessageQueue& motor_mixer_message_queue;
    MotorMixerInterface& motor_mixer;
    RpmFilters* rpm_filters;
    Debug& debug;
};

class MotorMixerTask : public TaskBase {
public:
    explicit MotorMixerTask(const motor_mixer_context_t& context);
public:
    static MotorMixerTask* create_task(task_info_t& task_info, const motor_mixer_context_t& context, uint8_t priority, uint32_t core);
    static MotorMixerTask* create_task(const motor_mixer_context_t& context, uint8_t priority, uint32_t core);
public:
    [[noreturn]] static void task_static(void* arg);
private:
    [[noreturn]] void task();
private:
    motor_mixer_context_t _context;
};
