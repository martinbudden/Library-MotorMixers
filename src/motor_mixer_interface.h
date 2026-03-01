#pragma once

#include <cstdint>

class Debug;
class RpmFilters;
struct motor_mixer_message_queue_item_t;

struct motor_mixer_commands_t {
    float throttle;
    float roll;
    float pitch;
    float yaw;
};

class MotorMixerInterface {
public:
    virtual ~MotorMixerInterface() = default;
    /*!
    Called by the scheduler when the updateOutputsUsingPIDs function running in the AHRS task SIGNALs that output data is available.
    It is typically called at frequency of between 1000Hz and 8000Hz, so it has to be FAST.
    */
    virtual void output_to_motors(const motor_mixer_message_queue_item_t& queue_item, RpmFilters* rpm_filters, float delta_t, uint32_t tick_count, Debug& debug) {
        (void)queue_item; (void)rpm_filters; (void)delta_t; (void)tick_count; (void)debug;
    }

};
