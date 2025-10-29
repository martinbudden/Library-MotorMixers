#pragma once

#include <FilterTemplates.h>
#include <Filters.h>

#if defined(FRAMEWORK_RPI_PICO)

#include <pico/critical_section.h>
#include <pico/mutex.h>

//#elif defined(FRAMEWORK_USE_FREERTOS)
#elif defined(FRAMEWORK_USE_FREERTOS) && (defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32))

#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#else
#if defined(FRAMEWORK_ARDUINO_STM32)
#include <STM32FreeRTOS.h>
#endif
#include <FreeRTOS.h>
#include <queue.h>
#include <semphr.h>
#include <task.h>
#endif

#endif

#include <xyz_type.h>


/*!
There are up to 3 filters for each motor, one for filtering at the fundamental frequency
and up to two others, for filtering at the second harmonic and the third harmonic.

The `setFilter` computations are time-critical and are divided up into chucks and driven by a state machine, to
reduce the time taken by each invocation of the function.

Generally speaking, the SECOND HARMONIC is used for 2-bladed propellors, and the THIRD HARMONIC is used
for 3-bladed propellors.
*/
class RPM_Filters {
public:
    enum { RPM_FILTER_HARMONICS_COUNT = 3 };
    struct config_t {
        uint16_t rpm_filter_fade_range_hz;  // range in which notch filters fade down to minHz
        uint16_t rpm_filter_q;              // Q of the notch filters
        uint16_t rpm_filter_lpf_hz;         // LPF cutoff (from motor rpm converted to Hz)
        uint8_t  rpm_filter_weights[RPM_FILTER_HARMONICS_COUNT];    // weight as a percentage for each harmonic
        uint8_t  rpm_filter_harmonics;      // number of harmonics, zero means filters off
        uint8_t  rpm_filter_min_hz;         // minimum notch frequency for fundamental harmonic
    };
public:
    enum { FUNDAMENTAL = 0, SECOND_HARMONIC = 1, THIRD_HARMONIC = 2 };
#if defined(LIBRARY_MOTOR_MIXERS_MAX_MOTOR_COUNT_EIGHT)
    enum { MAX_MOTOR_COUNT = 8 };
#else
    enum { MAX_MOTOR_COUNT = 4 };
#endif
public:
    RPM_Filters(size_t motorCount, float looptimeSeconds) : _motorCount(motorCount), _looptimeSeconds(looptimeSeconds) {}
    void setConfig(const config_t& config);
    const config_t& getConfig() const { return _config; }
    void setFrequencyHzIterationStart(size_t motorIndex, float frequencyHz); // called from the motor mixer
    void setFrequencyHzIterationStep(); // called from the motor mixer
    void filter(xyz_t& input, size_t motorIndex);
    bool isActive() const { return _config.rpm_filter_harmonics > 0; }
    size_t getMotorCount() const { return _motorCount; }
public:
    static constexpr float M_PI_F = static_cast<float>(M_PI);
private:
    size_t _motorCount;
    float _looptimeSeconds;
    // computation data so setFrequencyHzIterationStart() can be run as a state machine
    enum state_e { STATE_STOPPED, STATE_FUNDAMENTAL, STATE_SECOND_HARMONIC, STATE_THIRD_HARMONIC };
    struct motor_state_t {
        float frequencyHzUnclipped;
        float weightMultiplier;
        float omega;
        float sinOmega;
        float two_cosOmega;
    };
    struct state_t {
        state_e state;
        size_t motorIndex;
        motor_state_t motorStates[MAX_MOTOR_COUNT];
    };
    state_t _state {};

    std::array<float, RPM_FILTER_HARMONICS_COUNT> _weights {};
    float _minFrequencyHz { 100.0F };
    float _maxFrequencyHz {};
    float _halfOfMaxFrequencyHz {};
    float _thirdOfMaxFrequencyHz {};
    float _fadeRangeHz { 50.0F };
    float _Q { 0.0F };
    BiquadFilterT<xyz_t> _filters[MAX_MOTOR_COUNT][RPM_FILTER_HARMONICS_COUNT]; //!< note this is a template filter that filters all 3 axes
    std::array<PowerTransferFilter1, MAX_MOTOR_COUNT> _motorRPM_Filters {}; //!< filters the motor RPM before it is used to set the filter frequency
    config_t _config {}; //!< configuration data is only changed in setConfig
#if defined(FRAMEWORK_RPI_PICO)
    mutable mutex_t _mutex {};
    inline void LOCK_FILTERS() const { mutex_enter_blocking(&_mutex); }
    inline void UNLOCK_FILTERS() const { mutex_exit(&_mutex); }
#elif defined(FRAMEWORK_USE_FREERTOS) && (defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32))
    // vTaskSuspendAll suspends the scheduler. This prevents a context switch from occurring but leaves interrupts enabled.
    inline void LOCK_FILTERS() const { vTaskSuspendAll(); }
    inline void UNLOCK_FILTERS() const { xTaskResumeAll(); }
#else
    inline void LOCK_FILTERS() const {}
    inline void UNLOCK_FILTERS() const {}
#endif
};
