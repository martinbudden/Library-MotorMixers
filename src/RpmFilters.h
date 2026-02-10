#pragma once

#include <FilterTemplates.h>
#include <Filters.h>

#include <xyz_type.h>


/*!
There are up to 3 filters for each motor, one for filtering at the fundamental frequency
and up to two others, for filtering at the second harmonic and the third harmonic.

The `setFilter` computations are time-critical and are divided up into chucks and driven by a state machine, to
reduce the time taken by each invocation of the function.

Generally speaking, the SECOND HARMONIC is used for 2-bladed propellors, and the THIRD HARMONIC is used
for 3-bladed propellors.
*/
class RpmFilters {
public:
    RpmFilters(size_t motor_count, float looptime_seconds) : _motor_count(motor_count), _looptime_seconds(looptime_seconds) {}
public:
    static const uint8_t FUNDAMENTAL = 0;
    static const uint8_t SECOND_HARMONIC = 1;
    static const uint8_t THIRD_HARMONIC = 2;
    static const uint8_t RPM_FILTER_HARMONICS_COUNT = 3;

    struct config_t {
        uint16_t rpm_filter_fade_range_hz;  // range in which notch filters fade down to minHz
        uint16_t rpm_filter_q;              // Q of the notch filters
        uint16_t rpm_filter_lpf_hz;         // LPF cutoff (from motor rpm converted to Hz)
        uint8_t  rpm_filter_weights[RPM_FILTER_HARMONICS_COUNT];    // weight as a percentage for each harmonic
        uint8_t  rpm_filter_harmonics;      // number of harmonics, zero means filters off
        uint8_t  rpm_filter_min_hz;         // minimum notch frequency for fundamental harmonic
    };
public:
#if defined(LIBRARY_MOTOR_MIXERS_MAX_MOTOR_COUNT_EIGHT)
    enum { MAX_MOTOR_COUNT = 8 };
#else
    enum { MAX_MOTOR_COUNT = 4 };
#endif
public:
    void set_config(const config_t& config);
    const config_t& get_config() const { return _config; }
    void set_frequency_hz_iteration_start(size_t motor_index, float frequency_hz); // called from the motor mixer
    void set_frequency_hz_iteration_step(); // called from the motor mixer
    void filter(xyz_t& input, size_t motor_index);
    bool isActive() const { return _config.rpm_filter_harmonics > 0; }
    size_t get_motor_count() const { return _motor_count; }
private:
    size_t _motor_count;
    float _looptime_seconds;
    // computation data so set_frequency_hz_iteration_start() can be run as a state machine
    enum state_e { STATE_STOPPED, STATE_FUNDAMENTAL, STATE_SECOND_HARMONIC, STATE_THIRD_HARMONIC };
    struct motor_state_t {
        float frequency_hz_unclamped;
        float weight_multiplier;
        float omega;
        float sin_omega;
        float two_cosOmega;
    };
    struct state_t {
        state_e state;
        size_t motor_index;
        std::array<motor_state_t, MAX_MOTOR_COUNT> motorStates;
    };
    state_t _state {};

    std::array<float, RPM_FILTER_HARMONICS_COUNT> _weights {};
    float _min_frequency_hz { 100.0F };
    float _max_frequency_hz {};
    float _half_of_max_frequency_hz {};
    float _third_of_max_frequency_hz {};
    float _fadeRangeHz { 50.0F };
    float _Q { 0.0F };
    BiquadFilterT<xyz_t> _filters[MAX_MOTOR_COUNT][RPM_FILTER_HARMONICS_COUNT]; //!< note this is a template filter that filters all 3 axes
    std::array<PowerTransferFilter1, MAX_MOTOR_COUNT> _motor_rpm_filters {}; //!< filters the motor RPM before it is used to set the filter frequency
    config_t _config {}; //!< configuration data is only changed in set_config
};
