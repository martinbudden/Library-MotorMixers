#include "RpmFilters.h"
#include <fast_trigonometry.h>

#if defined(FRAMEWORK_USE_FREERTOS)

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

// vTaskSuspendAll suspends the scheduler. This prevents a context switch from occurring but leaves interrupts enabled.
inline void LOCK_FILTERS() { vTaskSuspendAll(); }
inline void UNLOCK_FILTERS() { xTaskResumeAll(); }

#else

inline void LOCK_FILTERS() {}
inline void UNLOCK_FILTERS() {}

#endif

#if (__cplusplus >= 202002L)
#include <ranges>
#endif


void RpmFilters::set_config(const config_t& config)
{
    _config = config;
    _Q = static_cast<float>(_config.rpm_filter_q) * 0.01F;

    _state.state = STATE_STOPPED;
    // just under  Nyquist frequency (ie just under half sampling rate)
    // for 8kHz loop this is 3840Hz
    _max_frequency_hz = 480000.0F / static_cast<float>(_looptime_seconds);
    _half_of_max_frequency_hz = _max_frequency_hz / 2.0F;
    _third_of_max_frequency_hz = _max_frequency_hz / 3.0F;
    _min_frequency_hz = _config.rpm_filter_min_hz;
    _fadeRangeHz = _config.rpm_filter_fade_range_hz;

    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-constant-array-index)
#if (__cplusplus >= 202002L)
    for (auto harmonic : std::views::iota(size_t{0}, _config.rpm_filter_harmonics)) {
        for (auto motor : std::views::iota(size_t{0}, _motor_count)) {
#else
    for (size_t harmonic = 0; harmonic < _config.rpm_filter_harmonics; ++harmonic) {
        for (size_t motor = 0; motor < _motor_count; ++motor) {
#endif
            _filters[motor][harmonic].initNotch(_min_frequency_hz * static_cast<float>(harmonic + 1), _looptime_seconds, _Q);
        }
    }
    // NOLINTEND(cppcoreguidelines-pro-bounds-constant-array-index)

    if (config.rpm_filter_lpf_hz == 0) {
        for (auto& rpmFilter : _motor_rpm_filters) {
            rpmFilter.setToPassthrough();
        }
    } else {
        for (auto& rpmFilter : _motor_rpm_filters) {
            rpmFilter.setCutoffFrequencyAndReset(config.rpm_filter_lpf_hz, _looptime_seconds);
        }
    }
}

/*!
This is called from MotorMixer::output_to_motors and so needs to be FAST.
*/
void RpmFilters::set_frequency_hz_iteration_start(size_t motor_index, float frequency_hz)
{
    if (_config.rpm_filter_lpf_hz == 0) {
        return;
    }
    if (_state.state == STATE_STOPPED) {
        _state.state = STATE_FUNDAMENTAL;
        _state.motor_index = 0;
    }
    motor_state_t& motorState = _state.motorStates[motor_index]; // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)

    frequency_hz = _motor_rpm_filters[motor_index].filter(frequency_hz);
    motorState.frequency_hz_unclamped = frequency_hz;
    frequency_hz = std::clamp(frequency_hz, _min_frequency_hz, _max_frequency_hz);

    const float margin_frequency_hz = frequency_hz - _min_frequency_hz;
    motorState.weight_multiplier = (margin_frequency_hz < _fadeRangeHz) ? margin_frequency_hz / _fadeRangeHz : 1.0F;

    const BiquadFilterT<xyz_t>& rpmFilter = _filters[motor_index][FUNDAMENTAL]; // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
    motorState.omega = rpmFilter.calculateOmega(frequency_hz);
}

/*!
This is called from MotorMixer::rpm_filter_set_frequency_hz_iteration_step and so needs to be FAST.
*/
void RpmFilters::set_frequency_hz_iteration_step() // NOLINT(readability-function-cognitive-complexity)
{
    // state machine sets notch filter for one harmonic of one motor on each iteration.

    switch (_state.state) {
    case STATE_STOPPED:
        return;
    case STATE_FUNDAMENTAL: {
        BiquadFilterT<xyz_t>& rpmFilter = _filters[_state.motor_index][FUNDAMENTAL]; // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
        // omega = frequency * _2PiLoopTimeSeconds
        // maxFrequency < 0.5 / looptime_seconds
        // maxOmega = (0.5 / looptime_seconds) * 2PiLooptimeSeconds = 0.5 * 2PI = PI;
        // so omega is in range [0, PI]
        motor_state_t& motorState = _state.motorStates[_state.motor_index]; // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
        FastTrigonometry::sin_cos(motorState.omega, motorState.sin_omega, motorState.two_cosOmega);
        motorState.two_cosOmega *= 2.0F;
        LOCK_FILTERS();
        rpmFilter.setNotchFrequencyWeighted(motorState.sin_omega, motorState.two_cosOmega, _weights[FUNDAMENTAL]*motorState.weight_multiplier);
        UNLOCK_FILTERS();
        ++_state.motor_index;
        if (_state.motor_index == _motor_count) {
            // we have set the notch frequency for all motors, so move onto the next harmonic if there is one, otherwise we are finished
            _state.motor_index = 0;
            if (_config.rpm_filter_harmonics >= 2) {
                if (_config.rpm_filter_weights[SECOND_HARMONIC] != 0) {
                    _state.state = STATE_SECOND_HARMONIC;
                } else if (_config.rpm_filter_harmonics >= 3 && _config.rpm_filter_weights[THIRD_HARMONIC] != 0) {
                    _state.state = STATE_THIRD_HARMONIC;
                } else {
                    _state.state = STATE_STOPPED;
                }
            } else {
                _state.state = STATE_STOPPED;
            }
        }
        break;
    }
    case STATE_SECOND_HARMONIC: {
        const motor_state_t& motorState = _state.motorStates[_state.motor_index]; // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
        if (motorState.frequency_hz_unclamped > _half_of_max_frequency_hz) { // ie 2.0F * frequency_hz_unclamped > _max_frequency_hz
            // no point filtering the second harmonic if it is above the Nyquist frequency
            _weights[SECOND_HARMONIC] = 0.0F;
        } else {
            _weights[SECOND_HARMONIC] = _config.rpm_filter_weights[SECOND_HARMONIC] * 0.01F;
            BiquadFilterT<xyz_t>& rpmFilter = _filters[_state.motor_index][SECOND_HARMONIC]; // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
            // sin(2θ) = 2 * sin(θ) * cos(θ)
            // cos(2θ) = 2 * cos^2(θ) - 1
            const float sin_2Omega = motorState.sin_omega * motorState.two_cosOmega;
            const float two_cos_2Omega = motorState.two_cosOmega * motorState.two_cosOmega - 2.0F;
            LOCK_FILTERS();
            rpmFilter.setNotchFrequencyWeighted(sin_2Omega, two_cos_2Omega, _weights[SECOND_HARMONIC]*motorState.weight_multiplier);
            UNLOCK_FILTERS();
        }
        ++_state.motor_index;
        if (_state.motor_index == _motor_count) {
            // we have set the notch frequency for all motors, so move onto the next harmonic if there is one, otherwise we are finished
            _state.motor_index = 0;
            if (_config.rpm_filter_harmonics >= 3 && _config.rpm_filter_weights[THIRD_HARMONIC] != 0) {
                _state.state = STATE_THIRD_HARMONIC;
            } else {
                _state.state = STATE_STOPPED;
            }
        }
        break;
    }
    case STATE_THIRD_HARMONIC: {
        const motor_state_t& motorState = _state.motorStates[_state.motor_index]; // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
        if (motorState.frequency_hz_unclamped > _third_of_max_frequency_hz) { // ie 3.0F * frequency_hz_unclamped > _max_frequency_hz
            // no point filtering the third harmonic if it is above the Nyquist frequency
            _weights[THIRD_HARMONIC] = 0.0F;
        } else {
            _weights[THIRD_HARMONIC] = _config.rpm_filter_weights[THIRD_HARMONIC] * 0.01F;
            BiquadFilterT<xyz_t>& rpmFilter = _filters[_state.motor_index][THIRD_HARMONIC]; // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
            // sin(3θ) = 3 * sin(θ)   - 4 * sin^3(θ)
            //         = sin(θ) * ( 3 - 4 * sin^2(θ) )
            //         = sin(θ) * ( 3 - 4 * (1 - cos^2(θ)) )
            //         = sin(θ) * ( 4 * cos^2(θ) - 1)
            // cos(3θ) = 4 * cos^3(θ) - 3 * cos(θ)
            //         = cos(θ) * ( 4 * cos^2(θ) - 3 )
            const float four_cosSquaredOmega = motorState.two_cosOmega * motorState.two_cosOmega;
            const float sin_3Omega = motorState.sin_omega * (four_cosSquaredOmega - 1.0F);
            const float two_cos_3Omega = motorState.two_cosOmega * (four_cosSquaredOmega - 3.0F);
            LOCK_FILTERS();
            rpmFilter.setNotchFrequencyWeighted(sin_3Omega, two_cos_3Omega, _weights[THIRD_HARMONIC]*motorState.weight_multiplier);
            UNLOCK_FILTERS();
        }
        ++_state.motor_index;
        if (_state.motor_index == _motor_count) {
            // we have set the notch frequency for all motors, so we are finished
            _state.motor_index = 0;
            _state.state = STATE_STOPPED;
        }
        break;
    }
    } // END SWITCH
}

void RpmFilters::filter(xyz_t& input, size_t motor_index) // NOLINT(readability-make-member-function-const) false positive
{
    input = _filters[motor_index][FUNDAMENTAL].filterWeighted(input); // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)

    if (_weights[SECOND_HARMONIC] != 0.0F) {
        input = _filters[motor_index][SECOND_HARMONIC].filterWeighted(input); // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
    };
    if (_weights[THIRD_HARMONIC] != 0.0F) {
        input = _filters[motor_index][THIRD_HARMONIC].filterWeighted(input); // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
    };
}
