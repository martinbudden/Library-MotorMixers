#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#if defined(FRAMEWORK_RPI_PICO)
#include "hardware/pio.h"
#elif defined(FRAMEWORK_ESPIDF)
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO
#endif // FRAMEWORK


#if !defined(F_CPU)
#define F_CPU 150000000L // CPU frequency
#endif

class EscDshot {
public:
    enum protocol_e {
        ESC_PROTOCOL_DSHOT150,
        ESC_PROTOCOL_DSHOT300,
        ESC_PROTOCOL_DSHOT600,
        ESC_PROTOCOL_PROSHOT,
        ESC_PROTOCOL_W2818B, // able to drive W2818B (NeoPixel) for debug setups
        ESC_PROTOCOL_COUNT
    };
    enum { DEFAULT_MOTOR_POLE_COUNT = 14 };
public:
    EscDshot(protocol_e protocol, uint16_t motor_pole_count);
    explicit EscDshot(protocol_e protocol) : EscDshot(protocol, DEFAULT_MOTOR_POLE_COUNT) {}
    EscDshot() : EscDshot(ESC_PROTOCOL_DSHOT300, DEFAULT_MOTOR_POLE_COUNT) {}
    void init(uint16_t pin);
public:
    enum { DSHOT_BIT_COUNT = 16 };
    void set_protocol(protocol_e protocol);
    void set_motor_pole_count(uint16_t motor_pole_count) { _motor_pole_count = motor_pole_count; }
    void write(uint16_t value); // value should be in the DShot range [47,2047]
    bool read();

    int32_t get_motor_rpm() const { return 2 * _erpm / _motor_pole_count; } // eRPM = RPM * poles/2, /2 due to pole pairs, not poles
    float get_motor_hz() const { return static_cast<float>(_erpm) * _erpm_to_hz; }
    void end();
    uint32_t nano_seconds_to_cycles(uint32_t nanoSeconds) const;
// for testing
    void set_use_high_order_bits(bool use_high_order_bits) { _use_high_order_bits = use_high_order_bits; }
    uint32_t get_data_high_pulse_width() const { return _data_high_pulse_width; }
    uint32_t get_data_low_pulse_width() const { return _data_low_pulse_width; }
    uint32_t getBufferItem(size_t index) const { return _dma_buffer[index]; }
protected:
    uint32_t _cpu_frequency {150000000};
    protocol_e _protocol;
    uint32_t _use_high_order_bits = 0;
    uint32_t _wrap_cycle_count {};
#if defined(FRAMEWORK_RPI_PICO)
#if defined(LIBRARY_MOTOR_MIXERS_USE_DSHOT_RPI_PICO_PIO)
    PIO _pio {};
    uint _pioStateMachine {};
    uint _pioOffset {};
#else
    enum { START_IMMEDIATELY = true, DONT_START_YET = false };
    uint32_t _dmaChannel {};
#endif // LIBRARY_MOTOR_MIXERS_USE_DSHOT_RPI_PICO_PIO
#endif
    uint32_t _data_high_pulse_width {};
    uint32_t _data_low_pulse_width {};
    enum { PIN_NOT_SET = 0xFFFF };
    uint16_t _pin {PIN_NOT_SET};
    uint16_t _motor_pole_count {DEFAULT_MOTOR_POLE_COUNT}; //!< number of poles the motor has, used to calculate RPM from telemetry data
    float _erpm_to_hz {};
    int32_t _erpm {}; //!< eRPM, ie not taking into account motor pole count
    uint32_t _telemetry_read_count {};
    uint32_t _telemetry_error_count {};

    enum { DMA_BUFFER_SIZE = DSHOT_BIT_COUNT + 1 }; // extra 1 for terminating zero value
    std::array<uint32_t, DMA_BUFFER_SIZE> _dma_buffer {};
};
