#include "DShotCodec.h"
#include "EscDshot.h"

#if defined(FRAMEWORK_RPI_PICO)

#include "pio/dshot_bidir_300.pio.h"
#include "pio/dshot_bidir_600.pio.h"

#include <hardware/dma.h>
#include <hardware/gpio.h>
#include <hardware/pwm.h>

#elif defined(FRAMEWORK_ESPIDF)
#elif defined(FRAMEWORK_STM32_CUBE)
#elif defined(FRAMEWORK_TEST)

#else // defaults to FRAMEWORK_ARDUINO

#if !defined(FRAMEWORK_TEST)
#include <Arduino.h>
#endif

#endif // FRAMEWORK

/*
See:
https://betaflight.com/docs/wiki/guides/current/DSHOT-RPM-Filtering
https://brushlesswhoop.com/dshot-and-bidirectional-dshot/

https://github.com/josephduchesne/pico-dshot-bidir
https://github.com/bastian2001/pico-bidir-dshot
https://github.com/symonb/Bidirectional-DSHOT-and-RPM-Filter?tab=readme-ov-file

For ESP32:
https://github.com/wjxway/ESP32-Bidirectional-DShot/blob/main/src/MotorCtrl.cpp (requires 2 wires)

*/

EscDshot::EscDshot(protocol_e protocol, uint16_t motor_pole_count) :
    _protocol(protocol),
    _motor_pole_count(motor_pole_count)
{
    set_protocol(protocol);
    static constexpr float SECONDS_PER_MINUTE = 60.0F;
    _erpm_to_hz = 2.0F * (100.0F / SECONDS_PER_MINUTE) / static_cast<float>(_motor_pole_count);
}

void EscDshot::init(uint16_t pin)
{
    _pin = pin;

#if defined(FRAMEWORK_RPI_PICO)
#if defined(LIBRARY_MOTOR_MIXERS_USE_DSHOT_RPI_PICO_PIO)
    if (_protocol == ESC_PROTOCOL_DSHOT300) {
        const bool success = pio_claim_free_sm_and_add_program_for_gpio_range(&dshot_bidir_300_program, &_pio, &_pioStateMachine, &_pioOffset, pin, 1, true);
        hard_assert(success);
        // Configure it to run our program, and start it, using the
        // helper function we included in our .pio file.
        //printf("Using gpio %d\n", pin);
        dshot_bidir_300_program_init(_pio, _pioStateMachine, _pioOffset, pin, _cpu_frequency);
    } else {
        const bool success = pio_claim_free_sm_and_add_program_for_gpio_range(&dshot_bidir_600_program, &_pio, &_pioStateMachine, &_pioOffset, pin, 1, true);
        hard_assert(success);
        // Configure it to run our program, and start it, using the
        // helper function we included in our .pio file.
        //printf("Using gpio %d\n", pin);
        dshot_bidir_600_program_init(_pio, _pioStateMachine, _pioOffset, pin, _cpu_frequency);
    }
    // The PIO State Machine is now running, to use we push onto its TX FIFO and pull from the RX FIFO
#else
    gpio_set_function(pin, GPIO_FUNC_PWM); // Set the pin to be PWM

    // RP2040 has 8 slices, RP2350 has 12 slices
    // Each slice can drive or measure 2 PWM signals, ie has 2 channels, PWM_CHAN_A and PWM_CHAN_B
    enum { PWM_CHANNEL_A = 0, PWM_CHANNEL_B =  1};

    // get PWM channel for the pin
    const uint32_t pwmChannel = pwm_gpio_to_channel(_pin);
    // channel B uses high order bits on RPI Pico
    _use_high_order_bits = pwmChannel == PWM_CHANNEL_B ? true : false;

    // Setup the PWM
    const uint32_t slice = pwm_gpio_to_slice_num(_pin);
    pwm_set_wrap(slice, _wrap_cycle_count);
    pwm_set_enabled(slice, true); // start the PWM

    // Setup the DMA
    enum { PANIC_IF_NONE_AVAILABLE = true };
    _dmaChannel = dma_claim_unused_channel(PANIC_IF_NONE_AVAILABLE);

    dma_channel_config dmaConfig = dma_channel_get_default_config(_dmaChannel); // NOLINT(cppcoreguidelines-init-variables) false positive
    channel_config_set_dreq(&dmaConfig, pwm_get_dreq(slice)); // Set the DMA Data Request (DREQ)
    // transfer 32 bits at a time
    // don't increment write address so we always transfer to the same PWM register.
    // increment read address so we pick up a new value each time
    channel_config_set_transfer_data_size(&dmaConfig, DMA_SIZE_32);
    channel_config_set_write_increment(&dmaConfig, false);
    channel_config_set_read_increment(&dmaConfig, true);

    dma_channel_configure(
        _dmaChannel,
        &dmaConfig,
        &pwm_hw->slice[slice].cc, // write to PWM counter compare
        nullptr, // inital read address, set when DMA is started
        0, // transfer count, set when DMA is started
        DONT_START_YET
    );
#endif // LIBRARY_MOTOR_MIXERS_USE_DSHOT_RPI_PICO_PIO
#elif defined(FRAMEWORK_ESPIDF)
#elif defined(FRAMEWORK_STM32_CUBE)
#elif defined(FRAMEWORK_TEST)

#else // defaults to FRAMEWORK_ARDUINO
#if !defined(UNIT_TEST_BUILD)
    pinMode(static_cast<uint8_t>(pin), OUTPUT);
    digitalWrite(static_cast<uint8_t>(pin), LOW);
#endif

#endif // FRAMEWORK
}

/*!
Sets the DShot protocol.
Calculate pulse widths for that protocol.

Called in construction, before init().
*/
void EscDshot::set_protocol(protocol_e protocol)
{
/*
    The DShot protocol is based on WS2812B (NeoPixel) protocol.

    For comparison with W2812B
    W2812B https://cdn-shop.adafruit.com/datasheets/WS2812B.pdf
    T0 is the width of the pulse, T1 is the width of gap to the next pulse
    T0H = 400ns +/- 150ns
    T1H = 800ns +/- 150ns
    T0L = 850ns +/- 150ns
    T1L = 450ns +/- 150ns
    TxH+TxL = 1250ns +/- 600ns (T0H + T0L or T1H + T1L)
    enum { W2818B_T0H = 400, W2818B_T1H = 800, W2818B_T = 1250 };

    DShot150 means 150 kilobytes/second
    DShot 150 specification is
    T0H = 2500ns (data low pulse width)
    T0L = 4180ns (data low gap width)
    T1H = 5000ns (data high pulse width)
    T1L = 1680ns (data high gap width)
    TxH+TxL = 6680ns  (T0H + T0L or T1H + T1L)

    DShot 300 specification is
    T0H = 1250ns (data low pulse width)
    T0L = 2090ns (data low gap width)
    T1H = 2500ns (data high pulse width)
    T1L =  840ns (data high gap width)
    TxH+TxL = 3340ns  (T0H + T0L or T1H + T1L)

    see https://blck.mn/2016/11/dshot-the-new-kid-on-the-block/
    DShot 600 specification is
    T0H =  625ns (data low pulse width)
    T0L = 1045ns (data low gap width)
    T1H = 1250ns (data high pulse width)
    T0L =  420ns (data hig gap width)
    TxH+TxL = 1670ns  (T0H + T0L or T1H + T1L)
*/

    _protocol = protocol;
    // DShot 150 specification
    enum { DSHOT150_T0H = 2500, DSHOT150_T1H = 5000, DSHOT150_T = 6680 };
    enum { W2818B_T0H = 400, W2818B_T1H = 800, W2818B_T = 1250 };

    // _data_low_pulse_width and _data_high_pulse_width are in processor cycles
    // for RPI_PICO: default CPU frequency is 150MHz, that is 0.15GHz

    _data_low_pulse_width = nano_seconds_to_cycles(DSHOT150_T0H);  // =  375 = 2500 * 0.15GHz
    _data_high_pulse_width = nano_seconds_to_cycles(DSHOT150_T1H); // =  750 = 5000 * 0.15GHz
    _wrap_cycle_count = nano_seconds_to_cycles(DSHOT150_T);       // = 1002 = 6680 * 0.15GHz

    switch (protocol) {
    case ESC_PROTOCOL_DSHOT150:
        break;
    case ESC_PROTOCOL_DSHOT300:
        _data_low_pulse_width /= 2;
        _data_high_pulse_width /= 2;
        _wrap_cycle_count /= 2;
        break;
    case ESC_PROTOCOL_DSHOT600:
        [[fallthrough]];
    case ESC_PROTOCOL_PROSHOT:
        _data_low_pulse_width /= 4;
        _data_high_pulse_width /= 4;
        _wrap_cycle_count /= 4;
        break;
    case ESC_PROTOCOL_W2818B:
        _data_low_pulse_width = nano_seconds_to_cycles(W2818B_T0H);  // =  60 =  400 * 0.15GHz
        _data_high_pulse_width = nano_seconds_to_cycles(W2818B_T1H); // = 120 =  800 * 0.15GHz
        _wrap_cycle_count = nano_seconds_to_cycles(W2818B_T);       // = 188 = 1250 * 0.15GHz
        break;
    default:
        break;
    }

#if defined(FRAMEWORK_RPI_PICO) && !defined(LIBRARY_MOTOR_MIXERS_USE_DSHOT_RPI_PICO_PIO)
    if (_pin != PIN_NOT_SET) {
        // the pin has already been set, so we need to re-set the wrap value
        const uint32_t slice = pwm_gpio_to_slice_num(_pin);
        pwm_set_enabled(slice, false); // stop the PWM
        pwm_set_wrap(slice, _wrap_cycle_count);
        pwm_set_enabled(slice, true); // restart the PWM
    }
#endif
}

uint32_t EscDshot::nano_seconds_to_cycles(uint32_t nanoSeconds) const
{
    // note: the k values cancel out, but give greater precision in the calculation
    const uint64_t k = 128;
    const uint64_t d = k * 1000000000L  / _cpu_frequency;
    const uint64_t ret = nanoSeconds * k / d;
    return static_cast<uint32_t>(ret);
}

/*!
value should be in the DShot range [47,2047]

Unidirectional DShot can use the hardware PWM generators and DMA.

For bidirectional DShot we need to wait for the response from the DMA and it seems bit-banging is required.
See [ESC BDShot protocol implementation](https://symonb.github.io/docs/drone/ESC/ESC_prot_impl_2_2#esc-bdshot-protocol-implementation)
for description and STM32 implementation.

On Raspberry Pi Pico we can use the Programmable IO (PIO) for this bit-banging.
*/
void EscDshot::write(uint16_t value) // NOLINT(readability-make-member-function-const)
{
#if defined(LIBRARY_MOTOR_MIXERS_USE_DSHOT_RPI_PICO_PIO)
    // use the value to create a bidirectional DShot frame and send it to the PIO state machine
    value = DshotCodec::frame_bidirectional(value);
    pio_sm_put(_pio, _pioStateMachine, value);
#else
    // set up a unidirectional DShot frame for sending via DMA
    const uint16_t frame = DshotCodec::frame_unidirectional(value);

    uint16_t maskBit = 1U << (DSHOT_BIT_COUNT - 1);
    if (_use_high_order_bits) {
        for (auto& item : _dma_buffer) {
            item = ((frame & maskBit) ? _data_high_pulse_width : _data_low_pulse_width) << 16;
            maskBit = static_cast<uint16_t>(maskBit >> 1);
        }
    } else {
        for (auto& item : _dma_buffer) {
            item = (frame & maskBit) ? _data_high_pulse_width : _data_low_pulse_width;
            maskBit = static_cast<uint16_t>(maskBit >> 1);
        }
    }
    _dma_buffer[DMA_BUFFER_SIZE - 1] = 0; // zero last value,  array size is DSHOT_BIT_COUNT + 1

#if defined(FRAMEWORK_RPI_PICO)
    // transfer DMA buffer to PWM
    dma_channel_set_trans_count(_dmaChannel, DMA_BUFFER_SIZE, DONT_START_YET);
    dma_channel_set_read_addr(_dmaChannel, &_dma_buffer[0], START_IMMEDIATELY);
#elif defined(FRAMEWORK_ESPIDF)
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO
#endif // FRAMEWORK

#endif // LIBRARY_MOTOR_MIXERS_USE_DSHOT_RPI_PICO_PIO
}

bool EscDshot::read()
{
    uint16_t telemetry_type {};
    uint32_t value {};

#if defined(LIBRARY_MOTOR_MIXERS_USE_DSHOT_RPI_PICO_PIO)
    const int32_t fifoCount = pio_sm_get_rx_fifo_level(_pio, _pioStateMachine);
    if (fifoCount >= 2) {
        // get DShot telemetry value from the PIO
        //value = pio_sm_get(_pio, _pioStateMachine);
        uint64_t samples = static_cast<uint64_t>(pio_sm_get_blocking(_pio, _pioStateMachine)) << 32;
        samples |= static_cast<uint64_t>(pio_sm_get_blocking(_pio, _pioStateMachine));
        value = DshotCodec::decode_samples(samples, telemetry_type);
    } else {
        return false;
    }
#else
    telemetry_type = DshotCodec::TELEMETRY_INVALID;
#endif

    ++_telemetry_read_count;
    switch (telemetry_type) {
    case DshotCodec::TELEMETRY_INVALID:
        ++_telemetry_error_count;
        return false;
    case DshotCodec::TELEMETRY_TYPE_ERPM: {
        // Convert to eRPM * 100
        //return ((1000000 * 60 / 100) + value / 2) / value;
        enum { ONE_MINUTE_IN_MICROSECONDS = 60000000 };
        // value is eRPM period in microseconds
        _erpm = static_cast<int32_t>(ONE_MINUTE_IN_MICROSECONDS / value);
        break;
    }
    case DshotCodec::TELEMETRY_TYPE_TEMPERATURE:
        [[fallthrough]];
    case DshotCodec::TELEMETRY_TYPE_VOLTAGE:
        [[fallthrough]];
    case DshotCodec::TELEMETRY_TYPE_CURRENT:
        [[fallthrough]];
    case DshotCodec::TELEMETRY_TYPE_DEBUG1:
        [[fallthrough]];
    case DshotCodec::TELEMETRY_TYPE_DEBUG2:
        [[fallthrough]];
    case DshotCodec::TELEMETRY_TYPE_STRESS_LEVEL:
        [[fallthrough]];
    case DshotCodec::TELEMETRY_TYPE_STATE_EVENTS:
        [[fallthrough]];
    default:
        break;
    }
    return true;
}

void EscDshot::end()
{
#if defined(FRAMEWORK_RPI_PICO)
#if defined(LIBRARY_MOTOR_MIXERS_USE_DSHOT_RPI_PICO_PIO)
    if (_protocol == ESC_PROTOCOL_DSHOT300) {
        pio_remove_program_and_unclaim_sm(&dshot_bidir_300_program, _pio, _pioStateMachine, _pioOffset);
    } else {
        pio_remove_program_and_unclaim_sm(&dshot_bidir_600_program, _pio, _pioStateMachine, _pioOffset);
    }
#endif
#elif defined(FRAMEWORK_ESPIDF)
    ESP_ERROR_CHECK(rmt_disable(_txChannel));
#elif defined(FRAMEWORK_STM32_CUBE)
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO
#if !defined(UNIT_TEST)
    digitalWrite(static_cast<uint8_t>(_pin), LOW);
#endif
#endif // FRAMEWORK
}
