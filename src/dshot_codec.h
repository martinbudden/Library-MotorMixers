#pragma once

#include <array>
#include <cstdint>


/*!
DShot Encoder/Decoder

DShot Frame Structure
The DShot Frame defines which information is at which position in the data stream:

    11 bit throttle(S): 2048 possible values.
        0 is reserved for disarmed.
        1 to 47 are reserved for special commands.
        48 to 2047 (2000 steps) are for the actual throttle value
    1 bit telemetry request(T) - if this is set, telemetry data is sent back via a separate channel
    4 bit checksum(C) aka CRC (Cyclic Redundancy Check) to validate the frame

This results in a 16 bit (2 byte) frame with the following structure:

   SSSSSSSSSSSTCCCC


eRPM Telemetry Frame Structure

The eRPM telemetry frame sent by the ESC in bidirectional DSHOT mode is a 16 bit value, in the format:
The encoding of the eRPM data is not as straight forward as the one of the throttle frame:

    eeemmmmmmmmmcccc

where m is the 9-bit mantissa and e is the 3 bit exponent and cccc the checksum.
The resultant value is the mantissa shifted left by the exponent.
*/
class DshotCodec {
public:
    static constexpr uint16_t TELEMETRY_TYPE_ERPM           = 0; // Electrical RPM
    static constexpr uint16_t TELEMETRY_TYPE_TEMPERATURE    = 1; // Temperature Celsius
    static constexpr uint16_t TELEMETRY_TYPE_VOLTAGE        = 2; // Voltage with a step size of 0.25V ie [0, 0.25 ..., 63.75]
    static constexpr uint16_t TELEMETRY_TYPE_CURRENT        = 3; // Current with a step size of 1A ie [0, 1, ..., 255]
    static constexpr uint16_t TELEMETRY_TYPE_DEBUG1         = 4;
    static constexpr uint16_t TELEMETRY_TYPE_DEBUG2         = 5;
    static constexpr uint16_t TELEMETRY_TYPE_STRESS_LEVEL   = 6;
    static constexpr uint16_t TELEMETRY_TYPE_STATE_EVENTS   = 7;
    static constexpr uint16_t TELEMETRY_TYPE_COUNT          = 8;

    static constexpr uint16_t TELEMETRY_INVALID             = 0xFFFF;
public:
    static inline uint16_t pwm_to_dshot(uint16_t value) { return static_cast<uint16_t>(((value - 1000) * 2) + 47); }
    static inline uint16_t pwm_to_dshot_clamped(uint16_t value) { return value > 2000 ? pwm_to_dshot(2000) : value > 1000 ? pwm_to_dshot(value) : 0; }

    // non-inverted Checksum for unidirectional DShot
    static inline uint16_t checksum_unidirectional(uint16_t value) { return (value ^ (value >> 4) ^ (value >> 8)) & 0x0F; }
    static inline bool checksum_unidirectional_is_ok(uint16_t value) { return checksum_unidirectional(static_cast<uint16_t>(value>>4)) == (value & 0x0F); }
    // inverted Checksum for bidirectional DShot
    static inline uint16_t checksum_bidirectional(uint16_t value) { return(~(value ^ (value >> 4) ^ (value >> 8))) & 0x0F; }
    static inline bool checksum_bidirectional_is_ok(uint16_t value) { return checksum_bidirectional(static_cast<uint16_t>(value>>4)) == (value & 0x0F); }

    static inline uint16_t frame_unidirectional(uint16_t value) {
        value = static_cast<uint16_t>(value << 1);
        return static_cast<uint16_t>(value << 4) | checksum_unidirectional(value);
    }
    static inline uint16_t frame_bidirectional(uint16_t value) {
        value = static_cast<uint16_t>(value << 1);
        return static_cast<uint16_t>(value << 4) | checksum_bidirectional(value);
    }

    static uint32_t decode_erpm(uint16_t value);
    static uint32_t decode_telemetry_frame(uint16_t value, uint16_t& telemetry_type);
    static uint32_t decode_samples(uint64_t value, uint16_t& telemetry_type);
    static uint32_t decode_samples(const uint32_t* samples, uint32_t count, uint16_t& telemetry_type);

    // see [DSHOT - the missing Handbook](https://brushlesswhoop.com/dshot-and-bidirectional-dshot/)
    // for a good description of these conversions
    static uint32_t erpm_to_gcr20(uint16_t value);
    static uint32_t gr20_to_gcr21(uint32_t value);
    static inline uint32_t gcr21_to_gcr20(uint32_t value) { return (value ^ (value >> 1U)); }
    static uint16_t gcr20_to_erpm(uint32_t value);
public:
    static constexpr std::array<uint32_t, 17> GCR_BIT_LENGTHS = {
        0, // 0 consecutive bits, not a valid lookup
        1, // 1 consecutive bit
        1, // 2 consecutive bits
        1, // 3 consecutive bits
        2, // 4 consecutive bits
        2, // 5 consecutive bits
        2, // 6 consecutive bits
        3, // 7 consecutive bits
        3, // 8 consecutive bits
        3, // 9 consecutive bits
        3, //10 consecutive bits
        4, //11 consecutive bits, not valid, but sometimes occurs at the end of the string
        4, //12 consecutive bits
        4, //13 consecutive bits
        5, //14 consecutive bits
        5, //15 consecutive bits
        5, //16 consecutive bits
        // more than 10 consecutive samples: means four 0s or 1s in a row, which is invalid in GCR
    };
    static constexpr std::array<uint32_t, 6> GCR_SET_BITS = {
        0b00000, // 0 consecutive bits, not a valid lookup
        0b00001, // 1 consecutive bit
        0b00011, // 2 consecutive bits
        0b00111, // 3 consecutive bits
        0b01111, // 4 consecutive bits
        0b11111  // 5 consecutive bits
    };
    // array to map 5-bit GCR quintet to 4-bit nibble
    static constexpr std::array<uint32_t, 32> QUINTET_TO_NIBBLE = {
        0, 0,  0,  0, 0,  0,  0,  0,
        0, 9, 10, 11, 0, 13, 14, 15,
        0, 0,  2,  3, 0,  5,  6,  7,
        0, 0,  8,  1, 0,  4, 12,  0
    };
    // array to map 4-bit nibble to 5-bit GCR quintet
    static constexpr std::array<uint8_t, 16> NIBBLE_TO_QUINTET = {
        0x19, 0x1B, 0x12, 0x13, 0x1D, 0x15, 0x16, 0x17,
        0x1A, 0x09, 0x0A, 0x0B, 0x1E, 0x0D, 0x0E, 0x0F
    };
};
