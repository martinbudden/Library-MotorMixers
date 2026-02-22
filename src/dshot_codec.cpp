#include "DshotCodec.h"

// NOLINTBEGIN(hicpp-signed-bitwise)
uint32_t DshotCodec::decode_erpm(uint16_t value)
{
    // eRPM range
    if (value == 0x0FFF) {
        return 0;
    }
    const uint16_t m = value & 0x01FF;
    const uint16_t e = (value & 0xFE00) >> 9;
    value = static_cast<uint16_t>(m << e);
    if (value == 0) {
        return TELEMETRY_INVALID;
    }
    return value;
}

uint32_t DshotCodec::decode_telemetry_frame(uint16_t value, uint16_t& telemetry_type)
{
    // value is of form "eeem mmmm mmmm": e - exponent, m - mantissa
    // https://github.com/bird-sanctuary/extended-dshot-telemetry

    // eRPM frames are of form "0000 mmmm mmmm" or "eee1 mmmm mmmm"
    const uint16_t type = (value & 0x0F00) >> 8;
    const bool isErpm = (type & 0x01) || (type == 0);
    if (isErpm) {
        telemetry_type = TELEMETRY_TYPE_ERPM;
        const uint16_t m = value & 0x01FF;
        const uint16_t e = (value & 0xFE00) >> 9;
        value = static_cast<uint16_t>(m << e);
        if (value == 0) {
            return TELEMETRY_INVALID;
        }
        return value;
    }
    // Extended DShot Telemetry (EDT) frame is of the form:
    // ppp0mmmmmmmm
    // where ppp is the telemetry type and mmmmmmmm is the value
    telemetry_type = type >> 1;
    return value & 0x00FF;
}

/*!
Decode samples returned by Raspberry Pi PIO implementation.

Returns the value of the Extended DShot Telemetry (EDT) frame (without the  checksum).
*/
uint32_t DshotCodec::decode_samples(uint64_t value, uint16_t& telemetry_type)
{
    // telemetry data must start with a 0, so if the first bit is high, we don't have any data
    if (value & 0x8000000000000000L) {
        telemetry_type = TELEMETRY_INVALID;
        return 0;
    }

    uint32_t consecutiveBitCount = 1;  // we always start with the MSB
    uint32_t currentBit = 0;
    uint32_t bitCount = 0;
    uint32_t gcr_result = 0;
    // starting at 2nd bit since we know our data starts with a 0
    // 56 samples @ 0.917us sample rate = 51.33us sampled
    // loop the mask from 2nd MSB to  LSB
    for (uint64_t mask = 0x4000000000000000; mask != 0; mask >>= 1) {
        if (((value & mask) != 0) != currentBit) {
            // if the masked bit doesn't match the current string of bits then end the current string and flip currentBit
            // bitshift gcr_result by N, and
            gcr_result = gcr_result << GCR_BIT_LENGTHS[consecutiveBitCount]; // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
            // then set N bits in gcr_result, if currentBit is 1
            if (currentBit) {
                gcr_result |= GCR_SET_BITS[GCR_BIT_LENGTHS[consecutiveBitCount]]; // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
            }
            bitCount += GCR_BIT_LENGTHS[consecutiveBitCount]; // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
            // invert currentBit, and reset consecutiveBitCount
            currentBit = !currentBit;
            consecutiveBitCount = 1;  // first bit found in the string is the one we just processed
        } else {
            // otherwise increment consecutiveBitCount
            ++consecutiveBitCount;
            if (consecutiveBitCount > 16) {
                // invalid run length at the current sample rate (outside of GCR_BIT_LENGTHS table)
                telemetry_type = TELEMETRY_INVALID;
                return 0;
            }
        }
    }

    // outside the loop, we still need to account for the final bits if the string ends with 1s
    // bitshift gcr_result by N, and
    gcr_result <<= GCR_BIT_LENGTHS[consecutiveBitCount]; // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
    // then set set N bits in gcr_result, if currentBit is 1
    if (currentBit) {
        gcr_result |= GCR_SET_BITS[GCR_BIT_LENGTHS[consecutiveBitCount]]; // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
    }
    // count bitCount (for debugging)
    bitCount += GCR_BIT_LENGTHS[consecutiveBitCount]; // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)

    // GCR data should be 21 bits
    if (bitCount < 21) {
        telemetry_type = TELEMETRY_INVALID;
        return 0;
    }

    // chop the GCR data down to just the 21 most significant bits
    gcr_result = gcr_result >> (bitCount - 21);

    // convert 21-bit edge transition GCR to 20-bit binary GCR
    const uint32_t gcr20 = gcr21_to_gcr20(gcr_result);

    const uint16_t result = gcr20_to_erpm(gcr20);

    if (!checksum_bidirectional_is_ok(result)) {
        telemetry_type = TELEMETRY_INVALID;
        return 0;
    }
    return decode_telemetry_frame(static_cast<uint16_t>(result >> 4), telemetry_type);
}


/*!
Decode samples value returned by bit-banging.

Returns the value of the Extended DShot Telemetry (EDT) frame (without the  checksum).
*/
uint32_t DshotCodec::decode_samples(const uint32_t* samples, uint32_t count, uint16_t& telemetry_type) // NOLINT(cppcoreguidelines-avoid-c-arrays,hicpp-avoid-c-arrays,modernize-avoid-c-arrays)
{
    // decode 16 bit GCR (Group Coded Recording) Run Length Limited (RLL) encoding
    // https://en.wikipedia.org/wiki/Run-length_limited#GCR:_(0,2)_RLL

    // see also https://github.com/bird-sanctuary/arduino-bi-directional-dshot/blob/master/src/arduino_dshot.cpp

    // a 16-bit value is encoded as 21 bits in GCR:
    //   each 4-bit nibble is encoded as 5 bits, giving 20 bits
    //   this 20-bit value is then encoded into 21 bits
    //   see https://brushlesswhoop.com/dshot-and-bidirectional-dshot/ for an example

    uint32_t value = 0;
    uint32_t bitCount = 0;
    for (uint32_t ii = 1; ii < count; ++ii) {
        const uint32_t diff = samples[ii] - samples[ii-1]; // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        const uint32_t runLength = (diff + 8) / 16;
        value <<= runLength;
        value |= 1 << (runLength - 1);
        bitCount += runLength;
        if (bitCount >= 21) {
            break;
        }
    }
    if (bitCount < 21) {
        const uint32_t runLength = 21 - bitCount;
        value <<= runLength;
        value |= 1 << (runLength - 1);
        bitCount += runLength;
    }
    if (bitCount != 21) {
        return TELEMETRY_INVALID;
    }

    const uint16_t result = gcr20_to_erpm(value);

    if (!checksum_bidirectional_is_ok(result)) {
        telemetry_type = TELEMETRY_INVALID;
        return 0;
    }
    return decode_telemetry_frame(static_cast<uint16_t>(result >> 4), telemetry_type);
}

uint16_t DshotCodec::gcr20_to_erpm(uint32_t value)
{
    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-constant-array-index)
    uint32_t ret = QUINTET_TO_NIBBLE[value & 0x1F];
    ret |= QUINTET_TO_NIBBLE[(value >> 5) & 0x1F] << 4;
    ret |= QUINTET_TO_NIBBLE[(value >> 10) & 0x1F] << 8;
    ret |= QUINTET_TO_NIBBLE[(value >> 15) & 0x1F] << 12;
    return static_cast<uint16_t>(ret);
    // NOLINTEND(cppcoreguidelines-pro-bounds-constant-array-index)
}

uint32_t DshotCodec::erpm_to_gcr20(uint16_t value)
{
    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-constant-array-index)
    uint32_t ret = NIBBLE_TO_QUINTET[value & 0x1F];
    ret |= static_cast<uint32_t>(NIBBLE_TO_QUINTET[(value >> 4) & 0x1F]) << 5;
    ret |= static_cast<uint32_t>(NIBBLE_TO_QUINTET[(value >> 8) & 0x1F]) << 10;
    ret |= static_cast<uint32_t>(NIBBLE_TO_QUINTET[(value >> 12) & 0x1F]) << 15;
    return ret;
    // NOLINTEND(cppcoreguidelines-pro-bounds-constant-array-index)
}

uint32_t DshotCodec::gr20_to_gcr21(uint32_t value)
{
// Map the GCR to a 21 bit value, this new value starts with a 0 and the rest of the bits are set by the following two rules:
//    1. If the current input bit in GCR data is a 1 then the output bit is the inverse of the previous output bit
//    2. If the current input bit in GCR data is a 0 then the output bit is the same as the previous output

    uint32_t ret = 0;
    uint32_t previousOutputBit = 0;

    for (uint32_t mask = 1U << 20; mask != 0; mask >>= 1U) {
        ret <<= 1;
        const uint32_t inputBit = value & mask;
        const uint32_t outputBit = inputBit ? !previousOutputBit : previousOutputBit;
        previousOutputBit = outputBit;
        ret |= outputBit;
    }

    return ret;
}

// NOLINTEND(hicpp-signed-bitwise)
