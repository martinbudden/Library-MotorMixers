#include <DshotCodec.h>
#include <unity.h>

void setUp()
{
}

void tearDown()
{
}

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,cppcoreguidelines-pro-bounds-constant-array-index,cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-signed-bitwise,readability-magic-numbers)
void test_dshot_quintets()
{
    static_assert(DshotCodec::QUINTET_TO_NIBBLE[DshotCodec::NIBBLE_TO_QUINTET[0]] == 0);
    static_assert(DshotCodec::QUINTET_TO_NIBBLE[DshotCodec::NIBBLE_TO_QUINTET[1]] == 1);
    static_assert(DshotCodec::QUINTET_TO_NIBBLE[DshotCodec::NIBBLE_TO_QUINTET[2]] == 2);
    static_assert(DshotCodec::QUINTET_TO_NIBBLE[DshotCodec::NIBBLE_TO_QUINTET[3]] == 3);
    static_assert(DshotCodec::QUINTET_TO_NIBBLE[DshotCodec::NIBBLE_TO_QUINTET[4]] == 4);
    static_assert(DshotCodec::QUINTET_TO_NIBBLE[DshotCodec::NIBBLE_TO_QUINTET[5]] == 5);
    static_assert(DshotCodec::QUINTET_TO_NIBBLE[DshotCodec::NIBBLE_TO_QUINTET[6]] == 6);
    static_assert(DshotCodec::QUINTET_TO_NIBBLE[DshotCodec::NIBBLE_TO_QUINTET[7]] == 7);
    static_assert(DshotCodec::QUINTET_TO_NIBBLE[DshotCodec::NIBBLE_TO_QUINTET[8]] == 8);
    static_assert(DshotCodec::QUINTET_TO_NIBBLE[DshotCodec::NIBBLE_TO_QUINTET[9]] == 9);
    static_assert(DshotCodec::QUINTET_TO_NIBBLE[DshotCodec::NIBBLE_TO_QUINTET[10]] == 10);
    static_assert(DshotCodec::QUINTET_TO_NIBBLE[DshotCodec::NIBBLE_TO_QUINTET[11]] == 11);
    static_assert(DshotCodec::QUINTET_TO_NIBBLE[DshotCodec::NIBBLE_TO_QUINTET[12]] == 12);
    static_assert(DshotCodec::QUINTET_TO_NIBBLE[DshotCodec::NIBBLE_TO_QUINTET[13]] == 13);
    static_assert(DshotCodec::QUINTET_TO_NIBBLE[DshotCodec::NIBBLE_TO_QUINTET[14]] == 14);
    static_assert(DshotCodec::QUINTET_TO_NIBBLE[DshotCodec::NIBBLE_TO_QUINTET[15]] == 15);
}

void test_dshot_codec_checksum()
{
    TEST_ASSERT_EQUAL(0b000000000110, DshotCodec::checksum_unidirectional(0b100000101100));
    TEST_ASSERT_EQUAL(0b000000001001, DshotCodec::checksum_bidirectional (0b100000101100));

    TEST_ASSERT_EQUAL(0b1000001011000110, DshotCodec::frame_unidirectional(0b010000010110));
    TEST_ASSERT_EQUAL(0b1000001011001001, DshotCodec::frame_bidirectional (0b010000010110));

    TEST_ASSERT_TRUE(DshotCodec::checksum_unidirectional_is_ok(DshotCodec::frame_unidirectional(0b010000010110)));
    TEST_ASSERT_TRUE(DshotCodec::checksum_bidirectional_is_ok(DshotCodec::frame_bidirectional(0b010000010110)));
}

void test_dshot_codec_mappings()
{
    TEST_ASSERT_EQUAL(0b11010100101111010110, DshotCodec::erpm_to_gcr20(0b1000001011000110));
    TEST_ASSERT_EQUAL(0b1000001011000110, DshotCodec::gcr20_to_erpm(0b11010100101111010110));

    TEST_ASSERT_EQUAL(0b010101010101010101010, DshotCodec::gcr21_to_gcr20(0b011001100110011001100));
    TEST_ASSERT_EQUAL(0b011001100110011001100, DshotCodec::gr20_to_gcr21(0b10101010101010101010));
}

void test_dshot_codec()
{
    TEST_ASSERT_EQUAL(47, DshotCodec::pwm_to_dshot(1000));
    TEST_ASSERT_EQUAL(2047, DshotCodec::pwm_to_dshot(2000));

    TEST_ASSERT_EQUAL(0, DshotCodec::pwm_to_dshot_clamped(0));
    TEST_ASSERT_EQUAL(0, DshotCodec::pwm_to_dshot_clamped(10));
    TEST_ASSERT_EQUAL(0, DshotCodec::pwm_to_dshot_clamped(999));

    TEST_ASSERT_EQUAL(0, DshotCodec::pwm_to_dshot_clamped(1000)); // should this be 0 or 48 ?
    //TEST_ASSERT_EQUAL(48, DshotCodec::pwm_to_dshot_clamped(1000)); // should this be 0 or 48 ?
    TEST_ASSERT_EQUAL(49, DshotCodec::pwm_to_dshot_clamped(1001));
    TEST_ASSERT_EQUAL(51, DshotCodec::pwm_to_dshot_clamped(1002));
    TEST_ASSERT_EQUAL(53, DshotCodec::pwm_to_dshot_clamped(1003));
    TEST_ASSERT_EQUAL(1047, DshotCodec::pwm_to_dshot_clamped(1500));
    TEST_ASSERT_EQUAL(2045, DshotCodec::pwm_to_dshot_clamped(1999));
    TEST_ASSERT_EQUAL(2047, DshotCodec::pwm_to_dshot_clamped(2000));
    TEST_ASSERT_EQUAL(2047, DshotCodec::pwm_to_dshot_clamped(2001));
    TEST_ASSERT_EQUAL(2047, DshotCodec::pwm_to_dshot_clamped(2002));
    TEST_ASSERT_EQUAL(2047, DshotCodec::pwm_to_dshot_clamped(4000));


    TEST_ASSERT_EQUAL(1542, DshotCodec::frame_unidirectional(48)); //0x606
    TEST_ASSERT_EQUAL(1572, DshotCodec::frame_unidirectional(49)); // 0x624
    TEST_ASSERT_EQUAL(33547, DshotCodec::frame_unidirectional(1048)); // 0x830B
    TEST_ASSERT_EQUAL(65484, DshotCodec::frame_unidirectional(2046)); // 0xFFCC
    TEST_ASSERT_EQUAL(65518, DshotCodec::frame_unidirectional(2047)); // 0xFFEB, 0xFFFF=65535

    // testing out of range values
    TEST_ASSERT_EQUAL(0, DshotCodec::frame_unidirectional(0));
    TEST_ASSERT_EQUAL(34, DshotCodec::frame_unidirectional(1));
    TEST_ASSERT_EQUAL(68, DshotCodec::frame_unidirectional(2));
    TEST_ASSERT_EQUAL(325, DshotCodec::frame_unidirectional(10));

    //TEST_ASSERT_EQUAL(1, DshotCodec::frame_unidirectional(2048));
    //TEST_ASSERT_EQUAL(35, DshotCodec::frame_unidirectional(2049));
    //TEST_ASSERT_EQUAL(69, DshotCodec::frame_unidirectional(2050));
}

// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,cppcoreguidelines-pro-bounds-constant-array-index,cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-signed-bitwise,readability-magic-numbers)

// See for example for testing GCR encoding https://github.com/betaflight/betaflight/pull/8554#issuecomment-512507625
// see also https://elmagnifico.tech/2023/04/07/bi-directional-DSHOT/

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_dshot_codec_checksum);
    RUN_TEST(test_dshot_codec_mappings);
    RUN_TEST(test_dshot_codec);

    UNITY_END();
}
