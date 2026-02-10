#include <DshotCodec.h>
#include <EscDshot.h>
#include <unity.h>

void setUp()
{
}

void tearDown()
{
}

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-signed-bitwise,readability-magic-numbers)
void test_dshot()
{
    enum { DSHOT_PIN = 4 };

    static EscDshot esc(EscDshot::ESC_PROTOCOL_DSHOT300);
    esc.init(DSHOT_PIN);
/*
    DShot 300 specification is
    T0H = 1250ns (data low pulse width)
    T0L = 2090ns (data low gap width)
    T1H = 2500ns (data high pulse width)
    T1L =  840ns (data high gap width)
    TxH+TxL = 3340ns  (T0H + T0L or T1H + T1L)
*/

    TEST_ASSERT_EQUAL(126, esc.nano_seconds_to_cycles(840));
    TEST_ASSERT_EQUAL(187, esc.nano_seconds_to_cycles(1250));
    TEST_ASSERT_EQUAL(313, esc.nano_seconds_to_cycles(2090));
    TEST_ASSERT_EQUAL(375, esc.nano_seconds_to_cycles(2500));
    TEST_ASSERT_EQUAL(750, esc.nano_seconds_to_cycles(5000));
    TEST_ASSERT_EQUAL(1125, esc.nano_seconds_to_cycles(7500));

    TEST_ASSERT_EQUAL(350, esc.nano_seconds_to_cycles(2333));
    TEST_ASSERT_EQUAL(700, esc.nano_seconds_to_cycles(4666));
    TEST_ASSERT_EQUAL(1000, esc.nano_seconds_to_cycles(6666));

    esc.end();
}

void test_dshot_init()
{
    enum { DSHOT_PIN = 4 };

    static EscDshot esc150(EscDshot::ESC_PROTOCOL_DSHOT150);
    esc150.init(DSHOT_PIN);
    TEST_ASSERT_EQUAL(750, esc150.get_data_high_pulse_width());
    TEST_ASSERT_EQUAL(375, esc150.get_data_low_pulse_width());

    static EscDshot esc300(EscDshot::ESC_PROTOCOL_DSHOT300);
    esc300.init(DSHOT_PIN);
    TEST_ASSERT_EQUAL(375, esc300.get_data_high_pulse_width());
    TEST_ASSERT_EQUAL(187, esc300.get_data_low_pulse_width());

    static EscDshot esc600(EscDshot::ESC_PROTOCOL_DSHOT600);
    esc600.init(DSHOT_PIN);
    TEST_ASSERT_EQUAL(187, esc600.get_data_high_pulse_width());
    TEST_ASSERT_EQUAL(93, esc600.get_data_low_pulse_width());
}

void test_dshot_write()
{
    enum { DSHOT_PIN = 4 };

    static EscDshot esc(EscDshot::ESC_PROTOCOL_DSHOT300);
    esc.init(DSHOT_PIN);

    const uint32_t HI = esc.get_data_high_pulse_width();
    const uint32_t LO = esc.get_data_low_pulse_width();

    TEST_ASSERT_EQUAL(375, HI);
    TEST_ASSERT_EQUAL(187, LO);

    const uint16_t dshot1000 = DshotCodec::pwm_to_dshot_clamped(1000);
    esc.write(dshot1000);
    const uint16_t frame1000 = DshotCodec::frame_unidirectional(dshot1000);
    TEST_ASSERT_EQUAL(0, frame1000);
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(0));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(1));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(2));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(3));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(4));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(5));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(6));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(7));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(8));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(9));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(10));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(11));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(12));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(13));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(14));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(15));
    TEST_ASSERT_EQUAL(0, esc.getBufferItem(16)); // check haven't written past end of buffer

    const uint16_t dshot1500 = DshotCodec::pwm_to_dshot(1500);
    const uint16_t frame = DshotCodec::frame_unidirectional(dshot1500);
    TEST_ASSERT_EQUAL(33508, frame); // 0x82E4, 1000 0010 1110 0100
    esc.write(dshot1500);
    // 1000 0010 1110 0100
    // 1000
    TEST_ASSERT_EQUAL(HI, esc.getBufferItem(0));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(1));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(2));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(3));
    // 0010
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(4));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(5));
    TEST_ASSERT_EQUAL(HI, esc.getBufferItem(6));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(7));
    // 1110
    TEST_ASSERT_EQUAL(HI, esc.getBufferItem(8));
    TEST_ASSERT_EQUAL(HI, esc.getBufferItem(9));
    TEST_ASSERT_EQUAL(HI, esc.getBufferItem(10));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(11));
    //0100
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(12));
    TEST_ASSERT_EQUAL(HI, esc.getBufferItem(13));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(14));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(15));
    TEST_ASSERT_EQUAL(0, esc.getBufferItem(16));

    const uint16_t dshot2000 = DshotCodec::pwm_to_dshot(2000);
    const uint16_t frame2000 = DshotCodec::frame_unidirectional(dshot2000);
    TEST_ASSERT_EQUAL(0xFFEE, frame2000); // 0xFFEE, 1111 1111 1110 1110
    esc.write(dshot2000);
    // 1111
    TEST_ASSERT_EQUAL(HI, esc.getBufferItem(0));
    TEST_ASSERT_EQUAL(HI, esc.getBufferItem(1));
    TEST_ASSERT_EQUAL(HI, esc.getBufferItem(2));
    TEST_ASSERT_EQUAL(HI, esc.getBufferItem(3));
    // 1110
    TEST_ASSERT_EQUAL(HI, esc.getBufferItem(12));
    TEST_ASSERT_EQUAL(HI, esc.getBufferItem(13));
    TEST_ASSERT_EQUAL(HI, esc.getBufferItem(14));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(15));
    TEST_ASSERT_EQUAL(0, esc.getBufferItem(16));
}

void test_dshot_write_channel_b()
{
    enum { DSHOT_PIN = 4 };

    static EscDshot esc(EscDshot::ESC_PROTOCOL_DSHOT300);
    esc.init(DSHOT_PIN);
    esc.set_use_high_order_bits(true);

    // channel B uses high order bits
    const uint32_t HI = esc.get_data_high_pulse_width() << 16;
    const uint32_t LO = esc.get_data_low_pulse_width() << 16;

    const uint16_t dshot1000 = DshotCodec::pwm_to_dshot_clamped(1000);
    esc.write(dshot1000);
    const uint16_t frame1000 = DshotCodec::frame_unidirectional(dshot1000);
    TEST_ASSERT_EQUAL(0, frame1000);
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(0));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(15));
    TEST_ASSERT_EQUAL(0, esc.getBufferItem(16)); // check haven't written past end of buffer

    const uint16_t dshot1500 = DshotCodec::pwm_to_dshot(1500);
    const uint16_t frame = DshotCodec::frame_unidirectional(dshot1500);
    TEST_ASSERT_EQUAL(33508, frame); // 0x82E4, 1000 0010 1110 0100
    esc.write(dshot1500);
    // 1000 0010 1110 0100
    // 1000
    TEST_ASSERT_EQUAL(HI, esc.getBufferItem(0));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(1));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(2));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(3));
    // 0010
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(4));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(5));
    TEST_ASSERT_EQUAL(HI, esc.getBufferItem(6));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(7));
    // 1110
    TEST_ASSERT_EQUAL(HI, esc.getBufferItem(8));
    TEST_ASSERT_EQUAL(HI, esc.getBufferItem(9));
    TEST_ASSERT_EQUAL(HI, esc.getBufferItem(10));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(11));
    //0100
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(12));
    TEST_ASSERT_EQUAL(HI, esc.getBufferItem(13));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(14));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(15));
    TEST_ASSERT_EQUAL(0, esc.getBufferItem(16));

    const uint16_t dshot2000 = DshotCodec::pwm_to_dshot(2000);
    const uint16_t frame2000 = DshotCodec::frame_unidirectional(dshot2000);
    TEST_ASSERT_EQUAL(65518, frame2000); // 0xFFEE, 1111 1111 1110 1110
    esc.write(dshot2000);
    // 1111
    TEST_ASSERT_EQUAL(HI, esc.getBufferItem(0));
    TEST_ASSERT_EQUAL(HI, esc.getBufferItem(1));
    TEST_ASSERT_EQUAL(HI, esc.getBufferItem(2));
    TEST_ASSERT_EQUAL(HI, esc.getBufferItem(3));
    // 1110
    TEST_ASSERT_EQUAL(HI, esc.getBufferItem(12));
    TEST_ASSERT_EQUAL(HI, esc.getBufferItem(13));
    TEST_ASSERT_EQUAL(HI, esc.getBufferItem(14));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(15));
    TEST_ASSERT_EQUAL(0, esc.getBufferItem(16));
}
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-signed-bitwise,readability-magic-numbers)

// See for example for testing GCR encoding https://github.com/betaflight/betaflight/pull/8554#issuecomment-512507625
// see also https://elmagnifico.tech/2023/04/07/bi-directional-DSHOT/
int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_dshot);
    RUN_TEST(test_dshot_init);
    RUN_TEST(test_dshot_write);
    RUN_TEST(test_dshot_write_channel_b);

    UNITY_END();
}
