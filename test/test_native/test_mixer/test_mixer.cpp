#include <Mixers.h>
#include <unity.h>

void setUp()
{
}

void tearDown()
{
}

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-signed-bitwise,readability-magic-numbers)
void test_mixer_quad_x_roll()
{
    std::array<float, 4> motorOutputs {};
    MotorMixerBase::commands_t commands {};
    float throttle = 0.0F;
    MotorMixerBase::parameters_t mixParams {
        .motorOutputMin = 0.0F,
        .motorOutputMax = 1.0F,
        .maxServoAngleRadians = 0.0F,
        .undershoot = 0.0F,
        .overshoot = 0.0F,
    };

    throttle = mixQuadX(motorOutputs, commands, mixParams);

    TEST_ASSERT_EQUAL_FLOAT(0.0F, mixParams.undershoot);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, mixParams.overshoot);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, throttle);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, motorOutputs[0]);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, motorOutputs[1]);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, motorOutputs[2]);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, motorOutputs[3]);

    commands.throttle = 0.4F;
    commands.roll = 0.3F;
    throttle = mixQuadX(motorOutputs, commands, mixParams);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, mixParams.undershoot);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, mixParams.overshoot);
    TEST_ASSERT_EQUAL_FLOAT(0.4F, throttle);
    TEST_ASSERT_EQUAL_FLOAT(0.1F, motorOutputs[0]); // throttle - commands.roll
    TEST_ASSERT_EQUAL_FLOAT(0.1F, motorOutputs[1]); // throttle - commands.roll
    TEST_ASSERT_EQUAL_FLOAT(0.7F, motorOutputs[2]); // throttle + commands.roll
    TEST_ASSERT_EQUAL_FLOAT(0.7F, motorOutputs[3]); // throttle + commands.roll

    commands.throttle = 0.8F;
    commands.roll = 0.3F;
    throttle = mixQuadX(motorOutputs, commands, mixParams);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, mixParams.undershoot);
    TEST_ASSERT_EQUAL_FLOAT(0.1F, mixParams.overshoot);
    TEST_ASSERT_EQUAL_FLOAT(0.7F, throttle);
    TEST_ASSERT_EQUAL_FLOAT(0.4F, motorOutputs[0]); // throttle - commands.roll
    TEST_ASSERT_EQUAL_FLOAT(0.4F, motorOutputs[1]); // throttle - commands.roll
    TEST_ASSERT_EQUAL_FLOAT(1.0F, motorOutputs[2]); // throttle + commands.roll
    TEST_ASSERT_EQUAL_FLOAT(1.0F, motorOutputs[3]); // throttle + commands.roll
}

void test_mixer_quad_x_pitch()
{
    std::array<float, 4> motorOutputs {};
    MotorMixerBase::commands_t commands {};
    float throttle {};
    MotorMixerBase::parameters_t mixParams {
        .motorOutputMin = 0.0F,
        .motorOutputMax = 1.0F,
        .maxServoAngleRadians = 0.0F,
        .undershoot = 0.0F,
        .overshoot = 0.0F,
    };

    commands.throttle = 0.4F;
    commands.pitch = 0.3F;
    throttle = mixQuadX(motorOutputs, commands, mixParams);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, mixParams.undershoot);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, mixParams.overshoot);
    TEST_ASSERT_EQUAL_FLOAT(0.4F, throttle);
    TEST_ASSERT_EQUAL_FLOAT(0.1F, motorOutputs[0]); // throttle - commands.pitch
    TEST_ASSERT_EQUAL_FLOAT(0.7F, motorOutputs[1]); // throttle + commands.pitch
    TEST_ASSERT_EQUAL_FLOAT(0.1F, motorOutputs[2]); // throttle - commands.pitch
    TEST_ASSERT_EQUAL_FLOAT(0.7F, motorOutputs[3]); // throttle + commands.pitch

    commands.throttle = 0.8F;
    commands.pitch = 0.3F;
    throttle = mixQuadX(motorOutputs, commands, mixParams);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, mixParams.undershoot);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, mixParams.overshoot); // pitch overshoot is ignored
    TEST_ASSERT_EQUAL_FLOAT(0.7F, throttle);
    TEST_ASSERT_EQUAL_FLOAT(0.4F, motorOutputs[0]); // throttle - commands.pitch
    TEST_ASSERT_EQUAL_FLOAT(1.0F, motorOutputs[1]); // throttle + commands.pitch
    TEST_ASSERT_EQUAL_FLOAT(0.4F, motorOutputs[2]); // throttle - commands.pitch
    TEST_ASSERT_EQUAL_FLOAT(1.0F, motorOutputs[3]); // throttle + commands.pitch
}

void test_mixer_quad_x_yaw()
{
    std::array<float, 4> motorOutputs {};
    MotorMixerBase::commands_t commands {};
    float throttle = 0.0F;
    MotorMixerBase::parameters_t mixParams {
        .motorOutputMin = 0.0F,
        .motorOutputMax = 1.0F,
        .maxServoAngleRadians = 0.0F,
        .undershoot = 0.0F,
        .overshoot = 0.0F,
    };

    commands.throttle = 0.4F;
    commands.yaw = 0.3F;
    throttle = mixQuadX(motorOutputs, commands, mixParams);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, mixParams.undershoot);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, mixParams.overshoot);
    TEST_ASSERT_EQUAL_FLOAT(0.4F, throttle);
    TEST_ASSERT_EQUAL_FLOAT(0.7F, motorOutputs[0]); // throttle + commands.yaw
    TEST_ASSERT_EQUAL_FLOAT(0.1F, motorOutputs[1]); // throttle - commands.yaw
    TEST_ASSERT_EQUAL_FLOAT(0.1F, motorOutputs[2]); // throttle - commands.yaw
    TEST_ASSERT_EQUAL_FLOAT(0.7F, motorOutputs[3]); // throttle + commands.yaw

    // this will give an undershoot of -0.1F, so commands.yaw should be adjusted to 0.2F
    commands.throttle = 0.4F;
    commands.yaw = 0.3F;
    mixParams.motorOutputMin = 0.2F;
    throttle = mixQuadX(motorOutputs, commands, mixParams);
    TEST_ASSERT_EQUAL_FLOAT(-0.1F, mixParams.undershoot);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, mixParams.overshoot);
    TEST_ASSERT_EQUAL_FLOAT(0.5F, throttle);
    TEST_ASSERT_EQUAL_FLOAT(0.6F, motorOutputs[0]); // throttle + commands.yaw
    TEST_ASSERT_EQUAL_FLOAT(0.2F, motorOutputs[1]); // throttle - commands.yaw
    TEST_ASSERT_EQUAL_FLOAT(0.2F, motorOutputs[2]); // throttle - commands.yaw
    TEST_ASSERT_EQUAL_FLOAT(0.6F, motorOutputs[3]); // throttle + commands.yaw

    // this will give an undershoot of -0.1F, so commands.yaw should be adjusted to -0.2F
    commands.throttle = 0.4F;
    commands.yaw = -0.3F;
    mixParams.motorOutputMin = 0.2F;
    throttle = mixQuadX(motorOutputs, commands, mixParams);
    TEST_ASSERT_EQUAL_FLOAT(-0.1F, mixParams.undershoot);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, mixParams.overshoot);
    TEST_ASSERT_EQUAL_FLOAT(0.5F, throttle);
    TEST_ASSERT_EQUAL_FLOAT(0.2F, motorOutputs[0]); // throttle + commands.yaw
    TEST_ASSERT_EQUAL_FLOAT(0.6F, motorOutputs[1]); // throttle - commands.yaw
    TEST_ASSERT_EQUAL_FLOAT(0.6F, motorOutputs[2]); // throttle - commands.yaw
    TEST_ASSERT_EQUAL_FLOAT(0.2F, motorOutputs[3]); // throttle + commands.yaw

    // this will give an overshoot of 0.1F, so commands.yaw should be adjusted to 0.2F
    commands.throttle = 0.8F;
    commands.yaw = 0.3F;
    mixParams.motorOutputMin = 0.0F;
    throttle = mixQuadX(motorOutputs, commands, mixParams);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, mixParams.undershoot);
    TEST_ASSERT_EQUAL_FLOAT(0.1F, mixParams.overshoot);
    TEST_ASSERT_EQUAL_FLOAT(0.7F, throttle);
    TEST_ASSERT_EQUAL_FLOAT(1.0F, motorOutputs[0]); // throttle + commands.yaw
    TEST_ASSERT_EQUAL_FLOAT(0.6F, motorOutputs[1]); // throttle - commands.yaw
    TEST_ASSERT_EQUAL_FLOAT(0.6F, motorOutputs[2]); // throttle - commands.yaw
    TEST_ASSERT_EQUAL_FLOAT(1.0F, motorOutputs[3]); // throttle + commands.yaw

    // this will give an overshoot of 0.1F, so commands.yaw should be adjusted to -0.2F
    commands.throttle = 0.8F;
    commands.yaw = -0.3F;
    mixParams.motorOutputMin = 0.0F;
    throttle = mixQuadX(motorOutputs, commands, mixParams);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, mixParams.undershoot);
    TEST_ASSERT_EQUAL_FLOAT(0.1F, mixParams.overshoot);
    TEST_ASSERT_EQUAL_FLOAT(0.7F, throttle);
    TEST_ASSERT_EQUAL_FLOAT(0.6F, motorOutputs[0]); // throttle + commands.yaw
    TEST_ASSERT_EQUAL_FLOAT(1.0F, motorOutputs[1]); // throttle - commands.yaw
    TEST_ASSERT_EQUAL_FLOAT(1.0F, motorOutputs[2]); // throttle - commands.yaw
    TEST_ASSERT_EQUAL_FLOAT(0.6F, motorOutputs[3]); // throttle + commands.yaw
}

void test_mixer_tricopter()
{
    enum { REAR = 0, FR = 1, FL = 2, S0 = 3};
    std::array<float, 4> motorOutputs {};
    float throttle = 0.0F;
    MotorMixerBase::commands_t commands {};
    MotorMixerBase::parameters_t mixParams {
        .motorOutputMin = 0.1F,
        .motorOutputMax = 1.0F,
        .maxServoAngleRadians = static_cast<float>(60.0 * M_PI / 180.0),
        .undershoot = 0.0F,
        .overshoot = 0.0F,
    };

    commands.throttle = 0.4F;
    throttle = mixTricopter(motorOutputs, commands, mixParams);
    TEST_ASSERT_EQUAL_FLOAT(-0.3F, mixParams.undershoot);
    TEST_ASSERT_EQUAL_FLOAT(-0.6F, mixParams.overshoot);
    TEST_ASSERT_EQUAL_FLOAT(0.4F, throttle);
    TEST_ASSERT_EQUAL_FLOAT(0.4F, motorOutputs[FL]);
    TEST_ASSERT_EQUAL_FLOAT(0.4F, motorOutputs[FR]);
    TEST_ASSERT_EQUAL_FLOAT(0.4, motorOutputs[REAR]);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, motorOutputs[S0]);


    commands.yaw = 0.3F;
    throttle = mixTricopter(motorOutputs, commands, mixParams);
    TEST_ASSERT_EQUAL_FLOAT(-0.3F, mixParams.undershoot);
    TEST_ASSERT_EQUAL_FLOAT(-0.5794151F, mixParams.overshoot);
    TEST_ASSERT_EQUAL_FLOAT(0.4F, throttle);
    TEST_ASSERT_EQUAL_FLOAT(0.4F, motorOutputs[FL]);
    TEST_ASSERT_EQUAL_FLOAT(0.4F, motorOutputs[FR]);
    TEST_ASSERT_EQUAL_FLOAT(0.42058489F, motorOutputs[REAR]);
    TEST_ASSERT_EQUAL_FLOAT(0.3F, motorOutputs[S0]);

    commands.yaw = 1.0F;
    throttle = mixTricopter(motorOutputs, commands, mixParams);
    TEST_ASSERT_EQUAL_FLOAT(-0.3F, mixParams.undershoot);
    TEST_ASSERT_EQUAL_FLOAT(-0.2F, mixParams.overshoot);
    TEST_ASSERT_EQUAL_FLOAT(0.4F, throttle);
    TEST_ASSERT_EQUAL_FLOAT(0.4F, motorOutputs[FL]);
    TEST_ASSERT_EQUAL_FLOAT(0.4F, motorOutputs[FR]);
    TEST_ASSERT_EQUAL_FLOAT(0.8, motorOutputs[REAR]);
    TEST_ASSERT_EQUAL_FLOAT(1.0F, motorOutputs[S0]);
}

// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-signed-bitwise,readability-magic-numbers)

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    UNITY_BEGIN();

    RUN_TEST(test_mixer_quad_x_roll);
    RUN_TEST(test_mixer_quad_x_pitch);
    RUN_TEST(test_mixer_quad_x_yaw);
    RUN_TEST(test_mixer_tricopter);

    UNITY_END();
}
