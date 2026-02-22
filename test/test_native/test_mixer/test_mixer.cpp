#include <mixers.h>
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
    std::array<float, 4> motor_outputs {};
    motor_mixer_commands_t commands {};
    float throttle = 0.0F;
    motor_mixer_parameters_t mixParams {
        .motor_output_min = 0.0F,
        .motor_output_max = 1.0F,
        .max_servo_angle_radians = 0.0F,
        .undershoot = 0.0F,
        .overshoot = 0.0F,
    };

    throttle = mix_quad_x(motor_outputs, commands, mixParams);

    TEST_ASSERT_EQUAL_FLOAT(0.0F, mixParams.undershoot);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, mixParams.overshoot);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, throttle);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, motor_outputs[0]);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, motor_outputs[1]);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, motor_outputs[2]);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, motor_outputs[3]);

    commands.throttle = 0.4F;
    commands.roll = 0.3F;
    throttle = mix_quad_x(motor_outputs, commands, mixParams);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, mixParams.undershoot);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, mixParams.overshoot);
    TEST_ASSERT_EQUAL_FLOAT(0.4F, throttle);
    TEST_ASSERT_EQUAL_FLOAT(0.1F, motor_outputs[0]); // throttle - commands.roll
    TEST_ASSERT_EQUAL_FLOAT(0.1F, motor_outputs[1]); // throttle - commands.roll
    TEST_ASSERT_EQUAL_FLOAT(0.7F, motor_outputs[2]); // throttle + commands.roll
    TEST_ASSERT_EQUAL_FLOAT(0.7F, motor_outputs[3]); // throttle + commands.roll

    commands.throttle = 0.8F;
    commands.roll = 0.3F;
    throttle = mix_quad_x(motor_outputs, commands, mixParams);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, mixParams.undershoot);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, mixParams.overshoot);
    TEST_ASSERT_EQUAL_FLOAT(0.8F, throttle);
    TEST_ASSERT_EQUAL_FLOAT(0.5F, motor_outputs[0]); // throttle - commands.roll
    TEST_ASSERT_EQUAL_FLOAT(0.5F, motor_outputs[1]); // throttle - commands.roll
    TEST_ASSERT_EQUAL_FLOAT(1.0F, motor_outputs[2]); // throttle + commands.roll
    TEST_ASSERT_EQUAL_FLOAT(1.0F, motor_outputs[3]); // throttle + commands.roll

    commands.throttle = 0.1F;
    commands.roll = 0.3F;
    throttle = mix_quad_x(motor_outputs, commands, mixParams);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, mixParams.undershoot);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, mixParams.overshoot);
    TEST_ASSERT_EQUAL_FLOAT(0.1F, throttle);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, motor_outputs[0]); // throttle - commands.roll
    TEST_ASSERT_EQUAL_FLOAT(0.0F, motor_outputs[1]); // throttle - commands.roll
    TEST_ASSERT_EQUAL_FLOAT(0.4F, motor_outputs[2]); // throttle + commands.roll
    TEST_ASSERT_EQUAL_FLOAT(0.4F, motor_outputs[3]); // throttle + commands.roll
}

void test_mixer_quad_x_pitch()
{
    std::array<float, 4> motor_outputs {};
    motor_mixer_commands_t commands {};
    float throttle {};
    motor_mixer_parameters_t mixParams {
        .motor_output_min = 0.0F,
        .motor_output_max = 1.0F,
        .max_servo_angle_radians = 0.0F,
        .undershoot = 0.0F,
        .overshoot = 0.0F,
    };

    commands.throttle = 0.4F;
    commands.pitch = 0.3F;
    throttle = mix_quad_x(motor_outputs, commands, mixParams);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, mixParams.undershoot);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, mixParams.overshoot);
    TEST_ASSERT_EQUAL_FLOAT(0.4F, throttle);
    TEST_ASSERT_EQUAL_FLOAT(0.1F, motor_outputs[0]); // throttle - commands.pitch
    TEST_ASSERT_EQUAL_FLOAT(0.7F, motor_outputs[1]); // throttle + commands.pitch
    TEST_ASSERT_EQUAL_FLOAT(0.1F, motor_outputs[2]); // throttle - commands.pitch
    TEST_ASSERT_EQUAL_FLOAT(0.7F, motor_outputs[3]); // throttle + commands.pitch

    commands.throttle = 0.8F;
    commands.pitch = 0.3F;
    throttle = mix_quad_x(motor_outputs, commands, mixParams);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, mixParams.undershoot);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, mixParams.overshoot); // pitch overshoot is ignored
    TEST_ASSERT_EQUAL_FLOAT(0.8F, throttle);
    TEST_ASSERT_EQUAL_FLOAT(0.5F, motor_outputs[0]); // throttle - commands.pitch
    TEST_ASSERT_EQUAL_FLOAT(1.0F, motor_outputs[1]); // throttle + commands.pitch
    TEST_ASSERT_EQUAL_FLOAT(0.5F, motor_outputs[2]); // throttle - commands.pitch
    TEST_ASSERT_EQUAL_FLOAT(1.0F, motor_outputs[3]); // throttle + commands.pitch

    commands.throttle = 0.1F;
    commands.pitch = 0.3F;
    throttle = mix_quad_x(motor_outputs, commands, mixParams);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, mixParams.undershoot);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, mixParams.overshoot); // pitch overshoot is ignored
    TEST_ASSERT_EQUAL_FLOAT(0.1F, throttle);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, motor_outputs[0]); // throttle - commands.pitch
    TEST_ASSERT_EQUAL_FLOAT(0.4F, motor_outputs[1]); // throttle + commands.pitch
    TEST_ASSERT_EQUAL_FLOAT(0.0F, motor_outputs[2]); // throttle - commands.pitch
    TEST_ASSERT_EQUAL_FLOAT(0.4F, motor_outputs[3]); // throttle + commands.pitch
}

void test_mixer_quad_x_yaw()
{
    std::array<float, 4> motor_outputs {};
    motor_mixer_commands_t commands {};
    float throttle = 0.0F;
    motor_mixer_parameters_t mixParams {
        .motor_output_min = 0.0F,
        .motor_output_max = 1.0F,
        .max_servo_angle_radians = 0.0F,
        .undershoot = 0.0F,
        .overshoot = 0.0F,
    };

    commands.throttle = 0.4F;
    commands.yaw = 0.3F;
    throttle = mix_quad_x(motor_outputs, commands, mixParams);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, mixParams.undershoot);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, mixParams.overshoot);
    TEST_ASSERT_EQUAL_FLOAT(0.4F, throttle);
    TEST_ASSERT_EQUAL_FLOAT(0.7F, motor_outputs[0]); // throttle + commands.yaw
    TEST_ASSERT_EQUAL_FLOAT(0.1F, motor_outputs[1]); // throttle - commands.yaw
    TEST_ASSERT_EQUAL_FLOAT(0.1F, motor_outputs[2]); // throttle - commands.yaw
    TEST_ASSERT_EQUAL_FLOAT(0.7F, motor_outputs[3]); // throttle + commands.yaw

    // this will give an undershoot of -0.1F, so commands.yaw should be adjusted to 0.2F
    commands.throttle = 0.4F;
    commands.yaw = 0.3F;
    mixParams.motor_output_min = 0.2F;
    throttle = mix_quad_x(motor_outputs, commands, mixParams);
    TEST_ASSERT_EQUAL_FLOAT(-0.1F, mixParams.undershoot);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, mixParams.overshoot);
    TEST_ASSERT_EQUAL_FLOAT(0.5F, throttle);
    TEST_ASSERT_EQUAL_FLOAT(0.6F, motor_outputs[0]); // throttle + commands.yaw
    TEST_ASSERT_EQUAL_FLOAT(0.2F, motor_outputs[1]); // throttle - commands.yaw
    TEST_ASSERT_EQUAL_FLOAT(0.2F, motor_outputs[2]); // throttle - commands.yaw
    TEST_ASSERT_EQUAL_FLOAT(0.6F, motor_outputs[3]); // throttle + commands.yaw

    // this will give an undershoot of -0.1F, so commands.yaw should be adjusted to -0.2F
    commands.throttle = 0.4F;
    commands.yaw = -0.3F;
    mixParams.motor_output_min = 0.2F;
    throttle = mix_quad_x(motor_outputs, commands, mixParams);
    TEST_ASSERT_EQUAL_FLOAT(-0.1F, mixParams.undershoot);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, mixParams.overshoot);
    TEST_ASSERT_EQUAL_FLOAT(0.5F, throttle);
    TEST_ASSERT_EQUAL_FLOAT(0.2F, motor_outputs[0]); // throttle + commands.yaw
    TEST_ASSERT_EQUAL_FLOAT(0.6F, motor_outputs[1]); // throttle - commands.yaw
    TEST_ASSERT_EQUAL_FLOAT(0.6F, motor_outputs[2]); // throttle - commands.yaw
    TEST_ASSERT_EQUAL_FLOAT(0.2F, motor_outputs[3]); // throttle + commands.yaw

    // this will give an overshoot of 0.1F, so commands.yaw should be adjusted to 0.2F
    commands.throttle = 0.8F;
    commands.yaw = 0.3F;
    mixParams.motor_output_min = 0.0F;
    throttle = mix_quad_x(motor_outputs, commands, mixParams);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, mixParams.undershoot);
    TEST_ASSERT_EQUAL_FLOAT(0.1F, mixParams.overshoot);
    TEST_ASSERT_EQUAL_FLOAT(0.7F, throttle);
    TEST_ASSERT_EQUAL_FLOAT(1.0F, motor_outputs[0]); // throttle + commands.yaw
    TEST_ASSERT_EQUAL_FLOAT(0.6F, motor_outputs[1]); // throttle - commands.yaw
    TEST_ASSERT_EQUAL_FLOAT(0.6F, motor_outputs[2]); // throttle - commands.yaw
    TEST_ASSERT_EQUAL_FLOAT(1.0F, motor_outputs[3]); // throttle + commands.yaw

    // this will give an overshoot of 0.1F, so commands.yaw should be adjusted to -0.2F
    commands.throttle = 0.8F;
    commands.yaw = -0.3F;
    mixParams.motor_output_min = 0.0F;
    throttle = mix_quad_x(motor_outputs, commands, mixParams);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, mixParams.undershoot);
    TEST_ASSERT_EQUAL_FLOAT(0.1F, mixParams.overshoot);
    TEST_ASSERT_EQUAL_FLOAT(0.7F, throttle);
    TEST_ASSERT_EQUAL_FLOAT(0.6F, motor_outputs[0]); // throttle + commands.yaw
    TEST_ASSERT_EQUAL_FLOAT(1.0F, motor_outputs[1]); // throttle - commands.yaw
    TEST_ASSERT_EQUAL_FLOAT(1.0F, motor_outputs[2]); // throttle - commands.yaw
    TEST_ASSERT_EQUAL_FLOAT(0.6F, motor_outputs[3]); // throttle + commands.yaw
}

void test_mixer_tricopter()
{
    enum { REAR = 0, FR = 1, FL = 2, S0 = 3};
    std::array<float, 4> motor_outputs {};
    float throttle = 0.0F;
    motor_mixer_commands_t commands {};
    motor_mixer_parameters_t mixParams {
        .motor_output_min = 0.1F,
        .motor_output_max = 1.0F,
        .max_servo_angle_radians = static_cast<float>(60.0 * M_PI / 180.0),
        .undershoot = 0.0F,
        .overshoot = 0.0F,
    };

    commands.throttle = 0.4F;
    throttle = mix_tricopter(motor_outputs, commands, mixParams);
    TEST_ASSERT_EQUAL_FLOAT(0.3F, mixParams.undershoot);
    TEST_ASSERT_EQUAL_FLOAT(-0.6F, mixParams.overshoot);
    TEST_ASSERT_EQUAL_FLOAT(0.4F, throttle);
    TEST_ASSERT_EQUAL_FLOAT(0.4F, motor_outputs[FL]);
    TEST_ASSERT_EQUAL_FLOAT(0.4F, motor_outputs[FR]);
    TEST_ASSERT_EQUAL_FLOAT(0.4, motor_outputs[REAR]);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, motor_outputs[S0]);


    commands.yaw = 0.3F;
    throttle = mix_tricopter(motor_outputs, commands, mixParams);
    TEST_ASSERT_EQUAL_FLOAT(0.3F, mixParams.undershoot);
    TEST_ASSERT_EQUAL_FLOAT(-0.5794151F, mixParams.overshoot);
    TEST_ASSERT_EQUAL_FLOAT(0.4F, throttle);
    TEST_ASSERT_EQUAL_FLOAT(0.4F, motor_outputs[FL]);
    TEST_ASSERT_EQUAL_FLOAT(0.4F, motor_outputs[FR]);
    TEST_ASSERT_EQUAL_FLOAT(0.42058489F, motor_outputs[REAR]);
    TEST_ASSERT_EQUAL_FLOAT(0.3F, motor_outputs[S0]);

    commands.yaw = 1.0F;
    throttle = mix_tricopter(motor_outputs, commands, mixParams);
    TEST_ASSERT_EQUAL_FLOAT(0.3F, mixParams.undershoot);
    TEST_ASSERT_EQUAL_FLOAT(-0.2F, mixParams.overshoot);
    TEST_ASSERT_EQUAL_FLOAT(0.4F, throttle);
    TEST_ASSERT_EQUAL_FLOAT(0.4F, motor_outputs[FL]);
    TEST_ASSERT_EQUAL_FLOAT(0.4F, motor_outputs[FR]);
    TEST_ASSERT_EQUAL_FLOAT(0.8, motor_outputs[REAR]);
    TEST_ASSERT_EQUAL_FLOAT(1.0F, motor_outputs[S0]);
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
