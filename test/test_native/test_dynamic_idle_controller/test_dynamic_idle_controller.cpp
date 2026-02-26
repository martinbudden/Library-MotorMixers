#include <debug.h>
#include <dynamic_idle_controller.h>
#include <unity.h>

void setUp()
{
}

void tearDown()
{
}

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-signed-bitwise,readability-magic-numbers)
void test_dynamic_idle_controller()
{
    enum { TASK_INTERVAL_MICROSECONDS = 1000 };
    const dynamic_idle_controller_config_t dynamic_idle_controllerConfig = {
        .dyn_idle_min_rpm_100 = 0,
        .dyn_idle_p_gain = 50,
        .dyn_idle_i_gain = 50,
        .dyn_idle_d_gain = 50,
        .dyn_idle_max_increase = 150,
    };
    static Debug debug;
    static DynamicIdleController dynamic_idle_controller(TASK_INTERVAL_MICROSECONDS);
    dynamic_idle_controller.set_config(dynamic_idle_controllerConfig);
    const float delta_t = static_cast<float>(TASK_INTERVAL_MICROSECONDS) * 0.000001F;

    TEST_ASSERT_EQUAL(0, dynamic_idle_controller.get_config().dyn_idle_min_rpm_100);

    TEST_ASSERT_EQUAL_FLOAT(0.0F, dynamic_idle_controller.calculate_speed_increase(0.0F, delta_t, debug));
    const float slowest_motor_hz = 1000.0F/ 60.0F; // 1000 RPM
    TEST_ASSERT_EQUAL_FLOAT(0.0F, dynamic_idle_controller.calculate_speed_increase(slowest_motor_hz, delta_t, debug));
}

void test_dynamic_idle_controller_p_only()
{
    const float motor_hz500RPM = 500.0F/ 60.0F;
    const float motor_hz750RPM = 750.0F/ 60.0F;
    const float motor_hz1000RPM = 1000.0F/ 60.0F;
    const float motor_hz2000RPM = 2000.0F/ 60.0F;

    enum { TASK_INTERVAL_MICROSECONDS = 1000 };
    const dynamic_idle_controller_config_t dynamic_idle_controllerConfig = {
        .dyn_idle_min_rpm_100 = 10, // 10*100 = 1000 rpm
        .dyn_idle_p_gain = 50,
        .dyn_idle_i_gain = 0,
        .dyn_idle_d_gain = 0,
        .dyn_idle_max_increase = 150,
    };
    static Debug debug;
    static DynamicIdleController dynamic_idle_controller(TASK_INTERVAL_MICROSECONDS);
    dynamic_idle_controller.set_config(dynamic_idle_controllerConfig);
    const float delta_t = static_cast<float>(TASK_INTERVAL_MICROSECONDS) * 0.000001F;

    TEST_ASSERT_EQUAL(10, dynamic_idle_controller.get_config().dyn_idle_min_rpm_100);
    TEST_ASSERT_EQUAL_FLOAT(motor_hz1000RPM, dynamic_idle_controller.get_minimum_allowed_motor_hz());

    // slowest motor faster than 1000 RPM, so no speed increase
    TEST_ASSERT_EQUAL_FLOAT(0.0F, dynamic_idle_controller.calculate_speed_increase(motor_hz2000RPM, delta_t, debug));
    TEST_ASSERT_EQUAL_FLOAT(0.0F, dynamic_idle_controller.calculate_speed_increase(motor_hz1000RPM, delta_t, debug));

    // slowest motor slower than 1000 RPM, speed increase
    TEST_ASSERT_EQUAL_FLOAT(0.0625F, dynamic_idle_controller.calculate_speed_increase(motor_hz500RPM, delta_t, debug));
    TEST_ASSERT_EQUAL_FLOAT(0.0625F, dynamic_idle_controller.calculate_speed_increase(motor_hz500RPM, delta_t, debug));
    // half the speed difference from 1000, so half the output, since PID is P-Term only
    TEST_ASSERT_EQUAL_FLOAT(0.03125F, dynamic_idle_controller.calculate_speed_increase(motor_hz750RPM, delta_t, debug));
    TEST_ASSERT_EQUAL_FLOAT(0.03125F, dynamic_idle_controller.calculate_speed_increase(motor_hz750RPM, delta_t, debug));
}

// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-signed-bitwise,readability-magic-numbers)

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_dynamic_idle_controller);
    RUN_TEST(test_dynamic_idle_controller_p_only);

    UNITY_END();
}
