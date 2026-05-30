#include <unity.h>
#include "pid/pid_controller.h"

void setUp() {}
void tearDown() {}

void test_p_only_proportional_to_error() {
    wp::PidController pid;
    pid.setGains({0.01f, 0.0f, 0.0f});
    // error = 50 - 0 = 50; gyro 0; out = 0.01*50 = 0.5
    float out = pid.update(50.0f, 0.0f, 0.0f, 0.001f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.5f, out);
}

void test_d_opposes_gyro_motion() {
    wp::PidController pid;
    pid.setGains({0.0f, 0.0f, 0.01f});
    // error 0; gyro +100 dps; D = -kd*gyro = -1.0 -> clamp to -1
    float out = pid.update(0.0f, 0.0f, 100.0f, 0.001f);
    TEST_ASSERT_TRUE(out < 0.0f);
}

void test_i_accumulates_and_ki_scales_live() {
    wp::PidController pid;
    pid.setGains({0.0f, 1.0f, 0.0f});
    for (int i = 0; i < 10; ++i) pid.update(1.0f, 0.0f, 0.0f, 0.1f);  // 1s of error=1
    float raw = pid.integralRaw();
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 1.0f, raw);   // integral_raw ~ error*time = 1
    // set ki=0 -> I contribution zero but raw retained
    pid.setGains({0.0f, 0.0f, 0.0f});
    float out = pid.update(0.0f, 0.0f, 0.0f, 0.001f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, out);
    TEST_ASSERT_TRUE(pid.integralRaw() > 0.5f);   // raw still there
}

void test_back_calc_antiwindup_caps_raw() {
    wp::PidController pid;
    pid.setGains({0.0f, 1.0f, 0.0f});
    for (int i = 0; i < 1000; ++i) pid.update(10.0f, 0.0f, 0.0f, 0.1f);  // huge sustained error
    float out = pid.update(10.0f, 0.0f, 0.0f, 0.1f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.0f, out);   // output clamped at +1
    // integral_raw should not exceed what maps to the limit (ki=1 -> raw<=1)
    TEST_ASSERT_TRUE(pid.integralRaw() <= 1.0f + 1e-3f);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_p_only_proportional_to_error);
    RUN_TEST(test_d_opposes_gyro_motion);
    RUN_TEST(test_i_accumulates_and_ki_scales_live);
    RUN_TEST(test_back_calc_antiwindup_caps_raw);
    return UNITY_END();
}
