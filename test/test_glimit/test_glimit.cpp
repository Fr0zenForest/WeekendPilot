#include <unity.h>
#include "safety/glimit.h"

void setUp() {}
void tearDown() {}

void test_below_soft_limit_unchanged() {
    wp::GLimitConfig cfg;   // soft 6, hard 10
    float out = wp::applyGLimit(0.8f, 3.0f, cfg);   // 3G < 6G
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.8f, out);
}

void test_midway_linear_attenuation() {
    wp::GLimitConfig cfg;
    // 8G：k = 1 - (8-6)/(10-6) = 0.5
    float out = wp::applyGLimit(0.8f, 8.0f, cfg);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.4f, out);
}

void test_at_hard_limit_zeroed() {
    wp::GLimitConfig cfg;
    float out = wp::applyGLimit(0.8f, 10.0f, cfg);   // k=0
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, out);
}

void test_beyond_hard_limit_zeroed() {
    wp::GLimitConfig cfg;
    float out = wp::applyGLimit(0.8f, 15.0f, cfg);   // k clamp 0
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, out);
}

void test_push_stick_not_limited() {
    wp::GLimitConfig cfg;
    // 推杆（负 pitch 需求）即使在高 G 也不限（高 G 来自别处）
    float out = wp::applyGLimit(-0.8f, 9.0f, cfg);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -0.8f, out);
}

void test_disabled_passes_through() {
    wp::GLimitConfig cfg; cfg.enabled = false;
    float out = wp::applyGLimit(0.8f, 12.0f, cfg);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.8f, out);
}

void test_inverted_convention_attenuates_negative_pull() {
    // 装反约定：pitch_loads_positive=false -> 拉杆是负 pitch，加载对应负 accel_z
    wp::GLimitConfig cfg; cfg.pitch_loads_positive = false;
    // g = -accel_z = 8 ; pulling = (demand<0) = true ; k = 1-(8-6)/4 = 0.5
    float out = wp::applyGLimit(-0.8f, -8.0f, cfg);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -0.4f, out);
    // 同约定下，正 pitch（此时是卸载方向）不被限制
    float out2 = wp::applyGLimit(0.8f, -8.0f, cfg);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.8f, out2);
}

void test_misconfigured_span_zeroes_pull() {
    // hard_g <= soft_g 误配：span<=0 -> k=0，超软限即清零拉杆（失效偏保守）
    wp::GLimitConfig cfg; cfg.soft_g = 6.0f; cfg.hard_g = 6.0f;
    float out = wp::applyGLimit(0.8f, 9.0f, cfg);   // g>soft, pulling, span=0 -> k=0
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, out);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_below_soft_limit_unchanged);
    RUN_TEST(test_midway_linear_attenuation);
    RUN_TEST(test_at_hard_limit_zeroed);
    RUN_TEST(test_beyond_hard_limit_zeroed);
    RUN_TEST(test_push_stick_not_limited);
    RUN_TEST(test_disabled_passes_through);
    RUN_TEST(test_inverted_convention_attenuates_negative_pull);
    RUN_TEST(test_misconfigured_span_zeroes_pull);
    return UNITY_END();
}
