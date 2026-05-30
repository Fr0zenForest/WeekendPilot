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

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_below_soft_limit_unchanged);
    RUN_TEST(test_midway_linear_attenuation);
    RUN_TEST(test_at_hard_limit_zeroed);
    RUN_TEST(test_beyond_hard_limit_zeroed);
    RUN_TEST(test_push_stick_not_limited);
    RUN_TEST(test_disabled_passes_through);
    return UNITY_END();
}
