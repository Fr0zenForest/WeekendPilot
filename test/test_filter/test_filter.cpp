#include <unity.h>
#include "math/filter_pt1.h"

void setUp() {}
void tearDown() {}

void test_pt1_converges_to_step() {
    wp::FilterPt1 f(10.0f);   // 10Hz cutoff
    float y = 0.0f;
    for (int i = 0; i < 1000; ++i) y = f.apply(1.0f, 0.001f);  // 1s @ 1kHz
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, y);   // settles near step value
}

void test_pt1_first_sample_attenuated() {
    wp::FilterPt1 f(10.0f);
    float y = f.apply(1.0f, 0.001f);
    TEST_ASSERT_TRUE(y > 0.0f && y < 0.2f);     // one 1ms sample: small fraction
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_pt1_converges_to_step);
    RUN_TEST(test_pt1_first_sample_attenuated);
    return UNITY_END();
}
