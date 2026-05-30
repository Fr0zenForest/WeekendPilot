#include <unity.h>
#include "mixer/mixer.h"

void setUp() {}
void tearDown() {}

static void zero_src(float s[5]) { for (int i = 0; i < 5; ++i) s[i] = 0.0f; }

void test_standard_centered_outputs_1500() {
    wp::Mixer m;  // 默认 Standard
    float src[5]; zero_src(src);
    uint16_t servo[wp::kNumServos];
    m.mix(src, servo);
    TEST_ASSERT_EQUAL_UINT16(1500, servo[0]);  // 副翼
    TEST_ASSERT_EQUAL_UINT16(1500, servo[1]);  // 升降
    TEST_ASSERT_EQUAL_UINT16(1000, servo[2]);  // 油门 src=0 -> 1000
    TEST_ASSERT_EQUAL_UINT16(1500, servo[3]);  // 方向
}

void test_standard_roll_maps_to_aileron() {
    wp::Mixer m;
    float src[5]; zero_src(src);
    src[(int)wp::MixSource::Roll] = 0.5f;       // 半舵右
    uint16_t servo[wp::kNumServos];
    m.mix(src, servo);
    TEST_ASSERT_EQUAL_UINT16(1750, servo[0]);   // 1500 + 0.5*500
    TEST_ASSERT_EQUAL_UINT16(1500, servo[1]);
    TEST_ASSERT_EQUAL_UINT16(1500, servo[3]);
}

void test_standard_throttle_full() {
    wp::Mixer m;
    float src[5]; zero_src(src);
    src[(int)wp::MixSource::Throttle] = 1.0f;
    uint16_t servo[wp::kNumServos];
    m.mix(src, servo);
    TEST_ASSERT_EQUAL_UINT16(2000, servo[2]);   // 1000 + 1.0*1000
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_standard_centered_outputs_1500);
    RUN_TEST(test_standard_roll_maps_to_aileron);
    RUN_TEST(test_standard_throttle_full);
    return UNITY_END();
}
