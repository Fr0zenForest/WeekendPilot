#include <unity.h>
#include "mixer/mixer.h"

void setUp() {}
void tearDown() {}

static void zero_src(float s[static_cast<int>(wp::MixSource::Count)]) {
    for (int i = 0; i < static_cast<int>(wp::MixSource::Count); ++i) s[i] = 0.0f;
}

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

void test_vtail_pitch_both_tails_same_dir() {
    wp::Mixer m; m.setAirframe(wp::Airframe::VTail);
    float src[5]; zero_src(src);
    src[(int)wp::MixSource::Pitch] = 0.4f;     // 拉升降
    uint16_t servo[wp::kNumServos];
    m.mix(src, servo);
    // 两片 V 尾 (servo1, servo3) 同向偏转 = 升降
    TEST_ASSERT_EQUAL_UINT16(1700, servo[1]);  // 1500 + 0.4*500
    TEST_ASSERT_EQUAL_UINT16(1700, servo[3]);
}

void test_vtail_yaw_tails_opposite() {
    wp::Mixer m; m.setAirframe(wp::Airframe::VTail);
    float src[5]; zero_src(src);
    src[(int)wp::MixSource::Yaw] = 0.4f;       // 方向
    uint16_t servo[wp::kNumServos];
    m.mix(src, servo);
    // 两片 V 尾差动 = 方向：一片 +，一片 -
    TEST_ASSERT_EQUAL_UINT16(1700, servo[1]);  // +0.4
    TEST_ASSERT_EQUAL_UINT16(1300, servo[3]);  // -0.4
}

void test_elevon_roll_opposite_pitch_same() {
    wp::Mixer m; m.setAirframe(wp::Airframe::Elevon);
    float src[5]; zero_src(src);
    // 纯 roll：两片副翼差动
    src[(int)wp::MixSource::Roll] = 0.4f;
    uint16_t servo[wp::kNumServos];
    m.mix(src, servo);
    TEST_ASSERT_EQUAL_UINT16(1700, servo[0]);  // +0.4 roll
    TEST_ASSERT_EQUAL_UINT16(1300, servo[1]);  // -0.4 roll
    // 纯 pitch：两片同向
    zero_src(src);
    src[(int)wp::MixSource::Pitch] = 0.4f;
    m.mix(src, servo);
    TEST_ASSERT_EQUAL_UINT16(1700, servo[0]);  // +0.4 pitch
    TEST_ASSERT_EQUAL_UINT16(1700, servo[1]);  // +0.4 pitch
}

void test_flaperon_flap_both_ailerons_same() {
    wp::Mixer m; m.setAirframe(wp::Airframe::Flaperon);
    float src[5]; zero_src(src);
    src[(int)wp::MixSource::Flap] = 0.4f;      // 放襟翼
    uint16_t servo[wp::kNumServos];
    m.mix(src, servo);
    // 左右副翼 (servo0, servo4) 同向下垂当襟翼。
    // servo0: +0.4 flap ; servo4: roll(-1)*0 + flap(+1)*0.4 = +0.4
    TEST_ASSERT_EQUAL_UINT16(1700, servo[0]);
    TEST_ASSERT_EQUAL_UINT16(1700, servo[4]);
}

void test_flaperon_roll_ailerons_differential() {
    wp::Mixer m; m.setAirframe(wp::Airframe::Flaperon);
    float src[5]; zero_src(src);
    src[(int)wp::MixSource::Roll] = 0.4f;
    uint16_t servo[wp::kNumServos];
    m.mix(src, servo);
    // servo0: roll(+1)*0.4 = +0.4 ; servo4: roll(-1)*0.4 = -0.4 -> 差动滚转
    TEST_ASSERT_EQUAL_UINT16(1700, servo[0]);
    TEST_ASSERT_EQUAL_UINT16(1300, servo[4]);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_standard_centered_outputs_1500);
    RUN_TEST(test_standard_roll_maps_to_aileron);
    RUN_TEST(test_standard_throttle_full);
    RUN_TEST(test_vtail_pitch_both_tails_same_dir);
    RUN_TEST(test_vtail_yaw_tails_opposite);
    RUN_TEST(test_elevon_roll_opposite_pitch_same);
    RUN_TEST(test_flaperon_flap_both_ailerons_same);
    RUN_TEST(test_flaperon_roll_ailerons_differential);
    return UNITY_END();
}
