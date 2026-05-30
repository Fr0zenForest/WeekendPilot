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

void test_flaperon_combined_roll_and_flap() {
    wp::Mixer m; m.setAirframe(wp::Airframe::Flaperon);
    float src[static_cast<int>(wp::MixSource::Count)]; zero_src(src);
    src[(int)wp::MixSource::Roll] = 0.3f;
    src[(int)wp::MixSource::Flap] = 0.2f;
    uint16_t servo[wp::kNumServos];
    m.mix(src, servo);
    // servo0 = Roll(+1)*0.3 + Flap(+1)*0.2 = +0.5 -> 1750
    // servo4 = Roll(-1)*0.3 + Flap(+1)*0.2 = -0.1 -> 1450
    TEST_ASSERT_EQUAL_UINT16(1750, servo[0]);
    TEST_ASSERT_EQUAL_UINT16(1450, servo[4]);
}

void test_new_presets_throttle_channel_is_throttle() {
    // 确认三种预设的 servo2 都是油门口（src=0 -> 1000，而非 normToServoUs 的 1500）
    float src[static_cast<int>(wp::MixSource::Count)]; zero_src(src);
    uint16_t servo[wp::kNumServos];
    wp::Mixer m;
    m.setAirframe(wp::Airframe::Flaperon); m.mix(src, servo);
    TEST_ASSERT_EQUAL_UINT16(1000, servo[2]);
    m.setAirframe(wp::Airframe::VTail); m.mix(src, servo);
    TEST_ASSERT_EQUAL_UINT16(1000, servo[2]);
    m.setAirframe(wp::Airframe::Elevon); m.mix(src, servo);
    TEST_ASSERT_EQUAL_UINT16(1000, servo[2]);
}

void test_vtail_roll_maps_to_aileron() {
    wp::Mixer m; m.setAirframe(wp::Airframe::VTail);
    float src[static_cast<int>(wp::MixSource::Count)]; zero_src(src);
    src[(int)wp::MixSource::Roll] = 0.4f;
    uint16_t servo[wp::kNumServos];
    m.mix(src, servo);
    // V尾仍有独立副翼 servo0 = Roll(+1)*0.4 -> 1700；两片尾翼(1,3)不受 roll 影响
    TEST_ASSERT_EQUAL_UINT16(1700, servo[0]);
    TEST_ASSERT_EQUAL_UINT16(1500, servo[1]);
    TEST_ASSERT_EQUAL_UINT16(1500, servo[3]);
}

void test_elevon_yaw_maps_to_servo3() {
    wp::Mixer m; m.setAirframe(wp::Airframe::Elevon);
    float src[static_cast<int>(wp::MixSource::Count)]; zero_src(src);
    src[(int)wp::MixSource::Yaw] = 0.4f;
    uint16_t servo[wp::kNumServos];
    m.mix(src, servo);
    // servo3 = Yaw(+1)*0.4 -> 1700；两片升降副翼(0,1)不受 yaw 影响
    TEST_ASSERT_EQUAL_UINT16(1700, servo[3]);
    TEST_ASSERT_EQUAL_UINT16(1500, servo[0]);
    TEST_ASSERT_EQUAL_UINT16(1500, servo[1]);
}

void test_elevon_saturation_clamps_per_channel() {
    wp::Mixer m; m.setAirframe(wp::Airframe::Elevon);
    float src[static_cast<int>(wp::MixSource::Count)]; zero_src(src);
    src[(int)wp::MixSource::Roll]  = 0.8f;
    src[(int)wp::MixSource::Pitch] = 0.8f;
    // servo0 = +0.8 +0.8 = 1.6 -> clamp 1.0 -> 2000
    // servo1 = -0.8 +0.8 = 0.0 -> 1500
    uint16_t servo[wp::kNumServos];
    m.mix(src, servo);
    TEST_ASSERT_EQUAL_UINT16(2000, servo[0]);
    TEST_ASSERT_EQUAL_UINT16(1500, servo[1]);
}

void test_throttle_clamps_below_zero() {
    wp::Mixer m;  // Standard
    float src[static_cast<int>(wp::MixSource::Count)]; zero_src(src);
    src[(int)wp::MixSource::Throttle] = -0.5f;   // 异常负值
    uint16_t servo[wp::kNumServos];
    m.mix(src, servo);
    TEST_ASSERT_EQUAL_UINT16(1000, servo[2]);     // clamp 到 0 -> 1000
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
    RUN_TEST(test_flaperon_combined_roll_and_flap);
    RUN_TEST(test_new_presets_throttle_channel_is_throttle);
    RUN_TEST(test_vtail_roll_maps_to_aileron);
    RUN_TEST(test_elevon_yaw_maps_to_servo3);
    RUN_TEST(test_elevon_saturation_clamps_per_channel);
    RUN_TEST(test_throttle_clamps_below_zero);
    return UNITY_END();
}
