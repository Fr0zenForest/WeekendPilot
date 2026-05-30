#include <unity.h>
#include "ahrs/ahrs_mahony.h"

void setUp() {}
void tearDown() {}

static wp::ImuSample level_imu() {
    wp::ImuSample s{};
    s.gyro_x = s.gyro_y = s.gyro_z = 0.0f;
    s.accel_x = 0.0f; s.accel_y = 0.0f; s.accel_z = 1.0f;  // 1g down -> level
    s.valid = true;
    return s;
}

void test_level_converges_to_zero_roll_pitch() {
    wp::AhrsMahony ahrs;
    wp::ImuSample s = level_imu();
    for (int i = 0; i < 5000; ++i) ahrs.update(s, 0.001f);  // 5s settle
    wp::Attitude a = ahrs.attitude();
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, a.roll_deg);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, a.pitch_deg);
}

void test_roll_right_accel_gives_positive_roll() {
    wp::AhrsMahony ahrs;
    wp::ImuSample s{};
    s.valid = true;
    // banked right ~30deg: gravity vector tilts -> ay component
    s.accel_x = 0.0f; s.accel_y = 0.5f; s.accel_z = 0.866f;
    for (int i = 0; i < 5000; ++i) ahrs.update(s, 0.001f);
    wp::Attitude a = ahrs.attitude();
    TEST_ASSERT_TRUE(a.roll_deg > 20.0f && a.roll_deg < 40.0f);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_level_converges_to_zero_roll_pitch);
    RUN_TEST(test_roll_right_accel_gives_positive_roll);
    return UNITY_END();
}
