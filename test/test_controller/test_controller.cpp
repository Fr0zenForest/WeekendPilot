#include <unity.h>
#include "controller.h"

void setUp() {}
void tearDown() {}

static void fill_centered(wp::ControlInput& in) {
    for (int i = 0; i < wp::kNumChannels; ++i) in.channels[i] = 1500;
    in.channels[2] = 1700;          // throttle up (not ground-frozen)
    in.dt = 0.01f; in.link_ok = true;
    in.imu.valid = true; in.imu.accel_z = 1.0f;
}

void test_off_mode_is_passthrough() {
    wp::Controller c;
    wp::ControlInput in{}; fill_centered(in);
    in.channels[4] = 1000;          // mode Off
    in.channels[0] = 1650;          // roll stick
    wp::ServoCommand out = c.update(in);
    TEST_ASSERT_EQUAL_UINT16(1650, out.servo[0]);   // aileron passthrough
}

void test_link_lost_forces_passthrough() {
    wp::Controller c;
    wp::ControlInput in{}; fill_centered(in);
    in.channels[4] = 1500;          // Angle
    in.link_ok = false;
    in.channels[0] = 1650;
    wp::ServoCommand out = c.update(in);
    TEST_ASSERT_EQUAL_UINT16(1650, out.servo[0]);
}

void test_angle_mode_banked_adds_correction() {
    wp::Controller c;
    wp::ControlInput in{}; fill_centered(in);
    in.channels[4] = 1500;          // Angle
    in.channels[5] = 2000;          // gain 100%
    in.imu.accel_y = 0.5f; in.imu.accel_z = 0.866f;  // banked right
    wp::ServoCommand out{};
    for (int i = 0; i < 300; ++i) out = c.update(in);   // settle AHRS + blend
    // banked right, stick centered -> aileron should move away from 1500
    TEST_ASSERT_TRUE(out.servo[0] != 1500);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_off_mode_is_passthrough);
    RUN_TEST(test_link_lost_forces_passthrough);
    RUN_TEST(test_angle_mode_banked_adds_correction);
    return UNITY_END();
}
