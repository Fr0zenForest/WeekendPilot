#include <unity.h>
#include "modes/mode_controller.h"

void setUp() {}
void tearDown() {}

static wp::ImuSample zero_imu() { wp::ImuSample s{}; s.valid = true; s.accel_z = 1.0f; return s; }

void test_off_mode_zero_correction() {
    wp::ModeController mc;
    wp::Attitude att{}; att.roll_deg = 20.0f;
    auto c = mc.update(wp::FlightMode::Off, 0,0,0, att, zero_imu(), 0.01f, false);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, c.roll);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, c.pitch);
}

void test_angle_mode_corrects_toward_level() {
    wp::ModeController mc;
    wp::Attitude att{}; att.roll_deg = 30.0f;   // banked right, stick centered
    wp::ImuSample imu = zero_imu();
    wp::StabCorrection c;
    for (int i = 0; i < 100; ++i)   // let blend settle
        c = mc.update(wp::FlightMode::Angle, 0,0,0, att, imu, 0.01f, false);
    // error = target(0) - 30 = -30 -> negative roll correction (toward level)
    TEST_ASSERT_TRUE(c.roll < 0.0f);
}

void test_rate_mode_opposes_rotation() {
    wp::ModeController mc;
    wp::Attitude att{};
    wp::ImuSample imu = zero_imu();
    imu.gyro_x = 50.0f;   // rolling right, stick centered -> target rate 0
    wp::StabCorrection c;
    for (int i = 0; i < 100; ++i)
        c = mc.update(wp::FlightMode::Rate, 0,0,0, att, imu, 0.01f, false);
    TEST_ASSERT_TRUE(c.roll < 0.0f);   // opposes the +50dps roll
}

void test_yaw_is_rate_controlled_in_angle_mode() {
    wp::ModeController mc;
    wp::Attitude att{};                 // level
    wp::ImuSample imu = zero_imu();
    imu.gyro_z = 50.0f;                  // yawing right, yaw stick centered -> target rate 0
    wp::StabCorrection c;
    for (int i = 0; i < 100; ++i)        // settle blend; mode stays Angle throughout
        c = mc.update(wp::FlightMode::Angle, 0, 0, 0, att, imu, 0.01f, false);
    // yaw PID runs even in Angle mode and opposes the +50 dps yaw rate
    TEST_ASSERT_TRUE(c.yaw < 0.0f);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_off_mode_zero_correction);
    RUN_TEST(test_angle_mode_corrects_toward_level);
    RUN_TEST(test_rate_mode_opposes_rotation);
    RUN_TEST(test_yaw_is_rate_controlled_in_angle_mode);
    return UNITY_END();
}
