#include <unity.h>
#include <cmath>
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

// Build a body-frame magnetometer reading from a known attitude, using the
// SAME convention the SITL bridge uses: C_bn = Rx(phi)Ry(theta)Rz(psi),
// earth field normalized, inclination 60deg northern hemisphere.
static wp::MagSample synth_mag(float roll_deg, float pitch_deg, float yaw_deg) {
    const float D2R = 0.017453292519943295f;
    const float phi = roll_deg * D2R, th = pitch_deg * D2R, psi = yaw_deg * D2R;
    const float incl = 60.0f * D2R;
    const float Bn = std::cos(incl), Be = 0.0f, Bd = std::sin(incl);
    const float cph = std::cos(phi), sph = std::sin(phi);
    const float cth = std::cos(th),  sth = std::sin(th);
    const float cps = std::cos(psi), sps = std::sin(psi);
    // Rz(psi)
    const float x1 =  cps * Bn + sps * Be;
    const float y1 = -sps * Bn + cps * Be;
    const float z1 =  Bd;
    // Ry(theta)
    const float x2 = cth * x1 - sth * z1;
    const float y2 = y1;
    const float z2 = sth * x1 + cth * z1;
    // Rx(phi)
    wp::MagSample m{};
    m.mag_x = x2;
    m.mag_y = cph * y2 + sph * z2;
    m.mag_z = -sph * y2 + cph * z2;
    m.valid = true;
    return m;
}

void test_level_converges_to_zero_roll_pitch() {
    wp::AhrsMahony ahrs;
    wp::ImuSample s = level_imu();
    for (int i = 0; i < 5000; ++i) ahrs.update(s, 0.001f);  // 6DOF path
    wp::Attitude a = ahrs.attitude();
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, a.roll_deg);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, a.pitch_deg);
}

void test_roll_right_accel_gives_positive_roll() {
    wp::AhrsMahony ahrs;
    wp::ImuSample s{};
    s.valid = true;
    s.accel_x = 0.0f; s.accel_y = 0.5f; s.accel_z = 0.866f;
    for (int i = 0; i < 5000; ++i) ahrs.update(s, 0.001f);
    wp::Attitude a = ahrs.attitude();
    TEST_ASSERT_TRUE(a.roll_deg > 20.0f && a.roll_deg < 40.0f);
}

void test_invalid_mag_falls_back_to_6dof() {
    wp::AhrsMahony ahrs;
    wp::ImuSample s = level_imu();
    wp::MagSample m{};  // valid=false
    for (int i = 0; i < 5000; ++i) ahrs.update(s, m, 0.001f);
    wp::Attitude a = ahrs.attitude();
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, a.roll_deg);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, a.pitch_deg);
}

void test_valid_mag_holds_level() {
    wp::AhrsMahony ahrs;
    wp::ImuSample s = level_imu();
    wp::MagSample m = synth_mag(0.0f, 0.0f, 0.0f);
    for (int i = 0; i < 5000; ++i) ahrs.update(s, m, 0.001f);
    wp::Attitude a = ahrs.attitude();
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, a.roll_deg);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, a.pitch_deg);
}

void test_heading_locks_to_mag() {
    wp::AhrsMahony ahrs;
    ahrs.setKp(10.0f);  // higher kp needed: heading channel converges slower than tilt
    wp::ImuSample s = level_imu();
    wp::MagSample m = synth_mag(0.0f, 0.0f, 30.0f);  // level, heading +30
    for (int i = 0; i < 8000; ++i) ahrs.update(s, m, 0.001f);
    wp::Attitude a = ahrs.attitude();
    TEST_ASSERT_FLOAT_WITHIN(5.0f, 30.0f, a.yaw_deg);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_level_converges_to_zero_roll_pitch);
    RUN_TEST(test_roll_right_accel_gives_positive_roll);
    RUN_TEST(test_invalid_mag_falls_back_to_6dof);
    RUN_TEST(test_valid_mag_holds_level);
    RUN_TEST(test_heading_locks_to_mag);
    return UNITY_END();
}
