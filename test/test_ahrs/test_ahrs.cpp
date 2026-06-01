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

void test_accel_gate_skips_correction_when_out_of_range() {
    // Feed a constant body roll RATE with a high-magnitude accel (2g, as in a
    // pull-up / coordinated turn). With the gate, the accel must be IGNORED and
    // the gyro integrated freely, so roll grows. Without the gate, the high kp
    // correction from the 2g accel (normalized to same direction as 1g) pins
    // roll near level.
    // kp=20 is needed: at that gain the old code pins roll to ~3 deg, the new
    // gated code lets it reach ~30 deg. Default kp=1 is too weak to show the
    // difference (normalization removes magnitude info before correction).
    wp::AhrsMahony ahrs;
    ahrs.setKp(20.0f);
    wp::ImuSample s{};
    s.valid = true;
    s.gyro_x = 30.0f;                 // +30 deg/s roll rate
    s.accel_x = 0.0f; s.accel_y = 0.0f; s.accel_z = 2.0f;  // 2g -> |a|^2=4, out of [0.81,1.21]
    for (int i = 0; i < 1000; ++i) ahrs.update(s, 0.001f);  // 1s -> ~30deg if gyro free
    wp::Attitude a = ahrs.attitude();
    // gyro integrates ~30 deg; if accel correction were applied at kp=20 roll
    // would be pinned to ~3 deg.
    TEST_ASSERT_TRUE(a.roll_deg > 20.0f);
}

// ki=0(默认)：喂带恒定零偏的陀螺 + 水平加速度，姿态仍由 kp 主导收敛到水平。
void test_ki_zero_keeps_level_bounded() {
    wp::AhrsMahony ahrs;                 // 默认 two_ki_=0
    wp::ImuSample s = level_imu();
    s.gyro_x = 2.0f;                     // +2 deg/s 恒定零偏
    for (int i = 0; i < 5000; ++i) ahrs.update(s, 0.001f);  // 6DOF
    wp::Attitude a = ahrs.attitude();
    TEST_ASSERT_TRUE(a.roll_deg > -10.0f && a.roll_deg < 10.0f);
}

// ki>0：注入恒定陀螺零偏，零偏被 ifb_ 估出对消，姿态收敛到水平。
void test_ki_estimates_constant_bias() {
    wp::AhrsMahony ahrs;
    ahrs.setKi(2.0f * 0.1f);             // 开零偏估计
    wp::ImuSample s = level_imu();
    s.gyro_x = 2.0f; s.gyro_y = 1.0f;    // 恒定零偏 roll+2/pitch+1 deg/s
    for (int i = 0; i < 20000; ++i) ahrs.update(s, 0.001f);  // 20s 足够收敛
    wp::Attitude a = ahrs.attitude();
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, a.roll_deg);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, a.pitch_deg);
}

// 限幅判别性测试：ki 开 + 极小 bias limit(0.02) + 大零偏(10°/s=0.175rad/s 远超限幅)。
// 限幅生效时 ifb_ 饱和在 0.02、无法完全对消零偏，kp 平衡残差 -> 稳态 roll 明显非零(~18°)。
// 若限幅被移除，ifb_ 会涨到 ~0.173 完全对消零偏 -> roll≈0。故"roll 明显非零且有界"
// 这条断言只在限幅真正生效时成立——移除 clampf 此测试即失败（判别性）。
void test_bias_limit_clamps_prevents_full_cancel() {
    wp::AhrsMahony ahrs;
    ahrs.setKi(2.0f * 0.5f);
    ahrs.setBiasLimit(0.02f);
    wp::ImuSample s = level_imu();
    s.gyro_x = 10.0f;                    // 大恒定零偏，远超限幅
    for (int i = 0; i < 20000; ++i) ahrs.update(s, 0.001f);
    wp::Attitude a = ahrs.attitude();
    // 限幅生效 -> 零偏未被完全对消 -> 稳态 roll 明显非零(仿真 ~18°)
    TEST_ASSERT_TRUE(a.roll_deg > 10.0f);
    // 仍有界(不发散到接近 ±90)
    TEST_ASSERT_TRUE(a.roll_deg < 45.0f);
    TEST_ASSERT_TRUE(a.roll_deg == a.roll_deg);   // 非 NaN
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_level_converges_to_zero_roll_pitch);
    RUN_TEST(test_roll_right_accel_gives_positive_roll);
    RUN_TEST(test_invalid_mag_falls_back_to_6dof);
    RUN_TEST(test_valid_mag_holds_level);
    RUN_TEST(test_heading_locks_to_mag);
    RUN_TEST(test_accel_gate_skips_correction_when_out_of_range);
    RUN_TEST(test_ki_zero_keeps_level_bounded);
    RUN_TEST(test_ki_estimates_constant_bias);
    RUN_TEST(test_bias_limit_clamps_prevents_full_cancel);
    return UNITY_END();
}
