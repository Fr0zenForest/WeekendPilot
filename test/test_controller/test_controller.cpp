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

void test_standard_throttle_passthrough_equivalent() {
    wp::Controller c;
    wp::ControlInput in{}; fill_centered(in);
    in.channels[4] = 1500;          // Angle（启用 mixer 路径）
    in.channels[2] = 1700;          // throttle
    wp::ServoCommand out = c.update(in);
    TEST_ASSERT_EQUAL_UINT16(1700, out.servo[2]);   // 1000 + 0.7*1000
}

void test_peripheral_passthrough_overrides_servo() {
    wp::ControllerConfig cfg;
    cfg.peripherals[0] = wp::PeripheralMap{5, 7, true};  // servo5 <- ch8 裸值
    wp::Controller c; c.setConfig(cfg);
    wp::ControlInput in{}; fill_centered(in);
    in.channels[4] = 1500;          // Angle
    in.channels[7] = 1234;          // 起落架开关位置
    wp::ServoCommand out = c.update(in);
    TEST_ASSERT_EQUAL_UINT16(1234, out.servo[5]);
}

void test_glimit_softens_pull_in_high_g() {
    wp::Controller c;
    wp::ControlInput in{}; fill_centered(in);
    in.channels[4] = 1500;          // Angle
    in.channels[5] = 0;             // gain 0 -> 纯手动，隔离 PID 影响
    in.channels[1] = 2000;          // 升降满拉 -> pitch_cmd = +1
    in.imu.accel_z = 10.0f;         // 硬限 -> 拉杆贡献清零
    wp::ServoCommand out = c.update(in);
    // pitch 需求被清零 -> 升降回中位
    TEST_ASSERT_EQUAL_UINT16(1500, out.servo[1]);
}

void test_flaperon_airframe_routes_flap_to_both_ailerons() {
    wp::ControllerConfig cfg;
    cfg.airframe = wp::Airframe::Flaperon;
    cfg.flap_enabled = true;        // 启用襟翼需求
    cfg.gain_channel = 5;
    wp::Controller c; c.setConfig(cfg);
    wp::ControlInput in{}; fill_centered(in);
    in.channels[4] = 1500;          // Angle
    in.channels[5] = 0;             // gain 0 -> 隔离 PID，纯看混控
    in.channels[6] = 2000;          // flap 通道满 -> flap 需求 = 1.0
    wp::ServoCommand out = c.update(in);
    // Flaperon: servo0 += Flap(+1)*1.0, servo4 += Flap(+1)*1.0 -> 两片副翼同向到满
    // 摇杆居中 roll=0，所以 servo0 = Flap 1.0 -> 2000；servo4 = -roll*0 + flap 1.0 -> 2000
    TEST_ASSERT_EQUAL_UINT16(2000, out.servo[0]);
    TEST_ASSERT_EQUAL_UINT16(2000, out.servo[4]);
}

void test_flap_disabled_keeps_flap_demand_zero() {
    wp::ControllerConfig cfg;
    cfg.airframe = wp::Airframe::Flaperon;
    cfg.flap_enabled = false;       // 关闭襟翼需求
    cfg.gain_channel = 5;
    wp::Controller c; c.setConfig(cfg);
    wp::ControlInput in{}; fill_centered(in);
    in.channels[4] = 1500;          // Angle
    in.channels[5] = 0;             // gain 0
    in.channels[6] = 2000;          // flap 通道满，但 flap_enabled=false 应被忽略
    wp::ServoCommand out = c.update(in);
    // flap 需求被强制 0，roll 居中 -> servo0/servo4 都回中位 1500
    TEST_ASSERT_EQUAL_UINT16(1500, out.servo[0]);
    TEST_ASSERT_EQUAL_UINT16(1500, out.servo[4]);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_off_mode_is_passthrough);
    RUN_TEST(test_link_lost_forces_passthrough);
    RUN_TEST(test_angle_mode_banked_adds_correction);
    RUN_TEST(test_standard_throttle_passthrough_equivalent);
    RUN_TEST(test_peripheral_passthrough_overrides_servo);
    RUN_TEST(test_glimit_softens_pull_in_high_g);
    RUN_TEST(test_flaperon_airframe_routes_flap_to_both_ailerons);
    RUN_TEST(test_flap_disabled_keeps_flap_demand_zero);
    return UNITY_END();
}
