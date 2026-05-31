#include <unity.h>
#include "controller.h"
#include "sensors/sensor_frontend.h"
#include "blackbox/blackbox_sink.h"
#include "blackbox/blackbox_frame.h"

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

void test_standard_throttle_routes_through_mixer() {
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
    in.channels[5] = 1000;          // gain 0 (合法 CRSF 下限) -> 纯手动，隔离 PID 影响
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

void test_update_from_bundle_matches_control_input() {
    // 同样的传感器数 + 通道，updateFromBundle 与手填 ControlInput 应得到相同 servo
    wp::Controller a, b;
    wp::SensorBundle bundle{};
    bundle.imu.gyro_x = 5.0f; bundle.imu.accel_z = 1.0f; bundle.imu.valid = true;

    uint16_t ch[wp::kNumChannels];
    for (int i = 0; i < wp::kNumChannels; ++i) ch[i] = 1500;
    float dt = 0.002f;

    wp::ServoCommand viaBundle = a.updateFromBundle(ch, bundle, dt, true);

    wp::ControlInput in{};
    for (int i = 0; i < wp::kNumChannels; ++i) in.channels[i] = ch[i];
    in.imu = bundle.imu; in.mag = bundle.mag; in.baro = bundle.baro;
    in.dt = dt; in.link_ok = true;
    wp::ServoCommand viaInput = b.update(in);

    for (int i = 0; i < wp::kNumServos; ++i)
        TEST_ASSERT_EQUAL_UINT16(viaInput.servo[i], viaBundle.servo[i]);
}

void test_update_from_bundle_matches_on_invalid_imu() {
    // imu 失效时 update 走直通；两路仍应逐字节相等
    wp::Controller a, b;
    wp::SensorBundle bundle{};
    bundle.imu.valid = false;   // 失效

    uint16_t ch[wp::kNumChannels];
    for (int i = 0; i < wp::kNumChannels; ++i) ch[i] = 1500;
    ch[0] = 1600; ch[1] = 1400;   // 给几个非中位值，确保直通可区分
    float dt = 0.002f;

    wp::ServoCommand viaBundle = a.updateFromBundle(ch, bundle, dt, true);

    wp::ControlInput in{};
    for (int i = 0; i < wp::kNumChannels; ++i) in.channels[i] = ch[i];
    in.imu = bundle.imu; in.mag = bundle.mag; in.baro = bundle.baro;
    in.dt = dt; in.link_ok = true;
    wp::ServoCommand viaInput = b.update(in);

    for (int i = 0; i < wp::kNumServos; ++i)
        TEST_ASSERT_EQUAL_UINT16(viaInput.servo[i], viaBundle.servo[i]);
}

void test_althold_drives_elevator_when_below_target() {
    wp::Controller c;
    wp::ControllerConfig cfg;
    cfg.althold_enabled = true;
    cfg.althold_channel = 7;          // ch8 作定高开关
    c.setConfig(cfg);

    wp::ControlInput in{};
    for (int i = 0; i < wp::kNumChannels; ++i) in.channels[i] = 1500;
    in.channels[cfg.mode_channel] = 1500;     // Angle
    in.channels[cfg.gain_channel] = 2000;     // gain 100%
    in.channels[cfg.althold_channel] = 1000;  // 定高先关
    in.dt = 0.02f; in.link_ok = true;
    in.imu.valid = true; in.imu.accel_z = 1.0f;
    in.baro.valid = true; in.baro.altitude_m = 100.0f;

    // 关定高跑一拍，记录基准 elevator
    wp::ServoCommand off = c.update(in);

    // 开定高，锁 100m；随后飞机掉到 90m
    in.channels[cfg.althold_channel] = 2000;
    c.update(in);                              // 接管，锁 100m
    in.baro.altitude_m = 90.0f;
    wp::ServoCommand on = c.update(in);

    // 低于目标 -> 抬头修正 -> 升降舵向抬头方向偏（servo[1] 高于基准）。
    TEST_ASSERT_TRUE(on.servo[1] > off.servo[1]);
}

// 定高通道未拨上 -> 不接管：两台同样喂入的控制器（一台高度恒定、一台高度大变）
// 升降输出应逐字节相同。两台独立 Controller 各自从同一初态推进，隔离 AHRS 积分漂移。
void test_althold_inactive_when_channel_low() {
    auto make = [](){
        wp::Controller c;
        wp::ControllerConfig cfg;
        cfg.althold_enabled = true;
        cfg.althold_channel = 7;
        c.setConfig(cfg);
        return c;
    };
    wp::Controller c_const = make();   // 高度恒定 100m
    wp::Controller c_vary  = make();   // 高度跌到 50m
    wp::ControlInput in{};
    for (int i = 0; i < wp::kNumChannels; ++i) in.channels[i] = 1500;
    in.channels[4] = 1500;                    // Angle
    in.channels[7] = 1000;                    // 定高通道关
    in.dt = 0.02f; in.link_ok = true;
    in.imu.valid = true; in.imu.accel_z = 1.0f;

    // 第 1 拍两台完全相同（都在 100m）
    in.baro.valid = true; in.baro.altitude_m = 100.0f;
    c_const.update(in);
    c_vary.update(in);
    // 第 2 拍：c_const 仍 100m，c_vary 跌到 50m；通道关 -> 定高不接管
    in.baro.altitude_m = 100.0f; wp::ServoCommand a = c_const.update(in);
    in.baro.altitude_m = 50.0f;  wp::ServoCommand b = c_vary.update(in);
    TEST_ASSERT_EQUAL_UINT16(a.servo[1], b.servo[1]);  // 高度变化不影响升降（未接管）
}

// 黑匣子使能 + 注入 sink -> update() 落帧；解出的 mode 与输入模式一致。
void test_blackbox_records_on_update() {
    static uint8_t bb_buf[4096];
    wp::RingBufferSink sink(bb_buf, sizeof(bb_buf));
    wp::Controller c;
    wp::ControllerConfig cfg;
    cfg.blackbox.enabled = true;
    cfg.blackbox.decimation = 1;     // 每拍都记
    c.setConfig(cfg);
    c.attachBlackboxSink(&sink);

    wp::ControlInput in{}; fill_centered(in);
    in.channels[4] = 1500;           // Angle
    in.baro.valid = true; in.baro.altitude_m = 50.0f;
    c.update(in);

    TEST_ASSERT_EQUAL_INT(wp::kFrameBytes, (int)sink.usedBytes());
    wp::BlackboxFrame g{};
    wp::decodeFrame(bb_buf, g);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)wp::FlightMode::Angle, g.mode);
    TEST_ASSERT_FLOAT_WITHIN(1e-2, 50.0f, g.baro_alt_m);
}

// 黑匣子默认关 -> 不记录。显式置 link_ok/imu/baro 有效并给 Angle 模式，
// 确保越过早返回、真正抵达 if(cfg_.blackbox.enabled) 守卫并正确跳过（而非因早返回侥幸为 0）。
void test_blackbox_disabled_by_default() {
    static uint8_t bb_buf[256];
    wp::RingBufferSink sink(bb_buf, sizeof(bb_buf));
    wp::Controller c;                // 默认 cfg：blackbox.enabled=false
    c.attachBlackboxSink(&sink);
    wp::ControlInput in{}; fill_centered(in);
    in.channels[4] = 1500;           // Angle（越过 Off 早返回）
    in.link_ok = true; in.imu.valid = true;
    in.baro.valid = true; in.baro.altitude_m = 50.0f;
    c.update(in);
    TEST_ASSERT_EQUAL_INT(0, (int)sink.usedBytes());
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_off_mode_is_passthrough);
    RUN_TEST(test_link_lost_forces_passthrough);
    RUN_TEST(test_angle_mode_banked_adds_correction);
    RUN_TEST(test_standard_throttle_routes_through_mixer);
    RUN_TEST(test_peripheral_passthrough_overrides_servo);
    RUN_TEST(test_glimit_softens_pull_in_high_g);
    RUN_TEST(test_flaperon_airframe_routes_flap_to_both_ailerons);
    RUN_TEST(test_flap_disabled_keeps_flap_demand_zero);
    RUN_TEST(test_update_from_bundle_matches_control_input);
    RUN_TEST(test_update_from_bundle_matches_on_invalid_imu);
    RUN_TEST(test_althold_drives_elevator_when_below_target);
    RUN_TEST(test_althold_inactive_when_channel_low);
    RUN_TEST(test_blackbox_records_on_update);
    RUN_TEST(test_blackbox_disabled_by_default);
    return UNITY_END();
}
