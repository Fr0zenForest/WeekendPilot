#include "unity.h"
#include "nav/altitude_hold.h"
using namespace wp;

void setUp() {} void tearDown() {}

// WARNING 新代码（级联参考 INAV 固定翼高度环，控制律手写）。
//    单元测试为逻辑/数学验证；闭环见 SITL check_alt_hold.py（JSBSim 真值）。未经实物气压计实测。

// 未请求接管 -> 返回 false，不接管
void test_not_engaged_when_no_request() {
    AltitudeHold ah;
    float pc = -99.0f;
    bool drive = ah.update(/*engage*/false, /*baro_valid*/true, /*alt*/100.0f,
                           /*pilot_pitch*/0.0f, /*dt*/0.02f, pc);
    TEST_ASSERT_FALSE(drive);
    TEST_ASSERT_FALSE(ah.engaged());
}

// 气压无效 -> failsafe 脱离
void test_disengage_when_baro_invalid() {
    AltitudeHold ah;
    float pc = 0.0f;
    bool drive = ah.update(true, /*baro_valid*/false, 100.0f, 0.0f, 0.02f, pc);
    TEST_ASSERT_FALSE(drive);
    TEST_ASSERT_FALSE(ah.engaged());
}

// 接管起始沿：锁定当前高度为目标
void test_engage_latches_target() {
    AltitudeHold ah;
    float pc = 0.0f;
    bool drive = ah.update(true, true, /*alt*/123.5f, 0.0f, 0.02f, pc);
    TEST_ASSERT_TRUE(drive);
    TEST_ASSERT_TRUE(ah.engaged());
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 123.5f, ah.targetAltitude());
}

// 爬升率估计符号：持续上升的高度 -> climbRate > 0
void test_climb_rate_sign_positive() {
    AltitudeHold ah;
    float pc = 0.0f, alt = 100.0f;
    for (int i = 0; i < 50; ++i) {            // 50 拍 @20ms，+2 m/s 上升
        alt += 2.0f * 0.02f;
        ah.update(true, true, alt, 0.0f, 0.02f, pc);
    }
    TEST_ASSERT_TRUE(ah.climbRate() > 1.0f);  // 收敛趋近 +2 m/s
}

// 飞手俯仰杆超死区 -> 交还手动（返回 false），并把目标重锁到当前高度。
void test_pilot_override_releases_and_relatches() {
    AltitudeHold ah;
    float pc = 0.0f;
    ah.update(true, true, 100.0f, 0.0f, 0.02f, pc);   // 锁 100m
    // 飞手大幅推杆，飞机此刻在 105m
    bool drive = ah.update(true, true, 105.0f, /*pilot*/0.6f, 0.02f, pc);
    TEST_ASSERT_FALSE(drive);                          // 交还手动
    TEST_ASSERT_FLOAT_WITHIN(1e-2, 105.0f, ah.targetAltitude());  // 重锁
    // 松杆回中：在 105m 重新接管保持
    bool drive2 = ah.update(true, true, 105.0f, 0.0f, 0.02f, pc);
    TEST_ASSERT_TRUE(drive2);
}

// 低于目标 -> 俯仰指令为正（抬头爬升）；高于目标 -> 为负（低头下降）。
void test_cascade_pitch_sign() {
    AltitudeHold ah;
    float pc = 0.0f;
    ah.update(true, true, 100.0f, 0.0f, 0.02f, pc);   // 接管，锁 100m
    // 掉到 90m：低于目标，应命令抬头 (>0)
    ah.update(true, true, 90.0f, 0.0f, 0.02f, pc);
    TEST_ASSERT_TRUE(pc > 0.0f);
    // 升到 110m：高于目标，应命令低头 (<0)
    AltitudeHold ah2;
    float pc2 = 0.0f;
    ah2.update(true, true, 100.0f, 0.0f, 0.02f, pc2);
    ah2.update(true, true, 110.0f, 0.0f, 0.02f, pc2);
    TEST_ASSERT_TRUE(pc2 < 0.0f);
}

// 目标爬升率被 max_climb_mps 限幅：大高度误差不应让目标爬升率爆掉。
void test_climb_target_clamped() {
    AltitudeHold ah;
    AltHoldConfig c;            // kp_alt 0.05, max_climb 3.0 -> 误差>60m 即饱和
    ah.setConfig(c);
    float pc = 0.0f;
    ah.update(true, true, 100.0f, 0.0f, 0.02f, pc);   // 锁 100m
    // 掉到 0m：误差 100m，外环目标爬升率应被钳在 +3 m/s，俯仰指令仍在 [-1,1]
    ah.update(true, true, 0.0f, 0.0f, 0.02f, pc);
    TEST_ASSERT_TRUE(pc <= 1.0f && pc >= -1.0f);
    TEST_ASSERT_TRUE(pc > 0.0f);
}

// 油门能量耦合：命令爬升(俯仰>0) -> 油门前馈增量>0(同号)，符号一致。
void test_throttle_delta_follows_pitch() {
    AltitudeHold ah;
    float pc = 0.0f;
    ah.update(true, true, 100.0f, 0.0f, 0.02f, pc);   // 锁 100m
    ah.update(true, true, 90.0f, 0.0f, 0.02f, pc);    // 掉到 90m -> 抬头爬升
    TEST_ASSERT_TRUE(pc > 0.0f);                       // 俯仰指令为正
    TEST_ASSERT_TRUE(ah.throttleDelta() > 0.0f);       // 油门前馈同号
    // 升到 110m -> 低头下降 -> 油门增量为负
    AltitudeHold ah2;
    float pc2 = 0.0f;
    ah2.update(true, true, 100.0f, 0.0f, 0.02f, pc2);
    ah2.update(true, true, 110.0f, 0.0f, 0.02f, pc2);
    TEST_ASSERT_TRUE(ah2.throttleDelta() < 0.0f);
}

// 未接管时油门增量为 0（不动油门）。
void test_throttle_delta_zero_when_disengaged() {
    AltitudeHold ah;
    float pc = 0.0f;
    ah.update(false, true, 100.0f, 0.0f, 0.02f, pc);   // 未请求接管
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, ah.throttleDelta());
    ah.update(true, false, 100.0f, 0.0f, 0.02f, pc);   // 气压无效
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, ah.throttleDelta());
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_not_engaged_when_no_request);
    RUN_TEST(test_disengage_when_baro_invalid);
    RUN_TEST(test_engage_latches_target);
    RUN_TEST(test_climb_rate_sign_positive);
    RUN_TEST(test_pilot_override_releases_and_relatches);
    RUN_TEST(test_cascade_pitch_sign);
    RUN_TEST(test_climb_target_clamped);
    RUN_TEST(test_throttle_delta_follows_pitch);
    RUN_TEST(test_throttle_delta_zero_when_disengaged);
    return UNITY_END();
}
