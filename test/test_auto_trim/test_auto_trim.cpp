#include "unity.h"
#include "nav/auto_trim.h"
using namespace wp;

void setUp() {} void tearDown() {}

// WARNING 新代码（学习律参考 INAV servoAutotrim/ArduPilot，手写）。机制可单测；
//    JSBSim 对称机体学出 trim≈0，"真能纠正不对称"未在实物验证。

// 条件不满足（摇杆未居中）-> 不学习，trim 不变。
void test_no_learn_when_stick_not_centered() {
    AutoTrim at;
    float roll_trim = 0.0f, pitch_trim = 0.0f;
    AutoTrimInputs in{};
    in.enabled = true; in.angle_mode = true;
    in.roll_cmd = 0.5f; in.pitch_cmd = 0.0f;     // roll 杆没居中
    in.roll_correction = 0.2f; in.pitch_correction = 0.0f;
    in.roll_deg = 0.0f; in.pitch_deg = 0.0f;
    in.gyro_x = 0.0f; in.gyro_y = 0.0f;
    in.dt = 0.02f;
    at.update(in, roll_trim, pitch_trim);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, roll_trim);
}

// 关闭自动配平 -> 不学习。
void test_no_learn_when_disabled() {
    AutoTrim at;
    float rt = 0.0f, pt = 0.0f;
    AutoTrimInputs in{};
    in.enabled = false; in.angle_mode = true;
    in.roll_correction = 0.3f; in.dt = 0.02f;
    at.update(in, rt, pt);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, rt);
}

// 稳态满足条件：持续正的 roll 修正 -> roll_trim 朝正方向增长。
void test_learns_toward_applied_correction() {
    AutoTrim at;
    float rt = 0.0f, pt = 0.0f;
    AutoTrimInputs in{};
    in.enabled = true; in.angle_mode = true;
    in.roll_cmd = 0.0f; in.pitch_cmd = 0.0f;       // 杆居中
    in.roll_correction = 0.2f; in.pitch_correction = -0.1f;
    in.roll_deg = 1.0f; in.pitch_deg = 1.0f;       // 接近水平
    in.gyro_x = 1.0f; in.gyro_y = 1.0f;            // 角速率低
    in.dt = 0.02f;
    for (int i = 0; i < 100; ++i) at.update(in, rt, pt);
    TEST_ASSERT_TRUE(rt > 0.0f);                    // 跟随正修正
    TEST_ASSERT_TRUE(pt < 0.0f);                    // 跟随负修正
}

// 闭环收敛：correction 随 trim 增大而减小，trim 应收敛到 target 附近并停。
void test_closed_loop_converges_to_target() {
    AutoTrim at;
    AutoTrimConfig c; at.setConfig(c);
    float rt = 0.0f, pt = 0.0f;
    const float target = 0.1f;     // 假想机体需要 0.1 的 roll trim
    for (int i = 0; i < 2000; ++i) {
        AutoTrimInputs in{};
        in.enabled = true; in.angle_mode = true;
        in.roll_cmd = 0.0f; in.pitch_cmd = 0.0f;
        in.roll_deg = 0.5f; in.pitch_deg = 0.5f;
        in.gyro_x = 0.5f; in.gyro_y = 0.5f;
        in.dt = 0.02f;
        // 增稳修正 = 残余误差所需补偿，随 trim 逼近 target 而减小
        in.roll_correction = (target - rt);
        at.update(in, rt, pt);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.02f, target, rt);   // 收敛到 target
}

// 限幅：超大修正不应让 trim 越过 max_trim。
void test_trim_clamped() {
    AutoTrim at;
    AutoTrimConfig c; c.max_trim = 0.25f; at.setConfig(c);
    float rt = 0.0f, pt = 0.0f;
    for (int i = 0; i < 1000; ++i) {
        AutoTrimInputs in{};
        in.enabled = true; in.angle_mode = true;
        in.roll_deg = 0.0f; in.pitch_deg = 0.0f;
        in.gyro_x = 0.0f; in.gyro_y = 0.0f;
        in.roll_correction = 1.0f;     // 持续满修正
        in.dt = 0.02f;
        at.update(in, rt, pt);
    }
    TEST_ASSERT_TRUE(rt <= 0.25f + 1e-6f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 0.25f, rt);     // 顶到限幅
}

// 角速率过高（如阵风/机动）-> 不学习。
void test_no_learn_when_rate_high() {
    AutoTrim at;
    float rt = 0.0f, pt = 0.0f;
    AutoTrimInputs in{};
    in.enabled = true; in.angle_mode = true;
    in.roll_correction = 0.3f;
    in.gyro_x = 50.0f;             // 角速率高
    in.dt = 0.02f;
    at.update(in, rt, pt);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, rt);
    TEST_ASSERT_FALSE(at.learning());
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_no_learn_when_stick_not_centered);
    RUN_TEST(test_no_learn_when_disabled);
    RUN_TEST(test_learns_toward_applied_correction);
    RUN_TEST(test_closed_loop_converges_to_target);
    RUN_TEST(test_trim_clamped);
    RUN_TEST(test_no_learn_when_rate_high);
    return UNITY_END();
}
