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

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_no_learn_when_stick_not_centered);
    RUN_TEST(test_no_learn_when_disabled);
    RUN_TEST(test_learns_toward_applied_correction);
    return UNITY_END();
}
