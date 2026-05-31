#pragma once

namespace wp {

// WARNING 新代码（学习律参考 INAV servoAutotrim / ArduPilot，手写非逐行移植）。
//    机制可单测；JSBSim 对称机体学出 trim≈0，纠正真实不对称的效果未实物验证。
struct AutoTrimConfig {
    float rate = 0.5f;            // 学习速率（每秒并入比例）
    float max_trim = 0.25f;       // trim 限幅 [-max,+max]（归一化舵量）
    float stick_deadband = 0.05f; // 摇杆居中判定（归一化）
    float level_deg = 5.0f;       // 姿态接近水平阈值（roll/pitch 绝对值）
    float rate_max_dps = 10.0f;   // 角速率低阈值（roll/pitch 陀螺）
};

// 学习器每周期输入（由 Controller 组装）。
struct AutoTrimInputs {
    bool enabled = false;
    bool angle_mode = false;
    float roll_cmd = 0.0f, pitch_cmd = 0.0f;             // 摇杆归一化
    float roll_correction = 0.0f, pitch_correction = 0.0f; // 增稳环施加的修正
    float roll_deg = 0.0f, pitch_deg = 0.0f;
    float gyro_x = 0.0f, gyro_y = 0.0f;                  // deg/s
    float dt = 0.0f;
};

// 平飞松杆稳态时，把增稳修正缓慢并入持久 trim（roll_trim/pitch_trim 原地更新）。
class AutoTrim {
public:
    void setConfig(const AutoTrimConfig& c) { cfg_ = c; }
    // 满足学习条件时更新 roll_trim/pitch_trim（引用，限幅）；返回本拍是否在学习。
    bool update(const AutoTrimInputs& in, float& roll_trim, float& pitch_trim);
    bool learning() const { return learning_; }

private:
    AutoTrimConfig cfg_;
    bool learning_ = false;
    static float absf(float v) { return v < 0.0f ? -v : v; }
    static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
};

}  // namespace wp
