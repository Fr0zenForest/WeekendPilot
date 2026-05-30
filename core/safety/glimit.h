#pragma once

namespace wp {

struct GLimitConfig {
    bool  enabled   = true;
    float soft_g    = 6.0f;    // 软限：超过开始衰减拉杆
    float hard_g    = 10.0f;   // 硬限：拉杆贡献清零
    // 拉杆方向约定：正 pitch 需求 = 抬头 = 加载（正法向 G 增大）。
    // 若实测舵面/IMU 装反，把 pitch_loads_positive 置 false。
    bool  pitch_loads_positive = true;
};

// 返回经 G-limit 衰减后的 pitch 需求。
// accel_z：IMU 法向载荷（单位 g，平飞 ~+1，正向抬头加载）。
// 只衰减"加载方向"的拉杆；卸载方向（推杆）不限。
inline float applyGLimit(float pitch_demand, float accel_z, const GLimitConfig& cfg) {
    if (!cfg.enabled) return pitch_demand;
    float g = cfg.pitch_loads_positive ? accel_z : -accel_z;
    bool pulling = cfg.pitch_loads_positive ? (pitch_demand > 0.0f) : (pitch_demand < 0.0f);
    if (g <= cfg.soft_g || !pulling) return pitch_demand;
    float span = cfg.hard_g - cfg.soft_g;
    float k = (span > 0.0f) ? (1.0f - (g - cfg.soft_g) / span) : 0.0f;
    if (k < 0.0f) k = 0.0f;
    if (k > 1.0f) k = 1.0f;
    return pitch_demand * k;
}

}  // namespace wp
