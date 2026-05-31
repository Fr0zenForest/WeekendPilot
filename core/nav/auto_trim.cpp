#include "nav/auto_trim.h"

namespace wp {

bool AutoTrim::update(const AutoTrimInputs& in, float& roll_trim, float& pitch_trim) {
    learning_ = false;
    // 门控：使能 + Angle 模式 + 摇杆居中 + 姿态近水平 + 角速率低。
    if (!in.enabled || !in.angle_mode) return false;
    if (absf(in.roll_cmd) > cfg_.stick_deadband || absf(in.pitch_cmd) > cfg_.stick_deadband) return false;
    if (absf(in.roll_deg) > cfg_.level_deg || absf(in.pitch_deg) > cfg_.level_deg) return false;
    if (absf(in.gyro_x) > cfg_.rate_max_dps || absf(in.gyro_y) > cfg_.rate_max_dps) return false;

    // 学习律：trim 朝当前增稳修正缓慢并入。闭环里 trim↑ -> 误差↓ -> 修正↓ -> 自然收敛。
    const float k = cfg_.rate * in.dt;
    roll_trim  = clampf(roll_trim  + k * in.roll_correction,  -cfg_.max_trim, cfg_.max_trim);
    pitch_trim = clampf(pitch_trim + k * in.pitch_correction, -cfg_.max_trim, cfg_.max_trim);
    learning_ = true;
    return true;
}

}  // namespace wp
