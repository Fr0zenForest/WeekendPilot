#include "nav/altitude_hold.h"

namespace wp {

AltitudeHold::AltitudeHold()
    : climb_pid_(1.0f), climb_filt_(2.0f) {
    climb_pid_.setGains(cfg_.climb_gains);
    climb_filt_.setCutoff(cfg_.climb_rate_cutoff_hz);
}

void AltitudeHold::setConfig(const AltHoldConfig& c) {
    cfg_ = c;
    climb_pid_.setGains(cfg_.climb_gains);
    climb_filt_.setCutoff(cfg_.climb_rate_cutoff_hz);
}

bool AltitudeHold::update(bool engage_request, bool baro_valid, float altitude_m,
                          float pilot_pitch_cmd, float dt, float& pitch_cmd_out) {
    // failsafe：未请求 / 气压无效 -> 脱离并复位。
    if (!engage_request || !baro_valid) {
        engaged_ = false;
        have_last_ = false;
        climb_pid_.reset();
        pitch_cmd_out = 0.0f;
        return false;
    }

    // 爬升率估计：高度差分 + PT1。首拍只播种 last，不出数。
    if (have_last_ && dt > 0.0f) {
        climb_filt_.apply((altitude_m - last_alt_m_) / dt, dt);
    } else {
        climb_filt_.reset(0.0f);
    }
    const float climb = climb_filt_.value();
    last_alt_m_ = altitude_m;
    have_last_ = true;

    // 接管起始沿：锁定当前高度为目标，复位积分。
    if (!engaged_) {
        engaged_ = true;
        target_alt_m_ = altitude_m;
        climb_pid_.reset();
    }

    // 飞手俯仰杆超死区：交还手动，并持续把目标重锁到当前高度（松杆后在新高度保持）。
    if (pilot_pitch_cmd > cfg_.pitch_deadband || pilot_pitch_cmd < -cfg_.pitch_deadband) {
        target_alt_m_ = altitude_m;
        climb_pid_.reset();
        pitch_cmd_out = 0.0f;
        return false;
    }

    // 外环：高度误差 -> 目标爬升率（P + 限幅）。
    float climb_target = cfg_.kp_alt * (target_alt_m_ - altitude_m);
    if (climb_target >  cfg_.max_climb_mps) climb_target =  cfg_.max_climb_mps;
    if (climb_target < -cfg_.max_climb_mps) climb_target = -cfg_.max_climb_mps;

    // 内环：爬升率误差 -> 归一化俯仰指令。误差>0(需爬升) -> 俯仰指令>0(抬头)。
    // gyro_rate 复用 climb：kd>0 时 D 项阻尼爬升率变化(≈d²alt/dt²)；kd 默认 0。
    pitch_cmd_out = climb_pid_.update(climb_target, climb, climb, dt);
    return true;
}

}  // namespace wp
