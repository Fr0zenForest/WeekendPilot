#include "modes/mode_controller.h"

namespace wp {

ModeController::ModeController() { applyGains(); }

void ModeController::applyGains() {
    pid_angle_roll_.setGains(cfg_.angle_roll);
    pid_angle_pitch_.setGains(cfg_.angle_pitch);
    pid_rate_roll_.setGains(cfg_.rate_roll);
    pid_rate_pitch_.setGains(cfg_.rate_pitch);
    pid_rate_yaw_.setGains(cfg_.rate_yaw);
}

StabCorrection ModeController::computeFor(FlightMode mode,
                                          float r_cmd, float p_cmd, float y_cmd,
                                          const Attitude& att, const ImuSample& imu,
                                          float dt, bool throttle_low) {
    StabCorrection c;
    if (mode == FlightMode::Off) return c;

    // Yaw 永远 Rate
    {
        float target = y_cmd * cfg_.max_rate_yaw_dps;
        c.yaw = pid_rate_yaw_.update(target, imu.gyro_z, imu.gyro_z, dt);
    }

    if (mode == FlightMode::Angle) {
        float tgt_roll  = r_cmd * cfg_.max_angle_roll_deg;
        float tgt_pitch = p_cmd * cfg_.max_angle_pitch_deg + cfg_.pitch_offset_deg;
        c.roll  = pid_angle_roll_.update(tgt_roll,  att.roll_deg,  imu.gyro_x, dt);
        c.pitch = pid_angle_pitch_.update(tgt_pitch, att.pitch_deg, imu.gyro_y, dt);
    } else { // Rate
        // Rate 地面冻结：油门低时冻结 I（防地面 I 跑飞）
        float tgt_roll  = r_cmd * cfg_.max_rate_roll_dps;
        float tgt_pitch = p_cmd * cfg_.max_rate_pitch_dps;
        if (throttle_low) {
            pid_rate_roll_.reset();
            pid_rate_pitch_.reset();
        }
        c.roll  = pid_rate_roll_.update(tgt_roll,  imu.gyro_x, imu.gyro_x, dt);
        c.pitch = pid_rate_pitch_.update(tgt_pitch, imu.gyro_y, imu.gyro_y, dt);
    }
    return c;
}

StabCorrection ModeController::update(FlightMode mode,
                                      float roll_cmd, float pitch_cmd, float yaw_cmd,
                                      const Attitude& att, const ImuSample& imu,
                                      float dt, bool throttle_low) {
    // 模式切换检测 -> 启动过渡
    if (mode != active_mode_) {
        active_mode_ = mode;
        blend_ = 0.0f;   // 从上次修正渐变到新模式
    }

    StabCorrection target = computeFor(mode, roll_cmd, pitch_cmd, yaw_cmd,
                                       att, imu, dt, throttle_low);

    // 线性过渡 blend 0..1
    if (blend_ < 1.0f && cfg_.blend_ms > 0.0f) {
        blend_ += dt * 1000.0f / cfg_.blend_ms;
        if (blend_ > 1.0f) blend_ = 1.0f;
    }
    StabCorrection out;
    out.roll  = last_corr_.roll  + (target.roll  - last_corr_.roll)  * blend_;
    out.pitch = last_corr_.pitch + (target.pitch - last_corr_.pitch) * blend_;
    out.yaw   = last_corr_.yaw   + (target.yaw   - last_corr_.yaw)   * blend_;

    if (blend_ >= 1.0f) last_corr_ = target;
    return out;
}

}  // namespace wp
