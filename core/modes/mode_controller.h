#pragma once
#include "types.h"
#include "modes/stab_mode.h"
#include "pid/pid_controller.h"
#include "ahrs/ahrs_mahony.h"

namespace wp {

struct StabConfig {
    PidGains angle_roll  {0.011f, 0.0f, 0.0056f};
    PidGains angle_pitch {0.022f, 0.0f, 0.011f};
    PidGains rate_roll   {0.0025f, 0.0f, 0.00003f};
    PidGains rate_pitch  {0.0025f, 0.0f, 0.00003f};
    PidGains rate_yaw    {0.004f, 0.0f, 0.0f};
    float max_angle_roll_deg  = 45.0f;
    float max_angle_pitch_deg = 30.0f;
    float max_rate_roll_dps   = 180.0f;
    float max_rate_pitch_dps  = 120.0f;
    float max_rate_yaw_dps    = 90.0f;
    float pitch_offset_deg    = 3.0f;   // 平飞迎角补偿
    float blend_ms            = 200.0f; // 模式切换过渡
};

// 三轴归一化修正量（叠加到手动舵量之前）
struct StabCorrection {
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
};

class ModeController {
public:
    ModeController();
    void setConfig(const StabConfig& cfg) { cfg_ = cfg; applyGains(); }

    // roll/pitch/yaw_cmd 为摇杆归一化 [-1,1]；att 当前姿态；imu 当前陀螺；dt 秒。
    StabCorrection update(FlightMode mode,
                          float roll_cmd, float pitch_cmd, float yaw_cmd,
                          const Attitude& att, const ImuSample& imu,
                          float dt, bool throttle_low);

    FlightMode activeMode() const { return active_mode_; }

private:
    StabConfig cfg_;
    PidController pid_angle_roll_, pid_angle_pitch_;
    PidController pid_rate_roll_, pid_rate_pitch_, pid_rate_yaw_;
    FlightMode active_mode_ = FlightMode::Off;
    float blend_ = 1.0f;          // 0..1 当前模式占比（过渡用）
    StabCorrection last_corr_;    // 过渡起点
    void applyGains();
    StabCorrection computeFor(FlightMode mode,
                              float r_cmd, float p_cmd, float y_cmd,
                              const Attitude& att, const ImuSample& imu,
                              float dt, bool throttle_low);
};

}  // namespace wp
