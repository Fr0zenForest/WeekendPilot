#pragma once
#include "math/filter_pt1.h"
#include "pid/pid_controller.h"

namespace wp {

// WARNING 新代码（非逐行移植）。级联结构参考 INAV 固定翼 nav_fixedwing 高度环思路，
//    控制律为本项目手写。SITL 闭环已用 JSBSim 真值高度验证；未经实物气压计实测。
struct AltHoldConfig {
    float kp_alt        = 0.05f;   // 高度误差(m) -> 目标爬升率(m/s)
    float max_climb_mps = 3.0f;    // 目标爬升率限幅 (±, m/s)
    PidGains climb_gains{0.08f, 0.03f, 0.0f};  // 爬升率误差(m/s) -> 归一化俯仰指令
    float climb_rate_cutoff_hz = 2.0f;  // 爬升率估计 PT1 截止频率
    float pitch_deadband = 0.10f;  // 飞手俯仰杆死区：超出则交还手动并重锁目标
    // 油门能量耦合（俯仰->油门前馈，参考 INAV fw_p2t）。纯升降舵定高在固定油门下
    // 拉杆抬头会耗尽空速→失速性下沉；命令爬升时按俯仰指令补油门维持能量。
    float kff_pitch_throttle = 0.5f;  // 俯仰指令[-1,1] -> 油门增量[-0.5,0.5]
};

// 气压定高层。每周期 update() 一次；仅在请求接管 & 气压有效时驱动俯仰。
class AltitudeHold {
public:
    AltitudeHold();
    void setConfig(const AltHoldConfig& c);

    // engage_request: 上层判定（已使能 & Angle 模式 & 通道拨上）
    // baro_valid    : 气压高度是否有效（无效 -> 强制脱离 failsafe）
    // altitude_m    : 当前相对高度 (m)
    // pilot_pitch_cmd: 飞手俯仰杆归一化 [-1,1]
    // dt            : 秒
    // pitch_cmd_out : 输出归一化俯仰指令（仅返回 true 时有效）
    // 返回 true = 定高正在驱动俯仰（调用方据此覆盖手动俯仰杆）；
    // 返回 false = 未接管（未使能/气压无效/飞手杆超死区），调用方维持手动。
    bool update(bool engage_request, bool baro_valid, float altitude_m,
                float pilot_pitch_cmd, float dt, float& pitch_cmd_out);

    bool  engaged() const { return engaged_; }
    float targetAltitude() const { return target_alt_m_; }
    float climbRate() const { return climb_filt_.value(); }
    // 油门前馈增量 [-kff,+kff]，叠加到基准油门。仅上一次 update() 返回 true 时有效，
    // 否则为 0（未接管不动油门）。
    float throttleDelta() const { return throttle_delta_; }

private:
    AltHoldConfig cfg_;
    PidController climb_pid_;   // out_limit 1.0 -> 归一化俯仰
    FilterPt1     climb_filt_;  // 爬升率低通
    bool   engaged_     = false;
    bool   have_last_   = false;
    float  last_alt_m_  = 0.0f;
    float  target_alt_m_= 0.0f;
    float  throttle_delta_ = 0.0f;  // 上次 update 的油门前馈增量
};

}  // namespace wp
