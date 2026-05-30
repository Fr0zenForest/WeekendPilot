#include "controller.h"

namespace wp {

Controller::Controller() {
    modes_.setConfig(cfg_.stab);
    mixer_.setAirframe(cfg_.airframe);
}

void Controller::setConfig(const ControllerConfig& cfg) {
    cfg_ = cfg;
    modes_.setConfig(cfg_.stab);
    mixer_.setAirframe(cfg_.airframe);
}

ServoCommand Controller::update(const ControlInput& in) {
    ServoCommand out{};
    // 默认直通所有通道
    for (int i = 0; i < kNumServos; ++i) out.servo[i] = in.channels[i];

    FlightMode mode = modeFromChannel(in.channels[cfg_.mode_channel]);

    // 直通条件：未启用 / 链路丢失 / IMU 无效 / Off 模式
    if (!cfg_.enabled || !in.link_ok || !in.imu.valid || mode == FlightMode::Off) {
        return out;
    }

    ahrs_.update(in.imu, in.mag, in.dt);

    float roll_cmd  = channelToNorm(in.channels[cfg_.roll_channel]);
    float pitch_cmd = channelToNorm(in.channels[cfg_.pitch_channel]);
    float yaw_cmd   = channelToNorm(in.channels[cfg_.yaw_channel]);
    float gain      = gainFromChannel(in.channels[cfg_.gain_channel]);
    bool throttle_low = in.channels[cfg_.throttle_channel] < cfg_.throttle_low_us;

    StabCorrection corr = modes_.update(mode, roll_cmd, pitch_cmd, yaw_cmd,
                                        ahrs_.attitude(), in.imu, in.dt, throttle_low);

    // 轴需求 = 手动 + gain*PID修正
    float demand[static_cast<int>(MixSource::Count)] = {0};
    demand[static_cast<int>(MixSource::Roll)]  = roll_cmd  + gain * corr.roll;
    demand[static_cast<int>(MixSource::Pitch)] = pitch_cmd + gain * corr.pitch;
    demand[static_cast<int>(MixSource::Yaw)]   = yaw_cmd   + gain * corr.yaw;
    // 油门：归一化 [0,1]（throttle 通道 us -> 0..1）
    demand[static_cast<int>(MixSource::Throttle)] =
        gainFromChannel(in.channels[cfg_.throttle_channel]);
    // 襟翼需求（可选）
    demand[static_cast<int>(MixSource::Flap)] = cfg_.flap_enabled
        ? gainFromChannel(in.channels[cfg_.flap_channel]) : 0.0f;

    // G-limit：衰减加载方向的 pitch 需求
    demand[static_cast<int>(MixSource::Pitch)] =
        applyGLimit(demand[static_cast<int>(MixSource::Pitch)], in.imu.accel_z, cfg_.glimit);

    // 混控（覆盖全部 kNumServos：无规则的输出口归中位，AUX 输出靠下方外设表显式路由）
    mixer_.mix(demand, out.servo);

    // 外设直通：用裸 RC us 覆盖指定输出口
    for (int i = 0; i < kMaxPeripherals; ++i) {
        const PeripheralMap& p = cfg_.peripherals[i];
        if (p.enabled && p.servo_out < kNumServos && p.rc_channel < kNumChannels)
            out.servo[p.servo_out] = in.channels[p.rc_channel];
    }
    return out;
}

}  // namespace wp
