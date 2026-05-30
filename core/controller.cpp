#include "controller.h"

namespace wp {

Controller::Controller() {
    modes_.setConfig(cfg_.stab);
}

void Controller::setConfig(const ControllerConfig& cfg) {
    cfg_ = cfg;
    modes_.setConfig(cfg_.stab);
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

    // 舵量 = 手动 + gain*PID修正
    auto mix = [&](uint8_t servo_idx, uint8_t in_ch, float c) {
        float manual = channelToNorm(in.channels[in_ch]);
        float blended = manual + gain * c;
        out.servo[servo_idx] = normToServoUs(blended);
    };
    mix(0, cfg_.roll_channel,  corr.roll);    // aileron
    mix(1, cfg_.pitch_channel, corr.pitch);   // elevator
    mix(3, cfg_.yaw_channel,   corr.yaw);     // rudder
    // servo[2] throttle 直通（已在上面 passthrough）
    return out;
}

}  // namespace wp
