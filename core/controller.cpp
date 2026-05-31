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
    althold_.setConfig(cfg_.althold);
    auto_trim_.setConfig(cfg_.auto_trim);
    blackbox_.begin(cfg_.blackbox, bb_sink_);
}

void Controller::attachBlackboxSink(IBlackboxSink* sink) {
    bb_sink_ = sink;
    blackbox_.begin(cfg_.blackbox, bb_sink_);
}

ServoCommand Controller::update(const ControlInput& in) {
    // 黑匣子时间戳：上电起的墙钟毫秒（含 Off/失控等不记录的拍），+0.5f 四舍五入到 ms。
    // 故意累加在早返回之前——日志里能看出在控/脱控的时间间隙，而非仅在控时长。
    frame_t_ms_ += static_cast<uint32_t>(in.dt * 1000.0f + 0.5f);
    ServoCommand out{};
    // TODO(phase3): 失效回退按 servo[i]=channels[i] 直通，仅 Standard 布局正确；
    // V尾/elevon/flaperon 在失效态会得到错误的舵面映射（见设计文档附录 D 已知限制）。
    // 后续应让失效回退也走 mixer，喂一个安全 demand。
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

    // 定高接管：仅 Angle 模式 + 总开关 + 通道拨上 + 气压有效时驱动俯仰。
    // cfg_.enabled 此处必为 true（上方 Off/失效早返回已拦截）。
    bool althold_req = cfg_.althold_enabled
                       && mode == FlightMode::Angle
                       && in.channels[cfg_.althold_channel] > kModeRateThresh;
    float ah_pitch = 0.0f;
    bool althold_active = althold_.update(althold_req, in.baro.valid, in.baro.altitude_m,
                                          pitch_cmd, in.dt, ah_pitch);
    if (althold_active) {
        pitch_cmd = ah_pitch;   // 用定高俯仰指令替换飞手俯仰杆，喂给 Angle 内环
    }

    StabCorrection corr = modes_.update(mode, roll_cmd, pitch_cmd, yaw_cmd,
                                        ahrs_.attitude(), in.imu, in.dt, throttle_low);

    // 轴需求 = 手动 + gain*PID修正
    float demand[static_cast<int>(MixSource::Count)] = {0};
    demand[static_cast<int>(MixSource::Roll)]  = roll_cmd  + gain * corr.roll  + cfg_.roll_trim;
    demand[static_cast<int>(MixSource::Pitch)] = pitch_cmd + gain * corr.pitch + cfg_.pitch_trim;
    demand[static_cast<int>(MixSource::Yaw)]   = yaw_cmd   + gain * corr.yaw;
    // 油门：归一化 [0,1]（throttle 通道 us -> 0..1）
    float throttle_demand = gainFromChannel(in.channels[cfg_.throttle_channel]);
    // 定高油门能量耦合：叠加俯仰->油门前馈增量，再夹回 [0,1]。
    if (althold_active) {
        throttle_demand += althold_.throttleDelta();
        if (throttle_demand > 1.0f) throttle_demand = 1.0f;
        if (throttle_demand < 0.0f) throttle_demand = 0.0f;
    }
    demand[static_cast<int>(MixSource::Throttle)] = throttle_demand;
    // 襟翼需求（可选）
    demand[static_cast<int>(MixSource::Flap)] = cfg_.flap_enabled
        ? gainFromChannel(in.channels[cfg_.flap_channel]) : 0.0f;

    // G-limit：衰减加载方向的 pitch 需求
    demand[static_cast<int>(MixSource::Pitch)] =
        applyGLimit(demand[static_cast<int>(MixSource::Pitch)], in.imu.accel_z, cfg_.glimit);

    // 自动配平：平飞松杆稳态时把增稳修正缓慢并入持久 trim。
    if (cfg_.auto_trim_enabled) {
        AutoTrimInputs ati{};
        ati.enabled = true;
        ati.angle_mode = (mode == FlightMode::Angle);
        ati.roll_cmd = roll_cmd; ati.pitch_cmd = pitch_cmd;
        ati.roll_correction = gain * corr.roll;
        ati.pitch_correction = gain * corr.pitch;
        Attitude a = ahrs_.attitude();
        ati.roll_deg = a.roll_deg; ati.pitch_deg = a.pitch_deg;
        ati.gyro_x = in.imu.gyro_x; ati.gyro_y = in.imu.gyro_y;
        ati.dt = in.dt;
        auto_trim_.update(ati, cfg_.roll_trim, cfg_.pitch_trim);
    }

    // 混控（覆盖全部 kNumServos：无规则的输出口归中位，AUX 输出靠下方外设表显式路由）
    mixer_.mix(demand, out.servo);

    // 外设直通：用裸 RC us 覆盖指定输出口
    for (int i = 0; i < kMaxPeripherals; ++i) {
        const PeripheralMap& p = cfg_.peripherals[i];
        if (p.enabled && p.servo_out < kNumServos && p.rc_channel < kNumChannels)
            out.servo[p.servo_out] = in.channels[p.rc_channel];
    }

    // 黑匣子：把本拍在控状态编码落盘（降采样在 Blackbox 内部）。
    // 注：Off/链路丢失/IMU 失效在前部早返回，不记录（黑匣子只覆盖在控状态）。
    if (cfg_.blackbox.enabled) {
        BlackboxFrame fr{};
        fr.t_ms = frame_t_ms_;
        fr.gyro_x = in.imu.gyro_x; fr.gyro_y = in.imu.gyro_y; fr.gyro_z = in.imu.gyro_z;
        fr.accel_x = in.imu.accel_x; fr.accel_y = in.imu.accel_y; fr.accel_z = in.imu.accel_z;
        Attitude a = ahrs_.attitude();
        fr.roll_deg = a.roll_deg; fr.pitch_deg = a.pitch_deg; fr.yaw_deg = a.yaw_deg;
        for (int i = 0; i < kNumServos; ++i) fr.servo[i] = out.servo[i];
        fr.baro_alt_m = in.baro.altitude_m;
        fr.mode = static_cast<uint8_t>(mode);
        // flags 位定义见 blackbox_frame.h。注意：本记录点在早返回之后，link_ok 与
        // imu.valid 在此恒为真（bit0/bit1 恒置位）——保留是为了位布局稳定，且日后若把
        // 记录点移到早返回之前可如实反映脱控；当前真正变化的是 bit2(baro)/bit3(althold)。
        fr.flags = uint8_t((in.link_ok ? 1 : 0) | (in.imu.valid ? 2 : 0) |
                           (in.baro.valid ? 4 : 0) | (althold_active ? 8 : 0));
        blackbox_.logFrame(fr);
    }
    return out;
}

ServoCommand Controller::updateFromBundle(const uint16_t (&channels)[kNumChannels],
                                          const SensorBundle& bundle,
                                          float dt, bool link_ok) {
    ControlInput in{};
    for (int i = 0; i < kNumChannels; ++i) in.channels[i] = channels[i];
    in.imu  = bundle.imu;
    in.mag  = bundle.mag;
    in.baro = bundle.baro;
    in.dt = dt;
    in.link_ok = link_ok;
    return update(in);
}

}  // namespace wp
