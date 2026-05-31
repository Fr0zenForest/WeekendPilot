#pragma once
#include "types.h"
#include "ahrs/ahrs_mahony.h"
#include "modes/mode_controller.h"
#include "modes/stab_mode.h"
#include "mixer/mixer.h"
#include "safety/glimit.h"
#include "sensors/sensor_bundle.h"
#include "nav/altitude_hold.h"
#include "nav/auto_trim.h"
#include "blackbox/blackbox.h"

namespace wp {

constexpr int kMaxPeripherals = 4;

// 外设直通：把某 RC 通道裸 us 直接覆盖到某 servo 输出口（起落架/灯/襟翼开关）。
struct PeripheralMap {
    uint8_t servo_out = 0;
    uint8_t rc_channel = 0;
    bool    enabled = false;
};

struct ControllerConfig {
    bool enabled = true;
    uint8_t mode_channel = 4;      // ch5
    uint8_t gain_channel = 5;      // ch6
    uint8_t throttle_channel = 2;  // ch3
    uint8_t roll_channel = 0;
    uint8_t pitch_channel = 1;
    uint8_t yaw_channel = 3;
    uint8_t flap_channel = 6;      // ch7，无则保持中位（flap 需求 0）
    bool    flap_enabled = false;  // 默认不引入 flap 需求
    float throttle_low_us = 1100.0f;
    Airframe airframe = Airframe::Standard;
    GLimitConfig glimit;
    PeripheralMap peripherals[kMaxPeripherals];
    StabConfig stab;
    bool    althold_enabled = false;   // 总开关（默认关，全写好先不使能）
    uint8_t althold_channel = 7;       // ch8：拨上启用定高
    AltHoldConfig althold;
    bool  auto_trim_enabled = false;  // 总开关（默认关，全写好先不使能）
    float roll_trim  = 0.0f;          // 持久配平（归一化舵量），随 NVS 存
    float pitch_trim = 0.0f;
    AutoTrimConfig auto_trim;
    BlackboxConfig blackbox;
};

class Controller {
public:
    Controller();
    void setConfig(const ControllerConfig& cfg);
    ServoCommand update(const ControlInput& in);
    // 从 SensorFrontend 的 SensorBundle 喂控制器（拆进 ControlInput 后复用 update）。
    // 当前只搬运 update() 认识的 imu/mag/baro；gnss/airspeed 待阶段 3 用上时再接。
    ServoCommand updateFromBundle(const uint16_t (&channels)[kNumChannels],
                                  const SensorBundle& bundle,
                                  float dt, bool link_ok);
    Attitude attitude() const { return ahrs_.attitude(); }
    void attachBlackboxSink(IBlackboxSink* sink);
    float rollTrim() const { return cfg_.roll_trim; }
    float pitchTrim() const { return cfg_.pitch_trim; }

private:
    ControllerConfig cfg_;
    AhrsMahony ahrs_;
    ModeController modes_;
    Mixer mixer_;
    AltitudeHold althold_;
    AutoTrim auto_trim_;
    Blackbox blackbox_;
    IBlackboxSink* bb_sink_ = nullptr;
    uint32_t frame_t_ms_ = 0;
};

}  // namespace wp
