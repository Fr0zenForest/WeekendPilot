#pragma once
#include "types.h"
#include "ahrs/ahrs_mahony.h"
#include "modes/mode_controller.h"
#include "modes/stab_mode.h"
#include "mixer/mixer.h"
#include "safety/glimit.h"

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
};

class Controller {
public:
    Controller();
    void setConfig(const ControllerConfig& cfg);
    ServoCommand update(const ControlInput& in);
    Attitude attitude() const { return ahrs_.attitude(); }

private:
    ControllerConfig cfg_;
    AhrsMahony ahrs_;
    ModeController modes_;
    Mixer mixer_;
};

}  // namespace wp
