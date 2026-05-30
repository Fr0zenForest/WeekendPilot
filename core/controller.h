#pragma once
#include "types.h"
#include "ahrs/ahrs_mahony.h"
#include "modes/mode_controller.h"
#include "modes/stab_mode.h"

namespace wp {

struct ControllerConfig {
    bool enabled = true;
    uint8_t mode_channel = 4;      // ch5
    uint8_t gain_channel = 5;      // ch6
    uint8_t throttle_channel = 2;  // ch3
    uint8_t roll_channel = 0;
    uint8_t pitch_channel = 1;
    uint8_t yaw_channel = 3;
    float throttle_low_us = 1100.0f;
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
};

}  // namespace wp
