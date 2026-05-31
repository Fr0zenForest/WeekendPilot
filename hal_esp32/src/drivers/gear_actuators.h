#pragma once
#include "config/capabilities.h"
#if WP_HAS_LANDING_GEAR
#include "nav/landing_gear.h"
#include "io/composite_servo_output.h"

namespace wp {

// 类型 B：连续旋转舵机。Deploy=放下方向 us，Retract=收起方向 us，Stop=1500µs（软件停舵）。
// 经 CompositeServoOutput 的某全局通道输出。
class GearServo : public ILandingGearActuator {
public:
    // out: 输出路由；ch: 全局输出通道；deploy_us/retract_us: 两个方向的速度指令。
    GearServo(CompositeServoOutput& out, int ch,
              uint16_t deploy_us = 2000, uint16_t retract_us = 1000)
        : out_(out), ch_(ch), deploy_us_(deploy_us), retract_us_(retract_us) {}
    void apply(GearDrive d) override;

private:
    CompositeServoOutput& out_;
    int      ch_;
    uint16_t deploy_us_, retract_us_;
};

// 类型 C：裸直流电机 + H 桥。PWM（全速 us）走 CompositeServoOutput，方向走 DIR GPIO。
// Stop：写 1500µs/占空比0 + DIR 任意；Deploy/Retract：全速 us + DIR 高/低。
class GearHbridge : public ILandingGearActuator {
public:
    GearHbridge(CompositeServoOutput& out, int pwm_ch, int dir_pin)
        : out_(out), pwm_ch_(pwm_ch), dir_pin_(dir_pin) {}
    void begin();   // pinMode(dir_pin, OUTPUT)
    void apply(GearDrive d) override;

private:
    CompositeServoOutput& out_;
    int pwm_ch_, dir_pin_;
};

}  // namespace wp
#endif  // WP_HAS_LANDING_GEAR
