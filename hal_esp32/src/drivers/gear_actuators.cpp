#include "drivers/gear_actuators.h"
#if WP_HAS_LANDING_GEAR
#include <Arduino.h>

namespace wp {

void GearServo::apply(GearDrive d) {
    switch (d) {
        case GearDrive::Deploy:  out_.writeUs(ch_, deploy_us_);  break;
        case GearDrive::Retract: out_.writeUs(ch_, retract_us_); break;
        case GearDrive::Stop:
        default:                 out_.writeUs(ch_, 1500);        break;  // 软件停舵
    }
}

void GearHbridge::begin() {
    pinMode(dir_pin_, OUTPUT);
    digitalWrite(dir_pin_, LOW);
}

void GearHbridge::apply(GearDrive d) {
    switch (d) {
        case GearDrive::Deploy:
            digitalWrite(dir_pin_, HIGH);
            out_.writeUs(pwm_ch_, 2000);   // 全速
            break;
        case GearDrive::Retract:
            digitalWrite(dir_pin_, LOW);
            out_.writeUs(pwm_ch_, 2000);   // 全速（反向由 DIR 决定）
            break;
        case GearDrive::Stop:
        default:
            out_.writeUs(pwm_ch_, 1000);   // 占空比最小，电机停
            break;
    }
}

}  // namespace wp
#endif  // WP_HAS_LANDING_GEAR
