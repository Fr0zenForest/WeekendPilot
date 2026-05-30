#include "pid/pid_controller.h"

namespace wp {

void PidController::reset() {
    integral_raw_ = 0.0f;
}

float PidController::update(float setpoint, float measurement, float gyro_rate, float dt) {
    const float error = setpoint - measurement;

    // 积分原始量累积（与 ki 分离）
    integral_raw_ += error * dt;

    const float p = gains_.kp * error;
    const float i = gains_.ki * integral_raw_;
    const float d = -gains_.kd * gyro_rate;   // D 用陀螺，符号为负，对抗运动

    float out = p + i + d;

    // back-calculation anti-windup：输出超限时把 integral_raw 回缩到
    // "刚好让输出落在限幅边界"对应的值。
    if (out > out_limit_) {
        if (gains_.ki > 0.0f) {
            integral_raw_ = (out_limit_ - p - d) / gains_.ki;
        }
        out = out_limit_;
    } else if (out < -out_limit_) {
        if (gains_.ki > 0.0f) {
            integral_raw_ = (-out_limit_ - p - d) / gains_.ki;
        }
        out = -out_limit_;
    }
    return out;
}

}  // namespace wp
