#pragma once

namespace wp {

struct PidGains {
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
};

// 输出限幅在 [-out_limit, out_limit]（默认 1.0，归一化舵量）。
class PidController {
public:
    explicit PidController(float out_limit = 1.0f) : out_limit_(out_limit) {}

    void setGains(const PidGains& g) { gains_ = g; }
    const PidGains& gains() const { return gains_; }

    // setpoint/measurement 同单位（角度或角速率）；gyro_rate 用于 D 项（deg/s）。
    // 返回归一化舵量 [-out_limit, out_limit]。
    float update(float setpoint, float measurement, float gyro_rate, float dt);

    void reset();
    float integralRaw() const { return integral_raw_; }

private:
    PidGains gains_;
    float out_limit_;
    float integral_raw_ = 0.0f;   // 与 ki 分离的原始积分量
};

}  // namespace wp
