#pragma once
#include "types.h"

namespace wp {

// Mahony 6DOF 互补滤波。输入陀螺 deg/s、加速度 g，输出欧拉角 deg。
// 移植自 madflight (MIT) / PaulStoffregen MahonyAHRS。
class AhrsMahony {
public:
    void setKp(float two_kp) { two_kp_ = two_kp; }
    void setKi(float two_ki) { two_ki_ = two_ki; }
    void reset();

    // gyro deg/s, accel g, dt seconds
    void update(const ImuSample& imu, float dt);
    Attitude attitude() const { return att_; }

private:
    float q0_ = 1.0f, q1_ = 0.0f, q2_ = 0.0f, q3_ = 0.0f;
    float ifb_x_ = 0.0f, ifb_y_ = 0.0f, ifb_z_ = 0.0f;
    float two_kp_ = 2.0f * 0.5f;
    float two_ki_ = 2.0f * 0.0f;
    Attitude att_;
    void quaternionToEuler();
};

}  // namespace wp
