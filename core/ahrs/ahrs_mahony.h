#pragma once
#include "types.h"

namespace wp {

// Mahony 6/9DOF 互补滤波。输入陀螺 deg/s、加速度 g，输出欧拉角 deg。
// 移植自 madflight (MIT) / PaulStoffregen MahonyAHRS。
class AhrsMahony {
public:
    void setKp(float two_kp) { two_kp_ = two_kp; }
    void setKi(float two_ki) { two_ki_ = two_ki; }
    // 陀螺零偏积分(ifb_)对称硬限幅(rad/s)。防系统性偏差致零偏估计无界爬升。
    // lim<0 忽略(保持当前值)；lim==0 合法(等于把零偏估计夹死为 0)。
    void setBiasLimit(float lim) { if (lim >= 0.0f) ifb_limit_ = lim; }
    // 加速度幅值门控窗口（g^2）。|a|^2 在 [min,max] 之外时跳过加速度/磁力计校正，
    // 让陀螺独自积分（协调转弯中加速度看不见倾斜，见 Appendix C）。默认 0.9-1.1g。
    void setAccelGate(float alen2_min, float alen2_max) {
        alen2_min_ = alen2_min; alen2_max_ = alen2_max;
    }
    void reset();

    // 6DOF：仅陀螺+加速度（无磁力计）。保持向后兼容。
    void update(const ImuSample& imu, float dt);
    // 9/6DOF 自动派发：mag.valid 且非零 -> 9DOF，否则退回 6DOF。
    void update(const ImuSample& imu, const MagSample& mag, float dt);
    Attitude attitude() const { return att_; }

private:
    float q0_ = 1.0f, q1_ = 0.0f, q2_ = 0.0f, q3_ = 0.0f;
    float ifb_x_ = 0.0f, ifb_y_ = 0.0f, ifb_z_ = 0.0f;
    float two_kp_ = 2.0f * 0.5f;
    float two_ki_ = 2.0f * 0.0f;
    float ifb_limit_ = 0.1f;    // 零偏积分硬限幅(rad/s)，≈5.7°/s。默认保守。
    float alen2_min_ = 0.81f;   // (0.9g)^2
    float alen2_max_ = 1.21f;   // (1.1g)^2
    Attitude att_;
    void update6DOF(float gx, float gy, float gz, float ax, float ay, float az, float dt);
    void update9DOF(float gx, float gy, float gz, float ax, float ay, float az,
                    float mx, float my, float mz, float dt);
    void integrateQuaternion(float gx, float gy, float gz, float dt);
    void quaternionToEuler();
};

}  // namespace wp
