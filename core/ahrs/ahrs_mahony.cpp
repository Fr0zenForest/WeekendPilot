// Ported from madflight src/ahr/Mahony/Mahony.cpp (MIT License),
// originally PaulStoffregen/MahonyAHRS. Adapted: deg/s input, euler output.
#include "ahrs/ahrs_mahony.h"
#include <cmath>

namespace wp {

static constexpr float DEG2RAD = 0.017453292519943295f;
static constexpr float RAD2DEG = 57.29577951308232f;

void AhrsMahony::reset() {
    q0_ = 1.0f; q1_ = q2_ = q3_ = 0.0f;
    ifb_x_ = ifb_y_ = ifb_z_ = 0.0f;
    att_ = Attitude{};
}

void AhrsMahony::update(const ImuSample& imu, float dt) {
    float gx = imu.gyro_x * DEG2RAD;
    float gy = imu.gyro_y * DEG2RAD;
    float gz = imu.gyro_z * DEG2RAD;
    float ax = imu.accel_x, ay = imu.accel_y, az = imu.accel_z;

    const float alen2 = ax * ax + ay * ay + az * az;
    if (alen2 > 0.0f) {
        const float recipNorm = 1.0f / std::sqrt(alen2);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        const float halfvx = q1_ * q3_ - q0_ * q2_;
        const float halfvy = q0_ * q1_ + q2_ * q3_;
        const float halfvz = q0_ * q0_ - 0.5f + q3_ * q3_;

        const float halfex = (ay * halfvz - az * halfvy);
        const float halfey = (az * halfvx - ax * halfvz);
        const float halfez = (ax * halfvy - ay * halfvx);

        if (two_ki_ > 0.0f) {
            ifb_x_ += two_ki_ * halfex * dt;
            ifb_y_ += two_ki_ * halfey * dt;
            ifb_z_ += two_ki_ * halfez * dt;
            gx += ifb_x_; gy += ifb_y_; gz += ifb_z_;
        } else {
            ifb_x_ = ifb_y_ = ifb_z_ = 0.0f;
        }

        gx += two_kp_ * halfex;
        gy += two_kp_ * halfey;
        gz += two_kp_ * halfez;
    }

    gx *= 0.5f * dt; gy *= 0.5f * dt; gz *= 0.5f * dt;
    const float qa = q0_, qb = q1_, qc = q2_;
    q0_ += (-qb * gx - qc * gy - q3_ * gz);
    q1_ += ( qa * gx + qc * gz - q3_ * gy);
    q2_ += ( qa * gy - qb * gz + q3_ * gx);
    q3_ += ( qa * gz + qb * gy - qc * gx);

    const float recipNorm = 1.0f / std::sqrt(q0_*q0_ + q1_*q1_ + q2_*q2_ + q3_*q3_);
    q0_ *= recipNorm; q1_ *= recipNorm; q2_ *= recipNorm; q3_ *= recipNorm;

    quaternionToEuler();
}

void AhrsMahony::quaternionToEuler() {
    // standard aerospace ZYX
    const float sinr = 2.0f * (q0_ * q1_ + q2_ * q3_);
    const float cosr = 1.0f - 2.0f * (q1_ * q1_ + q2_ * q2_);
    att_.roll_deg = std::atan2(sinr, cosr) * RAD2DEG;

    float sinp = 2.0f * (q0_ * q2_ - q3_ * q1_);
    if (sinp > 1.0f) sinp = 1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    att_.pitch_deg = std::asin(sinp) * RAD2DEG;

    const float siny = 2.0f * (q0_ * q3_ + q1_ * q2_);
    const float cosy = 1.0f - 2.0f * (q2_ * q2_ + q3_ * q3_);
    att_.yaw_deg = std::atan2(siny, cosy) * RAD2DEG;
}

}  // namespace wp
