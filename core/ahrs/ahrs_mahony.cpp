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
    update6DOF(imu.gyro_x * DEG2RAD, imu.gyro_y * DEG2RAD, imu.gyro_z * DEG2RAD,
               imu.accel_x, imu.accel_y, imu.accel_z, dt);
    quaternionToEuler();
}

void AhrsMahony::update(const ImuSample& imu, const MagSample& mag, float dt) {
    const float gx = imu.gyro_x * DEG2RAD;
    const float gy = imu.gyro_y * DEG2RAD;
    const float gz = imu.gyro_z * DEG2RAD;
    const bool mag_usable = mag.valid &&
        (mag.mag_x != 0.0f || mag.mag_y != 0.0f || mag.mag_z != 0.0f);
    if (mag_usable) {
        update9DOF(gx, gy, gz, imu.accel_x, imu.accel_y, imu.accel_z,
                   mag.mag_x, mag.mag_y, mag.mag_z, dt);
    } else {
        update6DOF(gx, gy, gz, imu.accel_x, imu.accel_y, imu.accel_z, dt);
    }
    quaternionToEuler();
}

void AhrsMahony::update6DOF(float gx, float gy, float gz,
                            float ax, float ay, float az, float dt) {
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
    integrateQuaternion(gx, gy, gz, dt);
}

void AhrsMahony::integrateQuaternion(float gx, float gy, float gz, float dt) {
    gx *= 0.5f * dt; gy *= 0.5f * dt; gz *= 0.5f * dt;
    const float qa = q0_, qb = q1_, qc = q2_;
    q0_ += (-qb * gx - qc * gy - q3_ * gz);
    q1_ += ( qa * gx + qc * gz - q3_ * gy);
    q2_ += ( qa * gy - qb * gz + q3_ * gx);
    q3_ += ( qa * gz + qb * gy - qc * gx);
    const float recipNorm = 1.0f / std::sqrt(q0_*q0_ + q1_*q1_ + q2_*q2_ + q3_*q3_);
    q0_ *= recipNorm; q1_ *= recipNorm; q2_ *= recipNorm; q3_ *= recipNorm;
}

void AhrsMahony::update9DOF(float gx, float gy, float gz,
                            float ax, float ay, float az,
                            float mx, float my, float mz, float dt) {
    const float alen2 = ax * ax + ay * ay + az * az;
    if (alen2 > 0.0f) {
        float recipNorm = 1.0f / std::sqrt(alen2);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        recipNorm = 1.0f / std::sqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm; my *= recipNorm; mz *= recipNorm;

        const float q0q0 = q0_*q0_, q0q1 = q0_*q1_, q0q2 = q0_*q2_, q0q3 = q0_*q3_;
        const float q1q1 = q1_*q1_, q1q2 = q1_*q2_, q1q3 = q1_*q3_;
        const float q2q2 = q2_*q2_, q2q3 = q2_*q3_, q3q3 = q3_*q3_;

        const float hx = 2.0f * (mx*(0.5f-q2q2-q3q3) + my*(q1q2-q0q3) + mz*(q1q3+q0q2));
        const float hy = 2.0f * (mx*(q1q2+q0q3) + my*(0.5f-q1q1-q3q3) + mz*(q2q3-q0q1));
        const float bx = std::sqrt(hx*hx + hy*hy);
        const float bz = 2.0f * (mx*(q1q3-q0q2) + my*(q2q3+q0q1) + mz*(0.5f-q1q1-q2q2));

        const float halfvx = q1q3 - q0q2;
        const float halfvy = q0q1 + q2q3;
        const float halfvz = q0q0 - 0.5f + q3q3;
        const float halfwx = bx*(0.5f-q2q2-q3q3) + bz*(q1q3-q0q2);
        const float halfwy = bx*(q1q2-q0q3) + bz*(q0q1+q2q3);
        const float halfwz = bx*(q0q2+q1q3) + bz*(0.5f-q1q1-q2q2);

        const float halfex = (ay*halfvz - az*halfvy) + (my*halfwz - mz*halfwy);
        const float halfey = (az*halfvx - ax*halfvz) + (mz*halfwx - mx*halfwz);
        const float halfez = (ax*halfvy - ay*halfvx) + (mx*halfwy - my*halfwx);

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
    integrateQuaternion(gx, gy, gz, dt);
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
