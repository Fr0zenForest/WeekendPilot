#pragma once
#include <cstdint>

namespace wp {

// 16 路 RC 通道，标准 CRSF 范围 [1000,2000]，中位 1500
constexpr int kNumChannels = 16;
constexpr int kNumServos = 8;          // 阶段 0~1 固定 8 路

struct ImuSample {
    float gyro_x, gyro_y, gyro_z;      // deg/s（机体系）
    float accel_x, accel_y, accel_z;   // g
    bool  valid = false;
};

struct BaroSample {
    float altitude_m = 0.0f;           // 相对高度（m）
    bool  valid = false;
};

struct MagSample {
    float mag_x = 0.0f, mag_y = 0.0f, mag_z = 0.0f;  // 机体系，任意单位（AHRS 内部归一化）
    bool  valid = false;
};

struct ControlInput {
    uint16_t channels[kNumChannels];   // RC us 值 [1000,2000]
    ImuSample imu;
    MagSample mag;
    BaroSample baro;
    float dt;                          // 距上次 update 的秒数
    bool  link_ok = true;              // CRSF 链路有效
};

struct ServoCommand {
    uint16_t servo[kNumServos];        // 输出 us 值 [1000,2000]
};

struct Attitude {
    float roll_deg = 0.0f;
    float pitch_deg = 0.0f;
    float yaw_deg = 0.0f;
};

}  // namespace wp
