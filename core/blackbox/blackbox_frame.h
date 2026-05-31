#pragma once
#include <cstdint>
#include "types.h"   // kNumServos

namespace wp {

// WARNING 新代码（自定义定长小端二进制格式，非 Betaflight BBL）。round-trip 可单测；
//    与 Blackbox Explorer 的兼容性、PSRAM 落盘均未实测。
struct BlackboxFrame {
    uint32_t t_ms = 0;                 // 时间戳 ms
    float gyro_x = 0, gyro_y = 0, gyro_z = 0;    // deg/s
    float accel_x = 0, accel_y = 0, accel_z = 0; // g
    float roll_deg = 0, pitch_deg = 0, yaw_deg = 0;
    uint16_t servo[kNumServos] = {0};  // 输出 us
    float baro_alt_m = 0;
    uint8_t mode = 0;                  // FlightMode 值
    uint8_t flags = 0;                 // bit0 link_ok, bit1 imu_valid, bit2 baro_valid, bit3 althold
};

// 定长字节数：4 + 9*4 + 8*2 + 4 + 1 + 1 = 62
constexpr int kFrameBytes = 4 + 9 * 4 + kNumServos * 2 + 4 + 1 + 1;

// 小端定长编码：out 必须至少 kFrameBytes 字节。
void encodeFrame(const BlackboxFrame& f, uint8_t* out);
// 解码：in 必须至少 kFrameBytes 字节。
void decodeFrame(const uint8_t* in, BlackboxFrame& out);

}  // namespace wp
