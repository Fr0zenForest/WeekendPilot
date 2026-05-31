#pragma once
#include <cstdint>
#include <cstddef>
#include "types.h"   // kNumServos

namespace wp {

// WARNING 新代码。调试状态快照（值类型，由板载每帧组装；core 不碰硬件）。
struct StatusSnapshot {
    uint32_t t_ms = 0;
    uint8_t mode = 0;            // FlightMode 值
    float roll_deg = 0, pitch_deg = 0, yaw_deg = 0;
    bool link_ok = false;
    bool imu_valid = false, baro_valid = false, mag_valid = false;
    bool althold_engaged = false;
    bool autotrim_learning = false;
    uint16_t servo[kNumServos] = {0};
    int8_t tier = 0;            // SensorTier 值
};

// 状态行缓冲上限（含结尾 NUL）。一行紧凑文本，够放所有字段。
constexpr size_t kStatusLineCap = 160;

// 把 snapshot 格式化成一行紧凑文本写进 out（snprintf 语义，必 NUL 收尾）。
// 返回 snprintf 返回值（欲写字符数，不含 NUL；>=cap 表示被截断）。
int formatStatusLine(const StatusSnapshot& s, char* out, size_t cap);

}  // namespace wp
