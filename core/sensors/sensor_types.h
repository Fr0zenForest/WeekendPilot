#pragma once
#include <cstdint>

namespace wp {

// 传感器档位（有序：缺高档传感器自动降级，见设计 §6.0）。
enum class SensorTier : uint8_t { None = 0, Base = 1, Plus = 2, Pro = 3, Max = 4 };

// GPS 标准样本（plus 档）。vel_ned: m/s，北/东/地。
struct GnssSample {
    double lat = 0.0, lon = 0.0;
    float  vel_ned[3] = {0.0f, 0.0f, 0.0f};
    float  ground_speed_mps = 0.0f;
    uint8_t fix = 0;        // 0=no fix
    uint8_t sats = 0;
    bool   valid = false;
};

// 空速标准样本（pro 档）。
struct AirspeedSample {
    float ias_mps = 0.0f;   // 指示空速
    bool  valid = false;
};

}  // namespace wp
