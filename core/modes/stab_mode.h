#pragma once
#include <cstdint>

namespace wp {

enum class FlightMode : uint8_t { Off = 0, Angle = 1, Rate = 2 };

// 三段开关阈值（CRSF us）。<1300 Off，1300~1700 Angle，>1700 Rate。
constexpr uint16_t kModeAngleThresh = 1300;
constexpr uint16_t kModeRateThresh  = 1700;

inline FlightMode modeFromChannel(uint16_t us) {
    if (us > kModeRateThresh) return FlightMode::Rate;
    if (us > kModeAngleThresh) return FlightMode::Angle;
    return FlightMode::Off;
}

// 通道 us[1000,2000] 中位 1500 -> 归一化 [-1,1]
inline float channelToNorm(uint16_t us) {
    float n = (static_cast<float>(us) - 1500.0f) / 500.0f;
    if (n > 1.0f) n = 1.0f;
    if (n < -1.0f) n = -1.0f;
    return n;
}

// 归一化舵量 [-1,1] -> us[1000,2000]
inline uint16_t normToServoUs(float n) {
    if (n > 1.0f) n = 1.0f;
    if (n < -1.0f) n = -1.0f;
    return static_cast<uint16_t>(1500.0f + n * 500.0f);
}

// 增益通道 us[1000,2000] -> [0,1]
inline float gainFromChannel(uint16_t us) {
    float g = (static_cast<float>(us) - 1000.0f) / 1000.0f;
    if (g > 1.0f) g = 1.0f;
    if (g < 0.0f) g = 0.0f;
    return g;
}

}  // namespace wp
