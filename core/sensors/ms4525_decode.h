#pragma once
#include "sensors/sensor_types.h"
#include <cstdint>

namespace wp {

namespace ms4525 {
    constexpr uint8_t kI2cAddr = 0x28;     // 备选 0x36 / 0x46（ArduPilot）
    constexpr float   kPsiRange = 1.0f;    // MS4525DO 1 psi
    constexpr float   kPsiToPa  = 6894.757f;
    constexpr float   kRhoSeaLevel = 1.225f;  // ⚠️ 标准海平面密度，实物须按高度/温度修正
}

// 解码结果：原始物理量（差压 Pa + 温度 °C）+ 是否坏数据。
struct Ms4525Raw {
    float diff_press_pa = 0.0f;
    float temperature_c = 0.0f;
    bool  valid = false;   // status 位正常且不在饱和极值
};

// 把一次读到的 4 字节解码为差压(Pa)+温度(°C)。
// status=(data[0]&0xC0)>>6，==2/3 视为坏数据 valid=false。
// dp_raw=14位，dT_raw=11位。公式与 ArduPilot 逐字对齐。
Ms4525Raw ms4525Decode(const uint8_t data[4]);

// 差压(Pa) → 指示空速(m/s)，伯努利 v=sqrt(2|Δp|/ρ)。负差压取绝对值（管接反/噪声）。
float ms4525PaToIas(float diff_press_pa);

// 便捷：4 字节 → AirspeedSample（合成 decode + 伯努利）。坏数据 valid=false。
AirspeedSample ms4525ToSample(const uint8_t data[4]);

}  // namespace wp
