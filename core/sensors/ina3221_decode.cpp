#include "sensors/ina3221_decode.h"

namespace wp {

float ina3221ShuntVolt(uint16_t raw16) {
    // 值在 bit15..3，13-bit 有符号。用 int16_t 算术右移 3 保符号。
    int16_t v = static_cast<int16_t>(raw16);
    v = static_cast<int16_t>(v >> 3);   // 13-bit 有符号，范围 [-4096,4095]
    return static_cast<float>(v) * 40e-6f;   // LSB = 40µV
}

float ina3221Current(uint16_t raw16, float shunt_ohm) {
    if (shunt_ohm <= 0.0f) return 0.0f;   // 防 0 除
    return ina3221ShuntVolt(raw16) / shunt_ohm;
}

}  // namespace wp
