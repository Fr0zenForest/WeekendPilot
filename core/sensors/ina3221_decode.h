#pragma once
#include <cstdint>

namespace wp {

// INA3221 shunt-voltage 寄存器（每通道一个，0x01/0x03/0x05）。
// 格式：13-bit 有符号，存于 bit15..3（bit2..0 恒 0）。LSB = 40µV（datasheet §8.6.1）。
// raw16 = I2C 读到的 16-bit 大端寄存器值（高字节先到，调用方组装为 uint16）。
// 返回 shunt 电压（V）。
float ina3221ShuntVolt(uint16_t raw16);

// shunt 电压（V）+ shunt 电阻（Ω）-> 电流（A）。I = Vshunt / Rshunt。
float ina3221Current(uint16_t raw16, float shunt_ohm);

}  // namespace wp
