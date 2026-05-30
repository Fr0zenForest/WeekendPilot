#pragma once
#include "types.h"
#include <cstdint>

namespace wp {

// IST8310 寄存器（移植自 ArduPilot AP_Compass_IST8310 / INAV，⚠️ 非数据手册）。
namespace ist8310 {
    constexpr uint8_t kI2cAddr   = 0x0E;
    constexpr uint8_t kRegWhoAmI = 0x00;   // 应读回 0x10
    constexpr uint8_t kWhoAmIVal = 0x10;
    constexpr uint8_t kRegStat1  = 0x02;
    constexpr uint8_t kRegDataXL = 0x03;   // X_L,X_H,Y_L,Y_H,Z_L,Z_H 连续 6 字节
    constexpr uint8_t kRegCntl1  = 0x0A;   // 0x01=单次测量
    constexpr uint8_t kRegCntl2  = 0x0B;   // 0x01=软复位
    constexpr uint8_t kRegAvg    = 0x41;   // 过采样/平均
    constexpr uint8_t kRegPdcntl = 0x42;   // pulse duration

    // ⚠️ 灵敏度：0.3 µT/LSB（数据手册）→ ArduPilot 直接乘 3.0 得毫高斯。
    // AHRS 内部对 mag 归一化，标度只影响相对幅度，不影响 yaw 解算。
    constexpr float kMilliGaussPerLsb = 3.0f;
}

// 把 X_L..Z_H 共 6 字节小端有符号原始值解码为 MagSample（机体系，毫高斯）。
// ⚠️ Z 轴翻转（z=-z）以符右手系——与 ArduPilot AP_Compass_IST8310 一致。
// 不做装配旋转——装配方向在 hal 层按板子定向处理（见 plan）。out.valid=true。
MagSample ist8310Decode(const uint8_t raw[6]);

}  // namespace wp
