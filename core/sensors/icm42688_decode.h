#pragma once
#include "types.h"
#include <cstdint>

namespace wp {

// ICM-42688-P 寄存器（移植自 ElrsRX imu_icm42688.cpp，⚠️ 配置 ±2000dps/±8g/1kHz）。
namespace icm42688 {
    constexpr uint8_t kI2cAddr   = 0x68;
    constexpr uint8_t kRegWhoAmI = 0x75;
    constexpr uint8_t kWhoAmIVal = 0x47;
    constexpr uint8_t kRegDeviceConfig = 0x11;  // 0x01=软复位
    constexpr uint8_t kRegPwrMgmt0     = 0x4E;  // 0x0F=陀螺+加速度低噪声
    constexpr uint8_t kRegGyroConfig0  = 0x4F;  // 0x06=±2000dps,1kHz
    constexpr uint8_t kRegAccelConfig0 = 0x50;  // (0x1<<5)|0x06=±8g,1kHz
    constexpr uint8_t kRegGyroAccelConfig0 = 0x52;  // (0x4<<4)|0x4=BW≈ODR/4
    constexpr uint8_t kRegTempData1    = 0x1D;  // 突发起点: TEMP(2)+ACCEL(6)+GYRO(6)=14B

    constexpr float kGyroScale  = 1.0f / 16.4f;    // ±2000dps -> 16.4 LSB/(°/s)
    constexpr float kAccelScale = 1.0f / 4096.0f;  // ±8g -> 4096 LSB/g
}

// 把从 TEMP_DATA1 突发读的 14 字节解码为 ImuSample（gyro deg/s, accel g, 机体系）。
// 布局：[0..1]=temp(跳过) [2..7]=accel XYZ(hi,lo) [8..13]=gyro XYZ(hi,lo)。
// out.valid=true。⚠️ 轴向极性须装机后核对（X前/Y右/Z下 NED），见 plan。
ImuSample icm42688Decode(const uint8_t raw[14]);

}  // namespace wp
