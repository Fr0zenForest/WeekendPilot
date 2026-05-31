#include "drivers/icm42688.h"
#if WP_HAS_IMU
#include <Arduino.h>

namespace wp {

bool Icm42688::writeReg(uint8_t reg, uint8_t val) {
    bus_.beginTransmission(addr_);
    bus_.write(reg); bus_.write(val);
    return bus_.endTransmission() == 0;
}

bool Icm42688::readRegs(uint8_t reg, uint8_t* buf, uint8_t n) {
    bus_.beginTransmission(addr_);
    bus_.write(reg);
    if (bus_.endTransmission(false) != 0) return false;
    if (bus_.requestFrom((int)addr_, (int)n) != n) return false;
    for (uint8_t i = 0; i < n; ++i) buf[i] = bus_.read();
    return true;
}

bool Icm42688::probe() {
    uint8_t who = 0;
    if (!readRegs(icm42688::kRegWhoAmI, &who, 1)) return false;
    return who == icm42688::kWhoAmIVal;
}

bool Icm42688::init() {
    if (!writeReg(icm42688::kRegDeviceConfig, 0x01)) return false;  // 软复位
    delay(10);
    uint8_t who = 0;
    if (!readRegs(icm42688::kRegWhoAmI, &who, 1) || who != icm42688::kWhoAmIVal) return false;
    // datasheet §14.36：滤波/FSR/ODR 必须在 PWR_MGMT0 上电前配（ElrsRX 同款顺序）
    if (!writeReg(icm42688::kRegGyroConfig0, 0x06)) return false;        // ±2000dps,1kHz
    if (!writeReg(icm42688::kRegAccelConfig0, (0x1 << 5) | 0x06)) return false;  // ±8g,1kHz
    if (!writeReg(icm42688::kRegGyroAccelConfig0, (0x4 << 4) | 0x4)) return false; // BW≈ODR/4
    if (!writeReg(icm42688::kRegPwrMgmt0, 0x0F)) return false;           // 陀螺+加速度低噪声
    delay(50);   // 陀螺上电后需 ≥30ms 稳定（datasheet §3.1）
    return true;
}

bool Icm42688::read(ImuSample& out) {
    uint8_t raw[14];
    if (!readRegs(icm42688::kRegTempData1, raw, 14)) return false;  // 失败前不碰 out
    ImuSample s = icm42688Decode(raw);   // ⚠️ 轴向极性须装机后核对
    if (!s.valid) return false;          // 契约：无效不污染 out
    out = s;
    return true;
}

}  // namespace wp
#endif  // WP_HAS_IMU
