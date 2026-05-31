#include "drivers/bmp390.h"
#if WP_HAS_BARO
#include <Arduino.h>

namespace wp {

bool Bmp390::writeReg(uint8_t reg, uint8_t val) {
    bus_.beginTransmission(addr_);
    bus_.write(reg); bus_.write(val);
    return bus_.endTransmission() == 0;
}

bool Bmp390::readRegs(uint8_t reg, uint8_t* buf, uint8_t n) {
    bus_.beginTransmission(addr_);
    bus_.write(reg);
    if (bus_.endTransmission(false) != 0) return false;
    if (bus_.requestFrom((int)addr_, (int)n) != n) return false;
    for (uint8_t i = 0; i < n; ++i) buf[i] = bus_.read();
    return true;
}

bool Bmp390::probe() {
    uint8_t id = 0;
    if (!readRegs(bmp390::kRegChipId, &id, 1)) return false;
    return id == bmp390::kChipId;
}

bool Bmp390::init() {
    if (!writeReg(bmp390::kRegCmd, bmp390::kCmdSoftReset)) return false;
    delay(5);
    uint8_t nvm[bmp390::kLenNvm];
    if (!readRegs(bmp390::kRegNvmPar, nvm, bmp390::kLenNvm)) return false;
    calib_ = bmp390ParseCalib(nvm);
    // ⚠️ 无人机预设移植自 ElrsRX/datasheet §3.4.5，未在本硬件实测：
    //    OSR t×1 p×8，ODR 50Hz，IIR coeff3
    if (!writeReg(bmp390::kRegOsr, (0x0 << 3) | 0x3)) return false;  // x1 / x8
    if (!writeReg(bmp390::kRegOdr, 0x02)) return false;             // 50 Hz
    if (!writeReg(bmp390::kRegConfig, (0x2 << 1))) return false;    // IIR coeff 3
    // press_en|temp_en|normal(0x3<<4)
    if (!writeReg(bmp390::kRegPwrCtrl, (1u<<0)|(1u<<1)|(0x3<<4))) return false;
    delay(10);
    return true;
}

bool Bmp390::read(BaroSample& out) {
    uint8_t data[bmp390::kLenData];
    if (!readRegs(bmp390::kRegData0, data, bmp390::kLenData)) return false;  // 失败前不碰 out
    Bmp390Reading r = bmp390Decode(data, calib_);
    if (!r.valid) return false;            // 契约：无效不污染 out
    if (!ref_set_) { alt_.setReference(r.pressure_pa); ref_set_ = true; }  // 首读锁基准
    BaroSample s{};
    s.altitude_m = alt_.altitude(r.pressure_pa);
    s.valid = true;
    out = s;
    return true;
}

}  // namespace wp
#endif  // WP_HAS_BARO
