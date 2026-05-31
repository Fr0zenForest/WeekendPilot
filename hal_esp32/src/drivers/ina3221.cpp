#include "drivers/ina3221.h"
#if WP_HAS_LANDING_GEAR
#include "sensors/ina3221_decode.h"

namespace wp {

// shunt 电压寄存器：ch0=0x01, ch1=0x03, ch2=0x05（datasheet §8.6）
static constexpr uint8_t REG_SHUNT[3] = {0x01, 0x03, 0x05};
static constexpr uint8_t REG_MANUF_ID = 0xFE;   // 应读回 0x5449 ('TI')

bool Ina3221::readReg(uint8_t reg, uint16_t& val) {
    wire_.beginTransmission(addr_);
    wire_.write(reg);
    if (wire_.endTransmission(false) != 0) return false;
    if (wire_.requestFrom((int)addr_, 2) != 2) return false;
    uint8_t hi = wire_.read();
    uint8_t lo = wire_.read();
    val = (uint16_t)((hi << 8) | lo);   // 大端
    return true;
}

bool Ina3221::begin() {
    uint16_t id = 0;
    ok_ = readReg(REG_MANUF_ID, id) && (id == 0x5449);
    return ok_;
}

bool Ina3221::readCurrent(int ch, float shunt_ohm, float& out_a) {
    if (!ok_ || ch < 0 || ch > 2) return false;
    uint16_t raw = 0;
    if (!readReg(REG_SHUNT[ch], raw)) return false;
    out_a = ina3221Current(raw, shunt_ohm);
    return true;
}

}  // namespace wp
#endif  // WP_HAS_LANDING_GEAR
