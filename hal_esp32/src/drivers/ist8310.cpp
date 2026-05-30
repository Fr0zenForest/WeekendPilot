#include "drivers/ist8310.h"
#if WP_HAS_MAG
#include <Arduino.h>

namespace wp {

bool Ist8310::writeReg(uint8_t reg, uint8_t val) {
    bus_.beginTransmission(addr_);
    bus_.write(reg); bus_.write(val);
    return bus_.endTransmission() == 0;
}

bool Ist8310::readRegs(uint8_t reg, uint8_t* buf, uint8_t n) {
    bus_.beginTransmission(addr_);
    bus_.write(reg);
    if (bus_.endTransmission(false) != 0) return false;  // repeated start
    uint8_t got = bus_.requestFrom((int)addr_, (int)n);
    if (got != n) return false;
    for (uint8_t i = 0; i < n; ++i) buf[i] = bus_.read();
    return true;
}

bool Ist8310::probe() {
    uint8_t who = 0;
    if (!readRegs(ist8310::kRegWhoAmI, &who, 1)) return false;
    return who == ist8310::kWhoAmIVal;
}

bool Ist8310::init() {
    if (!writeReg(ist8310::kRegCntl2, 0x01)) return false;  // 软复位 (CNTL2_VAL_SRST)
    delay(10);
    // AVGCNTL = AVGCNTL_VAL_Y_16 | AVGCNTL_VAL_XZ_16 = (4<<3)|4 = 0x24（ArduPilot）
    writeReg(ist8310::kRegAvg,    0x24);
    writeReg(ist8310::kRegPdcntl, 0xC0);   // PDCNTL_VAL_PULSE_DURATION_NORMAL（ArduPilot）
    return true;
}

bool Ist8310::read(MagSample& out) {
    if (!writeReg(ist8310::kRegCntl1, 0x01)) return false;  // 触发单次测量
    delay(10);                                               // ArduPilot SAMPLING_PERIOD_USEC = 10ms
    uint8_t raw[6];
    if (!readRegs(ist8310::kRegDataXL, raw, 6)) return false;
    out = ist8310Decode(raw);   // ⚠️ 装配方向标定留待实物
    return out.valid;
}

}  // namespace wp
#endif  // WP_HAS_MAG
