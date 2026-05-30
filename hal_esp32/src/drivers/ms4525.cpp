#include "drivers/ms4525.h"
#if WP_HAS_AIRSPEED

namespace wp {

bool Ms4525::readFour(uint8_t buf[4]) {
    // MS4525DO 是"读即测"：直接 requestFrom 4 字节，无需先写寄存器地址。
    uint8_t got = bus_.requestFrom((int)addr_, 4);
    if (got != 4) return false;
    for (int i = 0; i < 4; ++i) buf[i] = bus_.read();
    return true;
}

bool Ms4525::probe() {
    uint8_t d[4];
    if (!readFour(d)) return false;
    uint8_t status = (d[0] & 0xC0) >> 6;
    return status != 3;   // 3=fault；0/1/2 认为器件在
}

bool Ms4525::read(AirspeedSample& out) {
    uint8_t d[4];
    if (!readFour(d)) return false;
    AirspeedSample s = ms4525ToSample(d);   // ⚠️ 未扣零点偏置，标定留待实物
    if (!s.valid) return false;             // 契约：坏数据返回 false，out 不变
    out = s;
    return true;
}

}  // namespace wp
#endif  // WP_HAS_AIRSPEED
