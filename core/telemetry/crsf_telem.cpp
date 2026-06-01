#include "telemetry/crsf_telem.h"

namespace wp {

uint8_t crc8_dvbs2(const uint8_t* data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b)
            crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0xD5)
                               : static_cast<uint8_t>(crc << 1);
    }
    return crc;
}

int buildFrame(uint8_t addr, uint8_t type, const uint8_t* payload, int plen,
               uint8_t* out, int cap) {
    if (plen < 0) return 0;
    const int total = plen + 4;          // addr+len+type+payload+crc
    if (cap < total || total > kCrsfMaxFrame) return 0;
    out[0] = addr;
    out[1] = static_cast<uint8_t>(plen + 2);   // len = type+payload+crc
    out[2] = type;
    for (int i = 0; i < plen; ++i) out[3 + i] = payload[i];
    // crc 覆盖 type..payload = out[2 .. 2+1+plen)
    out[3 + plen] = crc8_dvbs2(out + 2, static_cast<size_t>(plen + 1));
    return total;
}

}  // namespace wp
