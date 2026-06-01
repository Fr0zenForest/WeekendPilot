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

static void put_i16_be(uint8_t* p, int16_t v) {
    p[0] = static_cast<uint8_t>((v >> 8) & 0xFF);
    p[1] = static_cast<uint8_t>(v & 0xFF);
}

static int16_t rad_to_i16(float rad) {
    float scaled = rad * 10000.0f;
    if (scaled >  32767.0f) scaled =  32767.0f;   // 饱和防溢出
    if (scaled < -32768.0f) scaled = -32768.0f;
    return static_cast<int16_t>(scaled > 0 ? scaled + 0.5f : scaled - 0.5f);
}

int encodeAttitude(float roll_rad, float pitch_rad, float yaw_rad,
                   uint8_t* out, int cap) {
    uint8_t payload[6];
    put_i16_be(payload + 0, rad_to_i16(roll_rad));
    put_i16_be(payload + 2, rad_to_i16(pitch_rad));
    put_i16_be(payload + 4, rad_to_i16(yaw_rad));
    return buildFrame(kCrsfSyncAddr, 0x1E, payload, 6, out, cap);
}

int encodeFlightMode(const char* mode, uint8_t* out, int cap) {
    uint8_t payload[16];
    int n = 0;
    while (mode && mode[n] != '\0' && n < 15) { payload[n] = static_cast<uint8_t>(mode[n]); ++n; }
    payload[n++] = 0x00;   // NUL 结尾
    return buildFrame(kCrsfSyncAddr, 0x21, payload, n, out, cap);
}

int encodeBattery(float volts, float amps, uint32_t mah, uint8_t pct,
                  uint8_t* out, int cap) {
    int16_t v = static_cast<int16_t>(volts * 10.0f + 0.5f);   // 0.1V
    int16_t a = static_cast<int16_t>(amps  * 10.0f + 0.5f);   // 0.1A
    uint8_t payload[8];
    put_i16_be(payload + 0, v);
    put_i16_be(payload + 2, a);
    payload[4] = static_cast<uint8_t>((mah >> 16) & 0xFF);    // uint24 大端
    payload[5] = static_cast<uint8_t>((mah >> 8) & 0xFF);
    payload[6] = static_cast<uint8_t>(mah & 0xFF);
    payload[7] = pct;
    return buildFrame(kCrsfSyncAddr, 0x08, payload, 8, out, cap);
}

}  // namespace wp
