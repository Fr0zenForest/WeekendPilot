#include "blackbox/blackbox_frame.h"
#include <cstring>

namespace wp {

namespace {
void putU32(uint8_t*& p, uint32_t v) {
    p[0] = uint8_t(v); p[1] = uint8_t(v >> 8);
    p[2] = uint8_t(v >> 16); p[3] = uint8_t(v >> 24); p += 4;
}
void putU16(uint8_t*& p, uint16_t v) { p[0] = uint8_t(v); p[1] = uint8_t(v >> 8); p += 2; }
void putF(uint8_t*& p, float v) { uint32_t u; std::memcpy(&u, &v, 4); putU32(p, u); }
uint32_t getU32(const uint8_t*& p) {
    uint32_t v = uint32_t(p[0]) | (uint32_t(p[1]) << 8) |
                 (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24); p += 4; return v;
}
uint16_t getU16(const uint8_t*& p) {
    uint16_t v = uint16_t(uint16_t(p[0]) | (uint16_t(p[1]) << 8)); p += 2; return v;
}
float getF(const uint8_t*& p) { uint32_t u = getU32(p); float v; std::memcpy(&v, &u, 4); return v; }
}  // namespace

void encodeFrame(const BlackboxFrame& f, uint8_t* out) {
    uint8_t* p = out;
    putU32(p, f.t_ms);
    putF(p, f.gyro_x); putF(p, f.gyro_y); putF(p, f.gyro_z);
    putF(p, f.accel_x); putF(p, f.accel_y); putF(p, f.accel_z);
    putF(p, f.roll_deg); putF(p, f.pitch_deg); putF(p, f.yaw_deg);
    for (int i = 0; i < kNumServos; ++i) putU16(p, f.servo[i]);
    putF(p, f.baro_alt_m);
    *p++ = f.mode;
    *p++ = f.flags;
}

void decodeFrame(const uint8_t* in, BlackboxFrame& out) {
    const uint8_t* p = in;
    out.t_ms = getU32(p);
    out.gyro_x = getF(p); out.gyro_y = getF(p); out.gyro_z = getF(p);
    out.accel_x = getF(p); out.accel_y = getF(p); out.accel_z = getF(p);
    out.roll_deg = getF(p); out.pitch_deg = getF(p); out.yaw_deg = getF(p);
    for (int i = 0; i < kNumServos; ++i) out.servo[i] = getU16(p);
    out.baro_alt_m = getF(p);
    out.mode = *p++;
    out.flags = *p++;
}

}  // namespace wp
