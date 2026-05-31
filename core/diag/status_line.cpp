#include "diag/status_line.h"
#include <cstdio>

namespace wp {

namespace {
const char* modeName(uint8_t m) {
    switch (m) {
        case 0: return "OFF";
        case 1: return "ANGLE";
        case 2: return "RATE";
        default: return "?";
    }
}
}  // namespace

int formatStatusLine(const StatusSnapshot& s, char* out, size_t cap) {
    if (out == nullptr || cap == 0) return 0;
    // 一行紧凑格式：
    // [t] MODE r/p/y | LINK ok IMU1 BAR0 MAG1 T<tier> | AH AT | s0 s1 s2 s3
    return snprintf(out, cap,
        "[%lu] %s r%.0f p%.0f y%.0f | LINK%s IMU%d BAR%d MAG%d T%d | %s %s | %u %u %u %u",
        (unsigned long)s.t_ms, modeName(s.mode),
        (double)s.roll_deg, (double)s.pitch_deg, (double)s.yaw_deg,
        s.link_ok ? "ok" : "?", s.imu_valid ? 1 : 0, s.baro_valid ? 1 : 0,
        s.mag_valid ? 1 : 0, (int)s.tier,
        s.althold_engaged ? "AH" : "--", s.autotrim_learning ? "AT" : "--",
        (unsigned)s.servo[0], (unsigned)s.servo[1],
        (unsigned)s.servo[2], (unsigned)s.servo[3]);
}

}  // namespace wp
