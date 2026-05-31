#include "sensors/icm42688_decode.h"

namespace wp {

ImuSample icm42688Decode(const uint8_t raw[14]) {
    auto be16 = [](uint8_t hi, uint8_t lo) -> int16_t {
        return static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) |
                                    static_cast<uint16_t>(lo));
    };
    ImuSample s{};
    s.accel_x = be16(raw[2],  raw[3])  * icm42688::kAccelScale;
    s.accel_y = be16(raw[4],  raw[5])  * icm42688::kAccelScale;
    s.accel_z = be16(raw[6],  raw[7])  * icm42688::kAccelScale;
    s.gyro_x  = be16(raw[8],  raw[9])  * icm42688::kGyroScale;
    s.gyro_y  = be16(raw[10], raw[11]) * icm42688::kGyroScale;
    s.gyro_z  = be16(raw[12], raw[13]) * icm42688::kGyroScale;
    s.valid = true;
    return s;
}

}  // namespace wp
