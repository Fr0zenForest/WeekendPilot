#include "sensors/ist8310_decode.h"

namespace wp {

MagSample ist8310Decode(const uint8_t raw[6]) {
    auto le16 = [](uint8_t lo, uint8_t hi) -> int16_t {
        return static_cast<int16_t>(static_cast<uint16_t>(lo) |
                                    (static_cast<uint16_t>(hi) << 8));
    };
    int16_t x = le16(raw[0], raw[1]);
    int16_t y = le16(raw[2], raw[3]);
    int16_t z = le16(raw[4], raw[5]);
    z = static_cast<int16_t>(-z);   // ⚠️ 右手系翻转，与 ArduPilot 一致
    MagSample m{};
    m.mag_x = x * ist8310::kMilliGaussPerLsb;
    m.mag_y = y * ist8310::kMilliGaussPerLsb;
    m.mag_z = z * ist8310::kMilliGaussPerLsb;
    m.valid = true;
    return m;
}

}  // namespace wp
