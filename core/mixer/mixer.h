#pragma once
#include <cstdint>
#include "types.h"

namespace wp {

enum class MixSource : uint8_t { Roll = 0, Pitch = 1, Yaw = 2, Throttle = 3, Flap = 4, Count = 5 };
enum class Airframe  : uint8_t { Standard = 0, Flaperon = 1, VTail = 2, Elevon = 3 };

struct MixRule {
    uint8_t   out;       // 输出 servo 索引 [0,kNumServos)
    MixSource src;       // 取哪一路轴需求
    float     weight;    // 权重（含符号）
};

constexpr int kMaxMixRules = 24;  // 余量：最密的 Flaperon 用 6 条，预留自定义混控

class Mixer {
public:
    Mixer() { setAirframe(Airframe::Standard); }
    void setAirframe(Airframe af);

    // src 下标用 MixSource：roll/pitch/yaw ∈ [-1,1]，throttle/flap ∈ [0,1]。
    // 输出写入 servo[kNumServos]（us 值）。
    void mix(const float src[static_cast<int>(MixSource::Count)],
             uint16_t servo[kNumServos]) const;

private:
    MixRule rules_[kMaxMixRules];
    uint8_t num_rules_ = 0;
    bool    is_throttle_[kNumServos] = {};
};

}  // namespace wp
