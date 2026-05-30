#include "mixer/mixer.h"
#include "modes/stab_mode.h"   // normToServoUs

namespace wp {

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

void Mixer::setAirframe(Airframe af) {
    for (int i = 0; i < kNumServos; ++i) is_throttle_[i] = false;
    num_rules_ = 0;
    num_outputs_ = kNumServos;
    auto add = [&](uint8_t out, MixSource src, float w) {
        rules_[num_rules_++] = MixRule{out, src, w};
    };
    switch (af) {
        case Airframe::Standard:
        default:
            add(0, MixSource::Roll,     +1.0f);
            add(1, MixSource::Pitch,    +1.0f);
            add(2, MixSource::Throttle, +1.0f); is_throttle_[2] = true;
            add(3, MixSource::Yaw,      +1.0f);
            break;
    }
}

void Mixer::mix(const float src[static_cast<int>(MixSource::Count)],
                uint16_t servo[kNumServos]) const {
    float acc[kNumServos] = {0};
    for (int i = 0; i < num_rules_; ++i) {
        const MixRule& r = rules_[i];
        acc[r.out] += r.weight * src[static_cast<int>(r.src)];
    }
    for (int i = 0; i < kNumServos; ++i) {
        if (is_throttle_[i]) {
            servo[i] = static_cast<uint16_t>(1000.0f + 1000.0f * clampf(acc[i], 0.0f, 1.0f));
        } else {
            servo[i] = normToServoUs(clampf(acc[i], -1.0f, 1.0f));
        }
    }
}

}  // namespace wp
