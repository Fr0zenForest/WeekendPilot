#include "controller.h"

namespace wp {

ServoCommand Controller::update(const ControlInput& in) {
    ServoCommand out{};
    for (int i = 0; i < kNumServos; ++i) {
        out.servo[i] = in.channels[i];   // 直通：通道 i -> 舵机 i
    }
    return out;
}

}  // namespace wp
