#pragma once
#include "types.h"

namespace wp {

class Controller {
public:
    // 阶段 0：纯直通。阶段 1 接入 AHRS/PID/模式。
    ServoCommand update(const ControlInput& in);
};

}  // namespace wp
