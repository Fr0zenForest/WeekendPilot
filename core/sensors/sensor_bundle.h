#pragma once
#include "types.h"
#include "sensors/sensor_types.h"

namespace wp {

// 喂给 core 的单组干净样本（core 永远只看到这一组，不知背后几个传感器）。
struct SensorBundle {
    ImuSample      imu;
    MagSample      mag;
    BaroSample     baro;
    GnssSample     gnss;
    AirspeedSample airspeed;
};

}  // namespace wp
