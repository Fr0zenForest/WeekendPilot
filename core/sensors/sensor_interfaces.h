#pragma once
#include "types.h"
#include "sensors/sensor_types.h"

namespace wp {

// 统一驱动契约（设计 §4.2.1）：每个实现给 probe/init/read。
// 接口零 I/O —— core 只认这些接口，不认型号；真驱动在 hal 实现，PC 用 Mock。
// read 返回 false 时 out 内容保持不变（不污染调用方缓冲），调用方应忽略本次读数。
struct IGyroAccel {
    virtual ~IGyroAccel() = default;
    virtual bool probe() = 0;              // WHO_AM_I 在不在
    virtual bool init()  = 0;
    virtual bool read(ImuSample& out) = 0; // gyro deg/s, accel g
};
struct IMagnetometer {
    virtual ~IMagnetometer() = default;
    virtual bool probe() = 0;
    virtual bool init()  = 0;
    virtual bool read(MagSample& out) = 0;
};
struct IBarometer {
    virtual ~IBarometer() = default;
    virtual bool probe() = 0;
    virtual bool init()  = 0;
    virtual bool read(BaroSample& out) = 0;
};
struct IGnss {
    virtual ~IGnss() = default;
    virtual bool probe() = 0;
    virtual bool init()  = 0;
    virtual bool read(GnssSample& out) = 0;
};
struct IAirspeed {
    virtual ~IAirspeed() = default;
    virtual bool probe() = 0;
    virtual bool init()  = 0;
    virtual bool read(AirspeedSample& out) = 0;
};

}  // namespace wp
