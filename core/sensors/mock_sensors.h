#pragma once
#include "sensors/sensor_interfaces.h"

namespace wp {

// 测试/SITL 用：注入合成数据。present 控制 probe 返回，valid 控制 read 成败。
struct MockGyroAccel : IGyroAccel {
    bool present = true;
    ImuSample sample{};
    bool probe() override { return present; }
    bool init()  override { return present; }
    bool read(ImuSample& out) override { if (!present) return false; out = sample; return true; }
};
struct MockMag : IMagnetometer {
    bool present = true;
    MagSample sample{};
    bool probe() override { return present; }
    bool init()  override { return present; }
    bool read(MagSample& out) override { if (!present) return false; out = sample; return true; }
};
struct MockBaro : IBarometer {
    bool present = true;
    BaroSample sample{};
    bool probe() override { return present; }
    bool init()  override { return present; }
    bool read(BaroSample& out) override { if (!present) return false; out = sample; return true; }
};
struct MockGnss : IGnss {
    bool present = true;
    GnssSample sample{};
    bool probe() override { return present; }
    bool init()  override { return present; }
    bool read(GnssSample& out) override { if (!present) return false; out = sample; return true; }
};
struct MockAirspeed : IAirspeed {
    bool present = true;
    AirspeedSample sample{};
    bool probe() override { return present; }
    bool init()  override { return present; }
    bool read(AirspeedSample& out) override { if (!present) return false; out = sample; return true; }
};

}  // namespace wp
