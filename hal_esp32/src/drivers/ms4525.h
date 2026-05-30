#pragma once
#include "config/capabilities.h"
#if WP_HAS_AIRSPEED
#include "sensors/sensor_interfaces.h"
#include "sensors/ms4525_decode.h"
#include <Wire.h>

namespace wp {

// IAirspeed over I2C（共享主总线）。MS4525DO 无需配置寄存器：上电即测，
// 直接读 4 字节。probe 读一次看 status 合法。
class Ms4525 : public IAirspeed {
public:
    explicit Ms4525(TwoWire& bus, uint8_t addr = ms4525::kI2cAddr)
        : bus_(bus), addr_(addr) {}
    bool probe() override;   // 读 4 字节，status 合法即认在
    bool init()  override { return true; }   // 无寄存器配置
    bool read(AirspeedSample& out) override;  // 读 4 字节 -> decode -> 伯努利
private:
    TwoWire& bus_;
    uint8_t  addr_;
    bool readFour(uint8_t buf[4]);
};

}  // namespace wp
#endif  // WP_HAS_AIRSPEED
