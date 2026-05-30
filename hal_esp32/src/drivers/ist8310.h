#pragma once
#include "config/capabilities.h"
#if WP_HAS_MAG
#include "sensors/sensor_interfaces.h"
#include "sensors/ist8310_decode.h"
#include <Wire.h>

namespace wp {

// IMagnetometer over I2C（共享主总线）。单次测量模式，read() 触发+取回。
class Ist8310 : public IMagnetometer {
public:
    explicit Ist8310(TwoWire& bus, uint8_t addr = ist8310::kI2cAddr)
        : bus_(bus), addr_(addr) {}
    bool probe() override;    // 读 WHO_AM_I == 0x10
    bool init()  override;    // 软复位 + 配置过采样
    bool read(MagSample& out) override;  // 触发单次 + 读 6 字节 + decode
private:
    TwoWire& bus_;
    uint8_t  addr_;
    bool writeReg(uint8_t reg, uint8_t val);
    bool readRegs(uint8_t reg, uint8_t* buf, uint8_t n);
};

}  // namespace wp
#endif  // WP_HAS_MAG
