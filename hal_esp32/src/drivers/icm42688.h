#pragma once
#include "config/capabilities.h"
#if WP_HAS_IMU
#include "sensors/sensor_interfaces.h"
#include "sensors/icm42688_decode.h"
#include <Wire.h>

namespace wp {

// IGyroAccel over I2C（共享主总线）。1kHz ODR 连续模式，read() 突发 14 字节。
class Icm42688 : public IGyroAccel {
public:
    explicit Icm42688(TwoWire& bus, uint8_t addr = icm42688::kI2cAddr)
        : bus_(bus), addr_(addr) {}
    bool probe() override;   // 读 WHO_AM_I == 0x47
    bool init()  override;   // 软复位 + 配置 + 上电（顺序敏感）
    bool read(ImuSample& out) override;
private:
    TwoWire& bus_;
    uint8_t  addr_;
    bool writeReg(uint8_t reg, uint8_t val);
    bool readRegs(uint8_t reg, uint8_t* buf, uint8_t n);
};

}  // namespace wp
#endif  // WP_HAS_IMU
