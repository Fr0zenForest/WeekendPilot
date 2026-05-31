#pragma once
#include "config/capabilities.h"
#if WP_HAS_BARO
#include "sensors/sensor_interfaces.h"
#include "sensors/bmp390_compensate.h"
#include "sensors/baro_altitude.h"
#include <Wire.h>

namespace wp {

// IBarometer over I2C（共享主总线）。init 读 trim + 配置 normal 模式；
// read() 突发 6 字节 -> 补偿 -> 相对高度。首个有效读数锁基准气压（起飞前置零）。
class Bmp390 : public IBarometer {
public:
    explicit Bmp390(TwoWire& bus, uint8_t addr = bmp390::kI2cAddr)
        : bus_(bus), addr_(addr) {}
    bool probe() override;   // CHIPID == 0x60(BMP390) 或 0x50(BMP388，兼容)
    bool init()  override;   // 软复位 + 读 trim + 配置
    bool read(BaroSample& out) override;
private:
    TwoWire& bus_;
    uint8_t  addr_;
    Bmp390Calib calib_{};
    BaroAltitude alt_;
    bool ref_set_ = false;
    bool writeReg(uint8_t reg, uint8_t val);
    bool readRegs(uint8_t reg, uint8_t* buf, uint8_t n);
};

}  // namespace wp
#endif  // WP_HAS_BARO
