#pragma once
#include "config/capabilities.h"
#if WP_HAS_LANDING_GEAR
#include <cstdint>
#include <Wire.h>

namespace wp {

// INA3221 三路电流监测（I2C）。读 shunt 电压寄存器，经 ina3221_decode 换算电流。
class Ina3221 {
public:
    Ina3221(TwoWire& wire, uint8_t addr) : wire_(wire), addr_(addr) {}
    bool begin();                                  // probe（读 manufacturer ID 0xFE）
    // 读通道 ch（0..2）电流（A）。shunt_ohm 为该路采样电阻。失败返回 false。
    bool readCurrent(int ch, float shunt_ohm, float& out_a);

private:
    bool readReg(uint8_t reg, uint16_t& val);
    TwoWire& wire_;
    uint8_t  addr_;
    bool     ok_ = false;
};

}  // namespace wp
#endif  // WP_HAS_LANDING_GEAR
