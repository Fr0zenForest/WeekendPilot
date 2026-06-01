#pragma once
#include "config/capabilities.h"
#if WP_HAS_PWM_EXPANDER
#include "io/servo_output.h"
#include <Wire.h>

namespace wp {

// PCA9685 16 路 12-bit PWM 扩展（I2C）。全局单一频率（24~1526Hz）。
// 寄存器序列参考 NXP PCA9685 datasheet §7 + RobTillaart/PCA9685_RT（provenance）。
class Pca9685Output : public IServoOutput {
public:
    Pca9685Output(TwoWire& wire, uint8_t addr);
    bool begin() override;                         // probe（读 MODE1）+ 设频率
    int  channelCount() const override { return 16; }
    void setFrequencyHz(uint16_t hz) override;     // 写 PRE_SCALE（需先进睡眠）
    void writeUs(int ch, uint16_t us) override;    // us -> 12-bit tick -> LEDn_ON/OFF

private:
    bool writeReg(uint8_t reg, uint8_t val);
    bool readReg(uint8_t reg, uint8_t& val);
    TwoWire& wire_;
    uint8_t  addr_;
    uint16_t freq_hz_ = 50;
    bool     ok_ = false;
};

}  // namespace wp
#endif  // WP_HAS_PWM_EXPANDER
