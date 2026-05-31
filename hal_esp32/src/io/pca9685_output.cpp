#include "io/pca9685_output.h"
#if WP_HAS_PWM_EXPANDER

namespace wp {

// 寄存器地址（datasheet §7.3）
static constexpr uint8_t REG_MODE1     = 0x00;
static constexpr uint8_t REG_LED0_ON_L = 0x06;  // 每路 4 字节：ON_L/ON_H/OFF_L/OFF_H
static constexpr uint8_t REG_PRESCALE  = 0xFE;
static constexpr uint8_t MODE1_SLEEP   = 0x10;
static constexpr uint8_t MODE1_AI      = 0x20;  // 自动递增
static constexpr uint8_t MODE1_RESTART = 0x80;

Pca9685Output::Pca9685Output(TwoWire& wire, uint8_t addr) : wire_(wire), addr_(addr) {}

bool Pca9685Output::writeReg(uint8_t reg, uint8_t val) {
    wire_.beginTransmission(addr_);
    wire_.write(reg);
    wire_.write(val);
    return wire_.endTransmission() == 0;
}

bool Pca9685Output::readReg(uint8_t reg, uint8_t& val) {
    wire_.beginTransmission(addr_);
    wire_.write(reg);
    if (wire_.endTransmission(false) != 0) return false;
    if (wire_.requestFrom((int)addr_, 1) != 1) return false;
    val = wire_.read();
    return true;
}

bool Pca9685Output::begin() {
    uint8_t m1 = 0;
    if (!readReg(REG_MODE1, m1)) { ok_ = false; return false; }  // probe：读不到 = 未焊
    if (!writeReg(REG_MODE1, MODE1_AI)) { ok_ = false; return false; }
    setFrequencyHz(freq_hz_);
    ok_ = true;
    return true;
}

void Pca9685Output::setFrequencyHz(uint16_t hz) {
    // prescale = round(25MHz / (4096 * freq)) - 1（datasheet §7.3.5）
    if (hz < 24) hz = 24;
    if (hz > 1526) hz = 1526;
    freq_hz_ = hz;   // 存夹取后的值：writeUs 的周期换算与硬件 prescale 必须一致
    uint8_t prescale = (uint8_t)((25000000.0 / (4096.0 * hz)) - 0.5);
    uint8_t m1 = 0;
    readReg(REG_MODE1, m1);
    writeReg(REG_MODE1, (uint8_t)((m1 & ~MODE1_RESTART) | MODE1_SLEEP));  // 进睡眠才能写 prescale
    writeReg(REG_PRESCALE, prescale);
    writeReg(REG_MODE1, (uint8_t)(m1 | MODE1_AI));                         // 唤醒
    delayMicroseconds(500);
    writeReg(REG_MODE1, (uint8_t)(m1 | MODE1_AI | MODE1_RESTART));         // restart
}

void Pca9685Output::writeUs(int ch, uint16_t us) {
    if (!ok_ || ch < 0 || ch >= 16) return;
    if (us < 1000) us = 1000;
    if (us > 2000) us = 2000;
    // us -> tick（12-bit @ freq）：tick = us / (1e6/freq) * 4096
    uint32_t period_us = 1000000UL / freq_hz_;
    uint16_t off = (uint16_t)((uint32_t)us * 4096UL / period_us);
    if (off > 4095) off = 4095;
    uint8_t reg = (uint8_t)(REG_LED0_ON_L + 4 * ch);
    wire_.beginTransmission(addr_);
    wire_.write(reg);
    wire_.write(0x00); wire_.write(0x00);              // ON = 0
    wire_.write((uint8_t)(off & 0xFF));                // OFF_L
    wire_.write((uint8_t)((off >> 8) & 0x0F));         // OFF_H
    wire_.endTransmission();
}

}  // namespace wp
#endif  // WP_HAS_PWM_EXPANDER
