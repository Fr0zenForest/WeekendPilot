#include "io/ledc_output.h"
#include <Arduino.h>

namespace wp {

LedcOutput::LedcOutput(const int* pins, int count) : pins_(pins), count_(count) {}

bool LedcOutput::begin() {
    for (int i = 0; i < count_; ++i) {
        ledcSetup(i, freq_hz_, 16);
        ledcAttachPin(pins_[i], i);
        writeUs(i, 1500);   // 上电中位
    }
    return true;   // 无外部硬件，恒可用
}

void LedcOutput::setFrequencyHz(uint16_t hz) {
    if (hz < 24) hz = 24;        // 防 0/过低：writeUs 用 1e6/freq 做周期，0 会除零
    freq_hz_ = hz;
    for (int i = 0; i < count_; ++i) ledcSetup(i, freq_hz_, 16);
}

void LedcOutput::writeUs(int ch, uint16_t us) {
    if (ch < 0 || ch >= count_) return;
    if (us < 1000) us = 1000;
    if (us > 2000) us = 2000;
    // 占空比 = us / 周期；周期 = 1e6/freq µs。沿用现 main.cpp 的 16-bit 满量程换算。
    uint32_t period_us = 1000000UL / freq_hz_;
    uint32_t duty = (uint32_t)((double)us / (double)period_us * 65535.0);
    ledcWrite(ch, duty);
}

}  // namespace wp
