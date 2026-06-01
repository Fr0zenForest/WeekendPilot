#pragma once
#include "io/servo_output.h"

namespace wp {

// 原生 ESP32-S3 LEDC 输出（8 路）。包装现 main.cpp 的 setupPwm/writeServoUs。
// 频率默认 50Hz、16-bit 分辨率（与现行为一致）。begin() 恒成功（无外部硬件可探测）。
class LedcOutput : public IServoOutput {
public:
    // pins: 8 个 GPIO；count: 路数（<=8）。
    LedcOutput(const int* pins, int count);
    bool begin() override;
    int  channelCount() const override { return count_; }
    void setFrequencyHz(uint16_t hz) override;
    void writeUs(int ch, uint16_t us) override;

private:
    const int* pins_;
    int        count_;
    uint16_t   freq_hz_ = 50;
};

}  // namespace wp
