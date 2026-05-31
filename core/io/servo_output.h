#pragma once
#include <cstdint>

namespace wp {

// 舵机/PWM 输出后端抽象。把"逻辑通道 us 值 -> 物理 PWM"与硬件解耦。
// 实现方：LedcOutput（原生 8 路）、Pca9685Output（I2C 16 路）、（将来）RP2040 协处理器。
class IServoOutput {
public:
    virtual ~IServoOutput() = default;
    // 初始化并探测硬件。返回 false = 不可用（如 I2C 探测失败）。
    virtual bool begin() = 0;
    // 本后端提供的通道数（标称值，与 probe 结果无关，用于全局索引分配）。
    virtual int  channelCount() const = 0;
    // 设置 PWM 刷新频率（Hz）。后端不支持调频则忽略或夹取到能力范围。
    virtual void setFrequencyHz(uint16_t hz) = 0;
    // 写第 ch 路（后端本地索引 [0,channelCount)）的脉宽（µs）。实现内部夹 [1000,2000]。
    virtual void writeUs(int ch, uint16_t us) = 0;
};

}  // namespace wp
