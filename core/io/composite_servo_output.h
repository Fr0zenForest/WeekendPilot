#pragma once
#include <cstdint>
#include "io/servo_output.h"

namespace wp {

constexpr int kMaxServoBackends = 3;  // LEDC + PCA9685 + 余量（将来 RP2040）

// 把多个后端按注册顺序拼成连续全局索引空间：
//   后端0 占 [0, c0)，后端1 占 [c0, c0+c1)，...（c_i = 后端 channelCount()）
// 区间宽度用标称 channelCount()，与 begin() 是否成功无关 —— 缺芯片时写入静默丢弃，
// 但索引不错位（PeripheralMap/配置引用的全局口稳定）。
class CompositeServoOutput {
public:
    // 注册一个后端（不取所有权）。返回 false = 已满。注册顺序决定索引区间。
    bool addBackend(IServoOutput* backend);
    // 对所有已注册后端调用 begin()。返回"至少一个成功"。各后端可用性单独记录。
    bool begin();
    // 全局通道总数（所有后端 channelCount 之和）。
    int  channelCount() const;
    // 对所有后端设频率。
    void setFrequencyHz(uint16_t hz);
    // 写全局索引 ch 的脉宽。越界或目标后端 begin 失败 -> 静默丢弃。
    void writeUs(int ch, uint16_t us);
    // 某全局索引是否可写（在范围内 且 所属后端 begin 成功）。
    bool channelReady(int ch) const;

private:
    IServoOutput* backends_[kMaxServoBackends] = {};
    bool          ready_[kMaxServoBackends] = {};
    int           count_ = 0;
};

}  // namespace wp
