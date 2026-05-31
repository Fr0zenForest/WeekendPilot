#pragma once
#include <cstdint>
#include <cstddef>

namespace wp {

// 黑匣子落盘接口：core 只认这个，不认介质。PSRAM/W25Q/SD 在 hal 实现。
// write 返回 false 表示本次写入被拒（块超单次容量）；环形覆盖不算失败。
struct IBlackboxSink {
    virtual ~IBlackboxSink() = default;
    virtual bool write(const uint8_t* data, size_t len) = 0;
    virtual void flush() = 0;
    virtual size_t capacityBytes() const = 0;
    virtual size_t usedBytes() const = 0;
};

// WARNING 新代码。固定容量环形缓冲：写满后从头覆盖最旧数据（保留最近 N 字节）。
// 调用方提供底层 buffer（PC 测试用栈数组，板载用 PSRAM）。
class RingBufferSink : public IBlackboxSink {
public:
    RingBufferSink(uint8_t* buf, size_t cap) : buf_(buf), cap_(cap) {}

    bool write(const uint8_t* data, size_t len) override {
        if (len > cap_) return false;          // 单块超总容量，拒绝
        for (size_t i = 0; i < len; ++i) {
            buf_[head_] = data[i];
            head_ = (head_ + 1) % cap_;
            if (used_ < cap_) ++used_; else wrapped_ = true;
        }
        return true;
    }
    void flush() override {}                    // 内存缓冲无需 flush
    size_t capacityBytes() const override { return cap_; }
    size_t usedBytes() const override { return used_; }
    bool wrapped() const { return wrapped_; }   // 是否已发生覆盖

private:
    uint8_t* buf_;
    size_t cap_;
    size_t head_ = 0;
    size_t used_ = 0;
    bool wrapped_ = false;
};

}  // namespace wp
