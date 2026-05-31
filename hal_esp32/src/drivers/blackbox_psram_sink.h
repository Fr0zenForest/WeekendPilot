#pragma once
#include "config/capabilities.h"
#if WP_HAS_BLACKBOX
#include "blackbox/blackbox_sink.h"

namespace wp {

// WARNING 新代码，未在实物 PSRAM 上实测。在 ESP32-S3 的 PSRAM(SPIRAM) 上分配环形缓冲，
//    复用 core 的 RingBufferSink 逻辑（仅 buffer 来自 heap_caps_malloc SPIRAM）。
class PsramSink : public IBlackboxSink {
public:
    explicit PsramSink(size_t cap_bytes);
    ~PsramSink() override;
    bool ok() const { return ring_ != nullptr; }

    bool write(const uint8_t* data, size_t len) override;
    void flush() override;
    size_t capacityBytes() const override;
    size_t usedBytes() const override;

private:
    uint8_t* buf_ = nullptr;
    RingBufferSink* ring_ = nullptr;   // 复用 core 环形逻辑
};

}  // namespace wp
#endif  // WP_HAS_BLACKBOX
