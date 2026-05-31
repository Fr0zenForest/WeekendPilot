#include "drivers/blackbox_psram_sink.h"
#if WP_HAS_BLACKBOX
#include <esp_heap_caps.h>
#include <new>

namespace wp {

PsramSink::PsramSink(size_t cap_bytes) {
    // 优先 SPIRAM；失败则保持 nullptr（ok()=false，调用方降级不记录）。
    buf_ = static_cast<uint8_t*>(heap_caps_malloc(cap_bytes, MALLOC_CAP_SPIRAM));
    if (buf_) {
        ring_ = new (std::nothrow) RingBufferSink(buf_, cap_bytes);
        if (!ring_) { heap_caps_free(buf_); buf_ = nullptr; }
    }
}

PsramSink::~PsramSink() {
    delete ring_;
    if (buf_) heap_caps_free(buf_);
}

bool PsramSink::write(const uint8_t* data, size_t len) {
    return ring_ ? ring_->write(data, len) : false;
}
void PsramSink::flush() { if (ring_) ring_->flush(); }
size_t PsramSink::capacityBytes() const { return ring_ ? ring_->capacityBytes() : 0; }
size_t PsramSink::usedBytes() const { return ring_ ? ring_->usedBytes() : 0; }

}  // namespace wp
#endif  // WP_HAS_BLACKBOX
