#include "blackbox/blackbox.h"

namespace wp {

void Blackbox::begin(const BlackboxConfig& cfg, IBlackboxSink* sink) {
    cfg_ = cfg;
    sink_ = sink;
    counter_ = 0;
    frames_written_ = 0;
}

void Blackbox::logFrame(const BlackboxFrame& f) {
    if (!cfg_.enabled || sink_ == nullptr) return;
    const uint16_t dec = cfg_.decimation == 0 ? 1 : cfg_.decimation;
    if (++counter_ < dec) return;     // 未到降采样间隔
    counter_ = 0;

    uint8_t buf[kFrameBytes];
    encodeFrame(f, buf);
    if (sink_->write(buf, kFrameBytes)) ++frames_written_;
}

}  // namespace wp
