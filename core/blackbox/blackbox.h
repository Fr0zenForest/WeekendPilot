#pragma once
#include <cstdint>
#include "blackbox/blackbox_frame.h"
#include "blackbox/blackbox_sink.h"

namespace wp {

struct BlackboxConfig {
    bool enabled = false;       // 总开关（默认关，全写好先不使能）
    uint16_t decimation = 4;    // 每 N 次 logFrame 落 1 帧（控制环 1kHz / 4 = 250Hz）
};

// WARNING 新代码。黑匣子记录器：按 decimation 降采样，把 BlackboxFrame 编码进 sink。
// sink 由调用方持有（PC 测试 RingBufferSink，板载 PsramSink）。
class Blackbox {
public:
    void begin(const BlackboxConfig& cfg, IBlackboxSink* sink);
    void logFrame(const BlackboxFrame& f);     // 每控制周期调一次
    uint32_t framesWritten() const { return frames_written_; }

private:
    BlackboxConfig cfg_;
    IBlackboxSink* sink_ = nullptr;
    uint16_t counter_ = 0;
    uint32_t frames_written_ = 0;
};

}  // namespace wp
