#include "io/composite_servo_output.h"

namespace wp {

bool CompositeServoOutput::addBackend(IServoOutput* backend) {
    if (count_ >= kMaxServoBackends || backend == nullptr) return false;
    backends_[count_++] = backend;
    return true;
}

bool CompositeServoOutput::begin() {
    bool any = false;
    for (int i = 0; i < count_; ++i) {
        ready_[i] = backends_[i]->begin();
        any = any || ready_[i];
    }
    return any;
}

int CompositeServoOutput::channelCount() const {
    int sum = 0;
    for (int i = 0; i < count_; ++i) sum += backends_[i]->channelCount();
    return sum;
}

void CompositeServoOutput::setFrequencyHz(uint16_t hz) {
    for (int i = 0; i < count_; ++i) backends_[i]->setFrequencyHz(hz);
}

// 把全局索引解析为 (后端下标, 本地索引)。返回后端下标，local 写出本地索引；越界返回 -1。
static int locate(IServoOutput* const* b, int count, int ch, int& local) {
    if (ch < 0) return -1;
    int base = 0;
    for (int i = 0; i < count; ++i) {
        int n = b[i]->channelCount();
        if (ch < base + n) { local = ch - base; return i; }
        base += n;
    }
    return -1;
}

void CompositeServoOutput::writeUs(int ch, uint16_t us) {
    int local = 0;
    int idx = locate(backends_, count_, ch, local);
    if (idx < 0 || !ready_[idx]) return;   // 越界或后端不可用 -> 静默丢弃
    backends_[idx]->writeUs(local, us);
}

bool CompositeServoOutput::channelReady(int ch) const {
    int local = 0;
    int idx = locate(backends_, count_, ch, local);
    return idx >= 0 && ready_[idx];
}

}  // namespace wp
