#pragma once
#include <cmath>

namespace wp {

// 一阶低通（PT1）。参考 Betaflight/iNav pt1FilterApply。
class FilterPt1 {
public:
    explicit FilterPt1(float cutoff_hz) : cutoff_hz_(cutoff_hz), state_(0.0f) {}

    void setCutoff(float cutoff_hz) { cutoff_hz_ = cutoff_hz; }
    void reset(float v = 0.0f) { state_ = v; }

    float apply(float input, float dt) {
        // RC = 1/(2*pi*fc); alpha = dt/(RC+dt)
        const float rc = 1.0f / (2.0f * 3.14159265358979f * cutoff_hz_);
        const float alpha = dt / (rc + dt);
        state_ += alpha * (input - state_);
        return state_;
    }
    float value() const { return state_; }

private:
    float cutoff_hz_;
    float state_;
};

}  // namespace wp
