#include "telemetry/crsf_telem_tx.h"
#if WP_HAS_TELEMETRY

namespace wp {

static constexpr uint32_t kAttPeriodMs = 100;   // 姿态 10Hz
static constexpr uint32_t kFmPeriodMs  = 500;   // 飞行模式 2Hz

void CrsfTelemetryTx::tick(uint32_t now_ms, const TelemSnapshot& snap) {
    uint8_t frame[kCrsfMaxFrame];
    // 半双工：每拍至多发一个帧。姿态优先（变化快），其次飞行模式。
    if ((int32_t)(now_ms - next_att_ms_) >= 0) {
        next_att_ms_ = now_ms + kAttPeriodMs;
        int n = encodeAttitude(snap.roll_rad, snap.pitch_rad, snap.yaw_rad,
                               frame, sizeof(frame));
        if (n > 0) uart_.write(frame, n);
        return;
    }
    if ((int32_t)(now_ms - next_fm_ms_) >= 0) {
        next_fm_ms_ = now_ms + kFmPeriodMs;
        int n = encodeFlightMode(snap.flight_mode, frame, sizeof(frame));
        if (n > 0) uart_.write(frame, n);
        return;
    }
}

}  // namespace wp
#endif  // WP_HAS_TELEMETRY
