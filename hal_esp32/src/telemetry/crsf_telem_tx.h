#pragma once
#include "config/capabilities.h"
#if WP_HAS_TELEMETRY
#include <Arduino.h>
#include "telemetry/crsf_telem.h"

namespace wp {

// CRSF 遥测发送调度器。半双工：每次 tick 至多发一个到点的帧，避免突发占满下行。
// 频率初值：姿态 10Hz、模式 2Hz（电池本轮不发，留接口）。
class CrsfTelemetryTx {
public:
    explicit CrsfTelemetryTx(HardwareSerial& uart) : uart_(uart) {}
    // now_ms：millis()。snap：本帧数据。到点则编码并写 uart（每次至多一帧）。
    void tick(uint32_t now_ms, const TelemSnapshot& snap);

private:
    HardwareSerial& uart_;
    uint32_t next_att_ms_ = 0;   // 姿态下次发送时刻
    uint32_t next_fm_ms_  = 0;   // 飞行模式下次发送时刻
};

}  // namespace wp
#endif  // WP_HAS_TELEMETRY
