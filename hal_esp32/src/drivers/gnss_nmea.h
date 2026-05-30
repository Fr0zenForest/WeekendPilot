#pragma once
#include "config/capabilities.h"
#if WP_HAS_GPS
#include "sensors/sensor_interfaces.h"
#include "sensors/nmea_parser.h"
#include <Arduino.h>

namespace wp {

// IGnss over UART：从 HardwareSerial 抽字节喂 NmeaParser，read() 返回最新样本。
class GnssNmea : public IGnss {
public:
    GnssNmea(HardwareSerial& uart, int rxPin, int txPin, int baud)
        : uart_(uart), rx_(rxPin), tx_(txPin), baud_(baud) {}
    bool probe() override;            // 启 UART，限时等首个合法句
    bool init()  override { return true; }  // probe 已配置
    bool read(GnssSample& out) override;     // 抽干 UART，无新有效样本则 false
private:
    HardwareSerial& uart_;
    int rx_, tx_, baud_;
    NmeaParser parser_;
    bool gotSentence_ = false;
};

}  // namespace wp
#endif  // WP_HAS_GPS
