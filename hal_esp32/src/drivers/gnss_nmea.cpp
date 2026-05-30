#include "drivers/gnss_nmea.h"
#if WP_HAS_GPS

namespace wp {

bool GnssNmea::probe() {
    uart_.begin(baud_, SERIAL_8N1, rx_, tx_);
    // 限时 1.2s 等任意一条合法 NMEA 句（不要求已定位）。
    uint32_t t0 = millis();
    while (millis() - t0 < 1200) {
        while (uart_.available()) {
            if (parser_.pushByte((char)uart_.read())) { gotSentence_ = true; return true; }
        }
    }
    return false;   // 没插 GPS / 没接对 -> probe 失败，Frontend 不计入 has_gnss
}

bool GnssNmea::read(GnssSample& out) {
    bool any = false;
    while (uart_.available()) {
        if (parser_.pushByte((char)uart_.read())) any = true;
    }
    if (!any && !gotSentence_) return false;  // 无任何句过
    out = parser_.sample();                    // 含 valid 标志
    return out.valid;                          // ⚠️ 契约：无效定位返回 false 不污染
}

}  // namespace wp
#endif  // WP_HAS_GPS
