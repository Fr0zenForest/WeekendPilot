#include "config/nvs_config_backend.h"

namespace wp {

bool NvsConfigBackend::write(const uint8_t* buf, uint16_t len) {
    if (!prefs_.begin(kNs, /*readOnly=*/false)) return false;
    size_t n = prefs_.putBytes(kKey, buf, len);
    prefs_.end();
    return n == len;
}

uint16_t NvsConfigBackend::read(uint8_t* buf, uint16_t cap) {
    if (!prefs_.begin(kNs, /*readOnly=*/true)) return 0;
    size_t stored = prefs_.getBytesLength(kKey);
    if (stored == 0 || stored > cap) { prefs_.end(); return 0; }
    size_t n = prefs_.getBytes(kKey, buf, stored);
    prefs_.end();
    return (uint16_t)n;
}

}  // namespace wp
