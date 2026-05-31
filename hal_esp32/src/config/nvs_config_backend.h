#pragma once
#include "config/config_store.h"
#include <Preferences.h>

namespace wp {

// IConfigBackend over ESP32 NVS（Preferences）。namespace "wpcfg"，key "blob"。
class NvsConfigBackend : public IConfigBackend {
public:
    bool write(const uint8_t* buf, uint16_t len) override;
    uint16_t read(uint8_t* buf, uint16_t cap) override;
private:
    Preferences prefs_;
    static constexpr const char* kNs  = "wpcfg";
    static constexpr const char* kKey = "blob";
};

}  // namespace wp
