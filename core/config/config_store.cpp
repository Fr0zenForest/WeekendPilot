#include "config/config_store.h"
#include "controller.h"   // 完整 ControllerConfig 定义
#include <cstring>
#include <type_traits>

namespace wp {

// size 字段是 buf[3..4]（uint16 小端）。一旦 ControllerConfig 超过 65535 字节，该字段
// 截断会导致 serialize/deserialize 静默错配 —— 届时需把 size 扩成 uint32 并升 version。
static_assert(sizeof(ControllerConfig) <= 65535,
    "ControllerConfig exceeds uint16 size field; widen size field + bump kConfigVersion");
static_assert(std::is_trivially_copyable<ControllerConfig>::value,
    "ControllerConfig must stay POD for memcpy serialization; "
    "adding std::string/std::vector etc. breaks the blob format");

uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; ++b)
            crc = (crc & 0x8000u)
                ? static_cast<uint16_t>((crc << 1) ^ 0x1021u)
                : static_cast<uint16_t>(crc << 1);
    }
    return crc;
}

uint16_t serializeConfig(const ControllerConfig& cfg, uint8_t* buf, size_t cap) {
    const uint16_t payload = static_cast<uint16_t>(sizeof(ControllerConfig));
    const uint16_t total = kConfigHeaderBytes + payload + kConfigCrcBytes;
    if (cap < total) return 0;
    buf[0] = static_cast<uint8_t>(kConfigMagic & 0xFF);
    buf[1] = static_cast<uint8_t>(kConfigMagic >> 8);
    buf[2] = kConfigVersion;
    buf[3] = static_cast<uint8_t>(payload & 0xFF);   // size 低字节
    buf[4] = static_cast<uint8_t>(payload >> 8);     // size 高字节
    std::memcpy(buf + kConfigHeaderBytes, &cfg, payload);
    uint16_t crc = crc16_ccitt(buf, kConfigHeaderBytes + payload);
    buf[kConfigHeaderBytes + payload]     = static_cast<uint8_t>(crc & 0xFF);
    buf[kConfigHeaderBytes + payload + 1] = static_cast<uint8_t>(crc >> 8);
    return total;
}

bool deserializeConfig(const uint8_t* buf, uint16_t len, ControllerConfig& out) {
    const uint16_t payload = static_cast<uint16_t>(sizeof(ControllerConfig));
    const uint16_t total = kConfigHeaderBytes + payload + kConfigCrcBytes;
    if (len < total) return false;
    uint16_t magic = static_cast<uint16_t>(buf[0]) | (static_cast<uint16_t>(buf[1]) << 8);
    if (magic != kConfigMagic) return false;
    if (buf[2] != kConfigVersion) return false;
    uint16_t stored_size = static_cast<uint16_t>(buf[3]) |
                           (static_cast<uint16_t>(buf[4]) << 8);
    if (stored_size != payload) return false;
    uint16_t want = crc16_ccitt(buf, kConfigHeaderBytes + payload);
    uint16_t got = static_cast<uint16_t>(buf[kConfigHeaderBytes + payload]) |
                   (static_cast<uint16_t>(buf[kConfigHeaderBytes + payload + 1]) << 8);
    if (want != got) return false;
    std::memcpy(&out, buf + kConfigHeaderBytes, payload);
    return true;
}

bool MemoryConfigBackend::write(const uint8_t* buf, uint16_t len) {
    if (len > kConfigBlobSize) return false;
    std::memcpy(store_, buf, len);
    len_ = len;
    return true;
}

uint16_t MemoryConfigBackend::read(uint8_t* buf, uint16_t cap) {
    if (len_ == 0 || cap < len_) return 0;
    std::memcpy(buf, store_, len_);
    return len_;
}

bool ConfigStore::save(const ControllerConfig& cfg) {
    if (!backend_) return false;
    uint8_t buf[kConfigBlobSize];
    uint16_t n = serializeConfig(cfg, buf, sizeof(buf));
    if (n == 0) return false;
    return backend_->write(buf, n);
}

bool ConfigStore::load(ControllerConfig& out) {
    if (!backend_) return false;
    uint8_t buf[kConfigBlobSize];
    uint16_t n = backend_->read(buf, sizeof(buf));
    if (n == 0) return false;
    return deserializeConfig(buf, n, out);
}

}  // namespace wp
