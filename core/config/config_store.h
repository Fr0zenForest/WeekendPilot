#pragma once
#include <cstdint>
#include <cstddef>

namespace wp {

struct ControllerConfig;  // 前置声明，避免头循环依赖

// blob 布局：[magic(2) | version(1) | size(2,小端) | payload(memcpy ControllerConfig) | crc16(2)]
constexpr uint16_t kConfigMagic   = 0x5750;  // 字母 W,P
constexpr uint8_t  kConfigVersion = 3;   // was 2: size 字段 uint8->uint16
constexpr int      kConfigHeaderBytes = 5;   // magic2 + version1 + size2
constexpr int      kConfigCrcBytes    = 2;

// blob 上限：头 + payload(sizeof ControllerConfig) + crc。给足余量。
constexpr int kConfigBlobSize = 512;

uint16_t crc16_ccitt(const uint8_t* data, size_t len);

// 序列化/反序列化纯函数。serialize 返回写入字节数（0=缓冲不足）；
// deserialize 校验 magic/version/size/crc，全过才写 out 并返回 true。
uint16_t serializeConfig(const ControllerConfig& cfg, uint8_t* buf, size_t cap);
bool     deserializeConfig(const uint8_t* buf, uint16_t len, ControllerConfig& out);

// 平台持久化后端（NVS/文件）抽象。core 不碰具体 I/O。
struct IConfigBackend {
    virtual ~IConfigBackend() = default;
    virtual bool write(const uint8_t* buf, uint16_t len) = 0;
    virtual uint16_t read(uint8_t* buf, uint16_t cap) = 0;  // 返回读到字节数，0=空
};

// PC/测试用内存后端
class MemoryConfigBackend : public IConfigBackend {
public:
    bool write(const uint8_t* buf, uint16_t len) override;
    uint16_t read(uint8_t* buf, uint16_t cap) override;
private:
    uint8_t store_[kConfigBlobSize];
    uint16_t len_ = 0;
};

// 串起序列化 + 后端
class ConfigStore {
public:
    explicit ConfigStore(IConfigBackend* backend) : backend_(backend) {}
    bool save(const ControllerConfig& cfg);
    bool load(ControllerConfig& out);
private:
    IConfigBackend* backend_;
};

}  // namespace wp
