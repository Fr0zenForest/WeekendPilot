#pragma once
#include <cstdint>
#include <cstddef>

namespace wp {

// CRSF 帧：[sync/addr][len][type][payload...][crc8]，整帧 <=64B。
// 移植自 TBS CRSF spec：crc8 = CRC8/DVB-S2(poly 0xD5)，覆盖 type..payload 末尾。
constexpr uint8_t  kCrsfSyncAddr = 0xC8;   // 飞控遥测下行 sync 字节
constexpr int      kCrsfMaxFrame = 64;     // 整帧字节上限

// CRC8/DVB-S2，多项式 0xD5。对 data[0..len) 计算。
uint8_t crc8_dvbs2(const uint8_t* data, size_t len);

// 组装一帧到 out：[addr][len][type][payload][crc8]。
//   payload/plen 可为 nullptr/0（无 payload 帧）。
//   crc8 覆盖 type..payload（即 out[2..2+plen]）。
//   返回写入的总字节数；out 容量不足（< plen+4）返回 0。
int buildFrame(uint8_t addr, uint8_t type, const uint8_t* payload, int plen,
               uint8_t* out, int cap);

// 飞控遥测快照（纯值，由板载每帧组装；core 不碰硬件）。仿 diag/status_line.h。
struct TelemSnapshot {
    float roll_rad = 0.0f, pitch_rad = 0.0f, yaw_rad = 0.0f;
    const char* flight_mode = "OFF ";   // 4 字符模式串（指向静态字面量）
    // 电池（留接口，本轮可不填/不发）：
    float battery_v = 0.0f, battery_a = 0.0f;
    uint32_t battery_mah = 0;
    uint8_t  battery_pct = 0;
};

// 0x1E 姿态：payload = 3×int16 大端，单位 rad×10000（roll/pitch/yaw）。
// 返回整帧字节数；cap 不足返回 0。
int encodeAttitude(float roll_rad, float pitch_rad, float yaw_rad,
                   uint8_t* out, int cap);

// 0x21 飞行模式：payload = NUL 结尾 ASCII。mode 为 C 字符串（自动加 NUL）。
int encodeFlightMode(const char* mode, uint8_t* out, int cap);

// 0x08 电池：电压(int16,0.1V) 电流(int16,0.1A) 容量(uint24 大端,mAh) 剩余(uint8,%)。
int encodeBattery(float volts, float amps, uint32_t mah, uint8_t pct,
                  uint8_t* out, int cap);

}  // namespace wp
