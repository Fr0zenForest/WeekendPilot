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

}  // namespace wp
