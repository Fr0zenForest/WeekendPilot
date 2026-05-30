#pragma once
#include "sensors/sensor_types.h"

namespace wp {

// NMEA0183 纯逻辑工具（零 I/O）。详见 docs §数据真实性声明。
// XOR 校验：'$' 与 '*' 之间所有字节异或，与 *HH 两位十六进制比对。
// sentence 含起始 '$'，len 为不含 \r\n 的有效长度（指到 '*HH' 末位）。
bool nmeaChecksumValid(const char* sentence, int len);

// DDMM.MMMMM（纬度）/ DDDMM.MMMM（经度）→ 十进制度。field 不可为空串。
double nmeaDdmToDeg(const char* field);

}  // namespace wp
