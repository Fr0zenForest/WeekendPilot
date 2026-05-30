#pragma once
#include "sensors/sensor_types.h"

namespace wp {

// NMEA0183 纯逻辑工具（零 I/O）。详见 docs §数据真实性声明。
// XOR 校验：'$' 与 '*' 之间所有字节异或，与 *HH 两位十六进制比对。
// sentence 含起始 '$'，len 为不含 \r\n 的有效长度（指到 '*HH' 末位）。
bool nmeaChecksumValid(const char* sentence, int len);

// DDMM.MMMMM（纬度）/ DDDMM.MMMM（经度）→ 十进制度。field 不可为空串。
double nmeaDdmToDeg(const char* field);

// 增量字节流解析器（零 I/O，零堆分配）。喂 UART 字节，整句校验通过后
// 更新内部 GnssSample。GGA→定位质量/卫星数/海拔；RMC→有效标志/速度/航向；
// VTG→地速。Down 速度 NMEA 给不出，恒置 0（见数据真实性声明）。
class NmeaParser {
public:
    // 喂一个字节。返回 true 表示本字节刚好结束了一条合法且解析过的语句。
    bool pushByte(char c);
    // 当前累积的最新样本（valid=有过有效 RMC/GGA 定位）。
    const GnssSample& sample() const { return sample_; }

private:
    static constexpr int kBufMax = 96;     // NMEA 句最长 82，留余量
    char buf_[kBufMax] = {0};
    int  len_ = 0;
    GnssSample sample_{};

    void processSentence(int len);
};

}  // namespace wp
