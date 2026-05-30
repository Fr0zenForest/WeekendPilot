#include "sensors/nmea_parser.h"
#include <cstdlib>
#include <cstring>

namespace wp {

static int hexNibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

bool nmeaChecksumValid(const char* s, int len) {
    // 形如 $....*HH，至少 "$*HH" -> len>=4，且倒数第三位是 '*'。
    if (len < 4 || s[0] != '$' || s[len - 3] != '*') return false;
    int hi = hexNibble(s[len - 2]);
    int lo = hexNibble(s[len - 1]);
    if (hi < 0 || lo < 0) return false;
    unsigned char want = static_cast<unsigned char>((hi << 4) | lo);
    unsigned char got = 0;
    for (int i = 1; i < len - 3; ++i)  // '$' 之后到 '*' 之前
        got ^= static_cast<unsigned char>(s[i]);
    return got == want;
}

double nmeaDdmToDeg(const char* field) {
    // DDMM.MMMM：小数点前两位是分，再前面是度。
    const char* dot = std::strchr(field, '.');
    int intDigits = dot ? static_cast<int>(dot - field)
                        : static_cast<int>(std::strlen(field));
    if (intDigits < 3) return 0.0;       // 至少 MM. 两位分 + 1 位度
    int degDigits = intDigits - 2;
    char degBuf[8] = {0};
    if (degDigits >= static_cast<int>(sizeof(degBuf))) return 0.0;
    std::memcpy(degBuf, field, static_cast<size_t>(degDigits));
    double degrees = std::atof(degBuf);
    double minutes = std::atof(field + degDigits);
    return degrees + minutes / 60.0;
}

}  // namespace wp
