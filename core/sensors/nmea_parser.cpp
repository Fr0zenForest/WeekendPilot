#include "sensors/nmea_parser.h"
#include <cmath>
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

// --- NmeaParser ---
// 极简字段切分：把句内 ','/'*' 改 '\0'，记录每字段起点。零堆分配。
namespace {
struct Fields {
    static constexpr int kMax = 24;
    const char* f[kMax] = {nullptr};
    int n = 0;
};
void splitFields(char* s, int len, Fields& out) {
    out.f[out.n++] = s;
    for (int i = 0; i < len && out.n < Fields::kMax; ++i) {
        if (s[i] == ',' || s[i] == '*') { s[i] = '\0'; out.f[out.n++] = &s[i + 1]; }
    }
}
bool blank(const char* f) { return !f || f[0] == '\0'; }
}  // namespace

bool NmeaParser::pushByte(char c) {
    if (c == '\n') {
        int n = len_; len_ = 0;
        if (nmeaChecksumValid(buf_, n)) { processSentence(n); return true; }
        return false;
    }
    if (c == '\r') return false;
    if (c == '$') len_ = 0;                  // 句首重同步
    if (len_ < kBufMax - 1) buf_[len_++] = c;
    return false;
}

void NmeaParser::processSentence(int len) {
    // 句型在第 3~5 位（$GNxxx / $GPxxx）。
    // 捕获类型字符到局部变量，在 splitFields 修改 buf_ 之前读取。
    char t0 = buf_[3], t1 = buf_[4], t2 = buf_[5];
    Fields fl; splitFields(buf_, len, fl);
    if (t0=='G' && t1=='G' && t2=='A') {
        // GGA: 2=lat 3=N/S 4=lon 5=E/W 6=fix 7=sats 9=altMSL
        if (fl.n > 9) {
            if (!blank(fl.f[2])) { sample_.lat = nmeaDdmToDeg(fl.f[2]);
                if (fl.f[3] && fl.f[3][0]=='S') sample_.lat = -sample_.lat; }
            if (!blank(fl.f[4])) { sample_.lon = nmeaDdmToDeg(fl.f[4]);
                if (fl.f[5] && fl.f[5][0]=='W') sample_.lon = -sample_.lon; }
            sample_.fix  = blank(fl.f[6]) ? 0 : (uint8_t)atoi(fl.f[6]);
            sample_.sats = blank(fl.f[7]) ? 0 : (uint8_t)atoi(fl.f[7]);
            if (sample_.fix > 0) sample_.valid = true;
        }
    } else if (t0=='R' && t1=='M' && t2=='C') {
        // RMC: 2=status(A/V)
        if (fl.n > 2) sample_.valid = (fl.f[2] && fl.f[2][0]=='A');
    } else if (t0=='V' && t1=='T' && t2=='G') {
        // VTG: 1=courseTrue(deg) 5=speedKnots
        if (fl.n > 5) {
            double course = blank(fl.f[1]) ? 0.0 : atof(fl.f[1]);
            double kn     = blank(fl.f[5]) ? 0.0 : atof(fl.f[5]);
            float mps = (float)(kn * 0.514444);
            sample_.ground_speed_mps = mps;
            double r = course * 3.14159265358979 / 180.0;
            sample_.vel_ned[0] = (float)(mps * cos(r));  // 北
            sample_.vel_ned[1] = (float)(mps * sin(r));  // 东
            sample_.vel_ned[2] = 0.0f;                    // ⚠️ NMEA 无垂速，恒 0
        }
    }
}

}  // namespace wp
