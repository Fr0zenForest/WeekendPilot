#include "unity.h"
#include "sensors/nmea_parser.h"
#include <cstring>
#include <cmath>
using namespace wp;

void setUp() {} void tearDown() {}

// ⚠️ 数据手册样例报文，非本台实物抓包（见 plan 数据真实性声明）。
void test_checksum_valid_on_datasheet_rmc() {
    // $GNVTG,,T,,M,0.001,N,0.002,K,D*3B —— 数据手册原样
    const char* s = "$GNVTG,,T,,M,0.001,N,0.002,K,D*3B";
    TEST_ASSERT_TRUE(nmeaChecksumValid(s, (int)strlen(s)));
}

void test_checksum_rejects_corrupted() {
    const char* s = "$GNVTG,,T,,M,0.001,N,0.009,K,D*3B"; // 篡改 payload
    TEST_ASSERT_FALSE(nmeaChecksumValid(s, (int)strlen(s)));
}

void test_checksum_rejects_too_short_or_no_star() {
    TEST_ASSERT_FALSE(nmeaChecksumValid("$X", 2));
    TEST_ASSERT_FALSE(nmeaChecksumValid("GPGGA,1,2,3", 11)); // 无 '$'
}

void test_ddm_to_deg_latitude() {
    // 2240.62039 -> 22 + 40.62039/60 = 22.677006...
    double d = nmeaDdmToDeg("2240.62039");
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 22.677006, d);
}

void test_ddm_to_deg_longitude() {
    // 11359.86703 -> 113 + 59.86703/60 = 113.997783...
    double d = nmeaDdmToDeg("11359.86703");
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 113.997783, d);
}

static void feed(NmeaParser& p, const char* s) {
    for (const char* c = s; *c; ++c) p.pushByte(*c);
    p.pushByte('\r'); p.pushByte('\n');
}

// ⚠️ 数据手册样例报文，非本台实物抓包。
void test_parse_gga_fills_position_and_sats() {
    NmeaParser p;
    feed(p, "$GNGGA,090446.00,2240.62039,N,11359.86703,E,2,12,0.49,98.8,M,-2.7,M,,*6E");
    const GnssSample& s = p.sample();
    TEST_ASSERT_TRUE(s.valid);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 22.677006, (float)s.lat);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 113.997783, (float)s.lon);
    TEST_ASSERT_EQUAL_UINT8(12, s.sats);
    TEST_ASSERT_EQUAL_UINT8(2, s.fix);   // GGA fix quality 字段
}

void test_parse_rmc_status_void_marks_invalid() {
    NmeaParser p;
    // 数据手册 RMC：状态位 'A'=有效。这里造一条 'V'=无效（仍合法格式）。
    feed(p, "$GPRMC,090446.00,V,2240.62039,N,11359.86703,E,0.001,,130923,,,N*63");
    TEST_ASSERT_FALSE(p.sample().valid);
}

void test_parse_vtg_fills_ground_speed_and_velned() {
    NmeaParser p;
    // VTG: 航向 90 deg(T), 地速 10.00 节 -> 5.1444 m/s, 向东
    feed(p, "$GPVTG,90.0,T,,M,10.00,N,18.52,K,A*3B");
    const GnssSample& s = p.sample();
    TEST_ASSERT_FLOAT_WITHIN(0.05, 5.1444f, s.ground_speed_mps);
    TEST_ASSERT_FLOAT_WITHIN(0.05, 0.0f, s.vel_ned[0]);  // 正东 -> 北分量 0
    TEST_ASSERT_FLOAT_WITHIN(0.05, 5.1444f, s.vel_ned[1]); // 东分量
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, s.vel_ned[2]);  // Down 恒 0
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_checksum_valid_on_datasheet_rmc);
    RUN_TEST(test_checksum_rejects_corrupted);
    RUN_TEST(test_checksum_rejects_too_short_or_no_star);
    RUN_TEST(test_ddm_to_deg_latitude);
    RUN_TEST(test_ddm_to_deg_longitude);
    RUN_TEST(test_parse_gga_fills_position_and_sats);
    RUN_TEST(test_parse_rmc_status_void_marks_invalid);
    RUN_TEST(test_parse_vtg_fills_ground_speed_and_velned);
    return UNITY_END();
}
