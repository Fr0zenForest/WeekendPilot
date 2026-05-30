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

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_checksum_valid_on_datasheet_rmc);
    RUN_TEST(test_checksum_rejects_corrupted);
    RUN_TEST(test_checksum_rejects_too_short_or_no_star);
    RUN_TEST(test_ddm_to_deg_latitude);
    RUN_TEST(test_ddm_to_deg_longitude);
    return UNITY_END();
}
