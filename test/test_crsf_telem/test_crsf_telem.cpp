#include "unity.h"
#include "telemetry/crsf_telem.h"
#include <cstring>
using namespace wp;

void setUp() {} void tearDown() {}

// CRC8/DVB-S2 已知向量（poly 0xD5），Python 参考实现交叉验证
void test_crc8_known_vectors() {
    uint8_t a[] = {0x00};
    TEST_ASSERT_EQUAL_HEX8(0x00, crc8_dvbs2(a, 1));
    uint8_t b[] = {0x01, 0x02, 0x03};
    TEST_ASSERT_EQUAL_HEX8(0x3F, crc8_dvbs2(b, 3));
    uint8_t c[] = {0x21, 0x4F, 0x4B, 0x00};   // type 0x21 + "OK\0"
    TEST_ASSERT_EQUAL_HEX8(0x97, crc8_dvbs2(c, 4));
}

// buildFrame：[addr][len][type][payload][crc8]，len=plen+2，crc 覆盖 type..payload
void test_build_frame_layout() {
    uint8_t payload[] = {0x4F, 0x4B, 0x00};   // "OK\0"
    uint8_t out[kCrsfMaxFrame];
    int n = buildFrame(kCrsfSyncAddr, 0x21, payload, 3, out, sizeof(out));
    TEST_ASSERT_EQUAL_INT(7, n);              // addr+len+type+3payload+crc = 7
    TEST_ASSERT_EQUAL_HEX8(0xC8, out[0]);     // sync
    TEST_ASSERT_EQUAL_HEX8(5, out[1]);        // len = type+payload+crc = 1+3+1
    TEST_ASSERT_EQUAL_HEX8(0x21, out[2]);     // type
    TEST_ASSERT_EQUAL_HEX8(0x4F, out[3]);
    TEST_ASSERT_EQUAL_HEX8(0x00, out[5]);
    // crc 覆盖 out[2..6) = {0x21,0x4F,0x4B,0x00}
    TEST_ASSERT_EQUAL_HEX8(0x97, out[6]);
}

// 无 payload 帧：len=2（type+crc）
void test_build_frame_no_payload() {
    uint8_t out[kCrsfMaxFrame];
    int n = buildFrame(kCrsfSyncAddr, 0x21, nullptr, 0, out, sizeof(out));
    TEST_ASSERT_EQUAL_INT(4, n);              // addr+len+type+crc
    TEST_ASSERT_EQUAL_HEX8(2, out[1]);        // len
    TEST_ASSERT_EQUAL_HEX8(0x21, out[2]);
}

// 容量不足返回 0，不越界写
void test_build_frame_cap_guard() {
    uint8_t payload[] = {1,2,3};
    uint8_t out[4];                            // 需要 7，给 4
    int n = buildFrame(kCrsfSyncAddr, 0x21, payload, 3, out, sizeof(out));
    TEST_ASSERT_EQUAL_INT(0, n);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_crc8_known_vectors);
    RUN_TEST(test_build_frame_layout);
    RUN_TEST(test_build_frame_no_payload);
    RUN_TEST(test_build_frame_cap_guard);
    return UNITY_END();
}
