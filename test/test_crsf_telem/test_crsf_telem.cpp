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

// 0x1E 姿态：CRSF 标准 payload 顺序 = pitch,roll,yaw。
// 调用 encodeAttitude(roll=0.5, pitch=-0.25, yaw=1.0)：
//   slot0-1 = pitch -0.25 -> -2500 = 0xF63C
//   slot2-3 = roll   0.5  ->  5000 = 0x1388
//   slot4-5 = yaw    1.0  -> 10000 = 0x2710
void test_encode_attitude_vector() {
    uint8_t out[kCrsfMaxFrame];
    int n = encodeAttitude(0.5f, -0.25f, 1.0f, out, sizeof(out));
    TEST_ASSERT_EQUAL_INT(10, n);             // C8 08 1E +6payload +crc
    TEST_ASSERT_EQUAL_HEX8(0xC8, out[0]);
    TEST_ASSERT_EQUAL_HEX8(0x08, out[1]);     // len=type+6+crc=8
    TEST_ASSERT_EQUAL_HEX8(0x1E, out[2]);
    TEST_ASSERT_EQUAL_HEX8(0xF6, out[3]);     // pitch -2500 = 0xF63C
    TEST_ASSERT_EQUAL_HEX8(0x3C, out[4]);
    TEST_ASSERT_EQUAL_HEX8(0x13, out[5]);     // roll 5000 = 0x1388
    TEST_ASSERT_EQUAL_HEX8(0x88, out[6]);
    TEST_ASSERT_EQUAL_HEX8(0x27, out[7]);     // yaw 10000 = 0x2710
    TEST_ASSERT_EQUAL_HEX8(0x10, out[8]);
    TEST_ASSERT_EQUAL_HEX8(0x74, out[9]);     // crc
}

// 负角符号：roll=-0.25 -> -2500 = 0xF63C，落在 roll 槽(slot2-3 = out[5]/out[6])
void test_encode_attitude_negative_sign() {
    uint8_t out[kCrsfMaxFrame];
    encodeAttitude(-0.25f, 0.0f, 0.0f, out, sizeof(out));
    TEST_ASSERT_EQUAL_HEX8(0x00, out[3]);     // pitch 0 (slot0-1)
    TEST_ASSERT_EQUAL_HEX8(0x00, out[4]);
    TEST_ASSERT_EQUAL_HEX8(0xF6, out[5]);     // roll -2500 (slot2-3)
    TEST_ASSERT_EQUAL_HEX8(0x3C, out[6]);
}

// 0x21 飞行模式 "ANGL" -> payload "ANGL\0"
void test_encode_flight_mode() {
    uint8_t out[kCrsfMaxFrame];
    int n = encodeFlightMode("ANGL", out, sizeof(out));
    TEST_ASSERT_EQUAL_INT(9, n);              // C8 07 21 +5("ANGL\0") +crc
    TEST_ASSERT_EQUAL_HEX8(0x07, out[1]);     // len=type+5+crc=7
    TEST_ASSERT_EQUAL_HEX8(0x21, out[2]);
    TEST_ASSERT_EQUAL_HEX8(0x41, out[3]);     // 'A'
    TEST_ASSERT_EQUAL_HEX8(0x4C, out[6]);     // 'L'
    TEST_ASSERT_EQUAL_HEX8(0x00, out[7]);     // NUL
    TEST_ASSERT_EQUAL_HEX8(0xA7, out[8]);     // crc
}

// 容量守卫：太小返回 0
void test_encode_cap_guard() {
    uint8_t out[5];
    TEST_ASSERT_EQUAL_INT(0, encodeAttitude(0,0,0, out, sizeof(out)));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_crc8_known_vectors);
    RUN_TEST(test_build_frame_layout);
    RUN_TEST(test_build_frame_no_payload);
    RUN_TEST(test_build_frame_cap_guard);
    RUN_TEST(test_encode_attitude_vector);
    RUN_TEST(test_encode_attitude_negative_sign);
    RUN_TEST(test_encode_flight_mode);
    RUN_TEST(test_encode_cap_guard);
    return UNITY_END();
}
