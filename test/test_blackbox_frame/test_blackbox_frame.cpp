#include "unity.h"
#include "blackbox/blackbox_frame.h"
#include <cstring>
using namespace wp;

void setUp() {} void tearDown() {}

// WARNING 新代码（自定义定长二进制格式，非 Betaflight BBL）。round-trip 逻辑可测；
//    PSRAM 落盘与 Blackbox Explorer 兼容性未实测。

// 编码再解码应还原所有字段（round-trip）。
void test_frame_roundtrip() {
    BlackboxFrame f{};
    f.t_ms = 0x12345678u;
    f.gyro_x = 12.5f; f.gyro_y = -33.0f; f.gyro_z = 7.25f;
    f.accel_x = 0.1f; f.accel_y = -0.2f; f.accel_z = 1.0f;
    f.roll_deg = 15.0f; f.pitch_deg = -6.0f; f.yaw_deg = 180.0f;
    for (int i = 0; i < kNumServos; ++i) f.servo[i] = static_cast<uint16_t>(1000 + i * 100);
    f.baro_alt_m = 123.5f;
    f.mode = 1; f.flags = 0x05;

    uint8_t buf[kFrameBytes];
    encodeFrame(f, buf);
    BlackboxFrame g{};
    decodeFrame(buf, g);

    TEST_ASSERT_EQUAL_UINT32(f.t_ms, g.t_ms);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, f.gyro_x, g.gyro_x);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, f.gyro_y, g.gyro_y);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, f.gyro_z, g.gyro_z);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, f.accel_x, g.accel_x);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, f.accel_y, g.accel_y);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, f.accel_z, g.accel_z);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, f.roll_deg, g.roll_deg);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, f.pitch_deg, g.pitch_deg);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, f.yaw_deg, g.yaw_deg);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, f.baro_alt_m, g.baro_alt_m);
    for (int i = 0; i < kNumServos; ++i)
        TEST_ASSERT_EQUAL_UINT16(f.servo[i], g.servo[i]);
    TEST_ASSERT_EQUAL_UINT8(f.mode, g.mode);
    TEST_ASSERT_EQUAL_UINT8(f.flags, g.flags);
}

// 帧长度固定且等于声明值。
void test_frame_size_fixed() {
    // t_ms(4) + 9 floats(36) + 8 servos(16) + baro(4) + mode(1) + flags(1) = 62
    TEST_ASSERT_EQUAL_INT(62, kFrameBytes);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_frame_roundtrip);
    RUN_TEST(test_frame_size_fixed);
    return UNITY_END();
}
