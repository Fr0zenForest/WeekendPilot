#include "unity.h"
#include "sensors/icm42688_decode.h"
using namespace wp;

void setUp() {} void tearDown() {}

// ⚠️ scales 移植自 ElrsRX，仅验字节装配+标度数学，非实物。
void test_decode_accel_gyro_scaling() {
    // 布局: [0..1]temp [2..3]ax [4..5]ay [6..7]az [8..9]gx [10..11]gy [12..13]gz, hi 在前
    uint8_t raw[14] = {0};
    raw[2] = 0x10; raw[3] = 0x00;   // ax = +4096 = 0x1000 -> +1.0 g
    raw[6] = 0x10; raw[7] = 0x00;   // az = +4096 -> +1.0 g
    raw[8] = 0x40; raw[9] = 0x10;   // gx = +16400 = 0x4010 -> 1000 dps
    ImuSample s = icm42688Decode(raw);
    TEST_ASSERT_TRUE(s.valid);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 1.0f, s.accel_x);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 1.0f, s.accel_z);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 1000.0f, s.gyro_x);
}

void test_decode_negative_signed() {
    uint8_t raw[14] = {0};
    raw[4] = 0xF0; raw[5] = 0x00;   // ay = -4096 = 0xF000 -> -1.0 g
    ImuSample s = icm42688Decode(raw);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, -1.0f, s.accel_y);
}

void test_whoami_constants() {
    TEST_ASSERT_EQUAL_UINT8(0x47, icm42688::kWhoAmIVal);
    TEST_ASSERT_EQUAL_UINT8(0x68, icm42688::kI2cAddr);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_decode_accel_gyro_scaling);
    RUN_TEST(test_decode_negative_signed);
    RUN_TEST(test_whoami_constants);
    return UNITY_END();
}
