#include "unity.h"
#include "sensors/ist8310_decode.h"
using namespace wp;

void setUp() {} void tearDown() {}

// ⚠️ 寄存器布局移植自 ArduPilot，非数据手册。仅验字节装配数学。
void test_decode_little_endian_signed() {
    // X=+1 (0x0001), Y=-1 (0xFFFF), Z=+256 (0x0100)
    // ⚠️ Z 经右手系翻转 -> 期望 -256（与 ArduPilot 一致）
    uint8_t raw[6] = {0x01, 0x00, 0xFF, 0xFF, 0x00, 0x01};
    MagSample m = ist8310Decode(raw);
    TEST_ASSERT_TRUE(m.valid);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 1.0f * ist8310::kMilliGaussPerLsb,    m.mag_x);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, -1.0f * ist8310::kMilliGaussPerLsb,   m.mag_y);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, -256.0f * ist8310::kMilliGaussPerLsb, m.mag_z);
}

void test_decode_zero_is_valid_zero() {
    uint8_t raw[6] = {0,0,0,0,0,0};
    MagSample m = ist8310Decode(raw);
    TEST_ASSERT_TRUE(m.valid);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, m.mag_x);
}

void test_whoami_constant() {
    TEST_ASSERT_EQUAL_UINT8(0x10, ist8310::kWhoAmIVal);
    TEST_ASSERT_EQUAL_UINT8(0x0E, ist8310::kI2cAddr);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_decode_little_endian_signed);
    RUN_TEST(test_decode_zero_is_valid_zero);
    RUN_TEST(test_whoami_constant);
    return UNITY_END();
}
