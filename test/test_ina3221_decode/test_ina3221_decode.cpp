#include "unity.h"
#include "sensors/ina3221_decode.h"
using namespace wp;

void setUp() {} void tearDown() {}

// LSB = 40µV，值在 bit15..3。raw=0x07D0(=2000) -> v=2000>>3=250 -> 250*40µV=0.01V
void test_shunt_volt_positive() {
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.01f, ina3221ShuntVolt(0x07D0));
}

// 负值：13-bit 有符号需正确符号扩展。-0.01V -> v=-250 -> (-250<<3)=0xF830
void test_shunt_volt_negative() {
    TEST_ASSERT_FLOAT_WITHIN(1e-6, -0.01f, ina3221ShuntVolt(0xF830));
}

// 零
void test_shunt_volt_zero() {
    TEST_ASSERT_FLOAT_WITHIN(1e-9, 0.0f, ina3221ShuntVolt(0x0000));
}

// bit2..0 应被忽略（恒 0，但即便有杂位也不影响）
void test_low_bits_ignored() {
    // 0x07D0 | 0x0007 = 0x07D7；>>3 仍 = 250
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.01f, ina3221ShuntVolt(0x07D7));
}

// 电流：I = Vshunt / Rshunt。0.01V / 0.01Ω = 1.0A
void test_current_positive() {
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 1.0f, ina3221Current(0x07D0, 0.01f));
}

// 负电流
void test_current_negative() {
    TEST_ASSERT_FLOAT_WITHIN(1e-4, -1.0f, ina3221Current(0xF830, 0.01f));
}

// shunt=0 防护：返回 0 而非 inf/nan
void test_current_zero_shunt_guarded() {
    float i = ina3221Current(0x07D0, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-9, 0.0f, i);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_shunt_volt_positive);
    RUN_TEST(test_shunt_volt_negative);
    RUN_TEST(test_shunt_volt_zero);
    RUN_TEST(test_low_bits_ignored);
    RUN_TEST(test_current_positive);
    RUN_TEST(test_current_negative);
    RUN_TEST(test_current_zero_shunt_guarded);
    return UNITY_END();
}
