#include "unity.h"
#include "config/capabilities.h"
#include "config/board_config.h"
using namespace wp;

void setUp() {} void tearDown() {}

void test_pc_test_build_enables_all_capabilities() {
    // PC/测试构建默认全开（算法都要在 SITL 验）
    TEST_ASSERT_TRUE(kHasImu);
    TEST_ASSERT_TRUE(kHasBaro);
    TEST_ASSERT_TRUE(kHasMag);
}

void test_board_pins_defined() {
    // 板级 profile 暴露引脚常量（值见设计 §6.3）
    TEST_ASSERT_EQUAL_INT(18, kPinI2cSda);
    TEST_ASSERT_EQUAL_INT(39, kPinI2cScl);
    TEST_ASSERT_EQUAL_INT(0x68, kAddrImu);
    TEST_ASSERT_EQUAL_INT(0x76, kAddrBaro);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_pc_test_build_enables_all_capabilities);
    RUN_TEST(test_board_pins_defined);
    return UNITY_END();
}
