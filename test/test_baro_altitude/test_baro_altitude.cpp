#include "unity.h"
#include "sensors/baro_altitude.h"
using namespace wp;

void setUp() {} void tearDown() {}

// ⚠️ 新代码，ISA 公式数学可测；实物须起飞前置零基准。
void test_at_reference_is_zero() {
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 0.0f, baroPressureToAltitude(101325.0f, 101325.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 0.0f, baroPressureToAltitude(95000.0f, 95000.0f));
}

void test_lower_pressure_is_higher_altitude() {
    float h = baroPressureToAltitude(101325.0f - 100.0f, 101325.0f); // -1 hPa
    TEST_ASSERT_TRUE(h > 7.0f && h < 10.0f);   // ~8.4 m
}

void test_known_altitude_ratio() {
    // 海平面标准下 p=89876 Pa 对应约 1000 m（ISA 表值）。
    float h = baroPressureToAltitude(89876.0f, 101325.0f);
    TEST_ASSERT_FLOAT_WITHIN(15.0f, 1000.0f, h);
}

void test_reference_class_relative() {
    BaroAltitude ba;
    ba.setReference(98000.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 0.0f, ba.altitude(98000.0f));
    TEST_ASSERT_TRUE(ba.altitude(97000.0f) > 0.0f);
}

void test_invalid_pressure_zero() {
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, baroPressureToAltitude(0.0f, 101325.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, baroPressureToAltitude(95000.0f, 0.0f));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_at_reference_is_zero);
    RUN_TEST(test_lower_pressure_is_higher_altitude);
    RUN_TEST(test_known_altitude_ratio);
    RUN_TEST(test_reference_class_relative);
    RUN_TEST(test_invalid_pressure_zero);
    return UNITY_END();
}
