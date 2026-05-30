#include "unity.h"
#include "sensors/sensor_types.h"
using namespace wp;

void setUp() {} void tearDown() {}

void test_tier_enum_ordered() {
    // 档位有序：None < Base < Plus < Pro < Max，便于"≥某档"比较
    TEST_ASSERT_TRUE(static_cast<int>(SensorTier::None) < static_cast<int>(SensorTier::Base));
    TEST_ASSERT_TRUE(static_cast<int>(SensorTier::Base) < static_cast<int>(SensorTier::Plus));
    TEST_ASSERT_TRUE(static_cast<int>(SensorTier::Plus) < static_cast<int>(SensorTier::Pro));
    TEST_ASSERT_TRUE(static_cast<int>(SensorTier::Pro) < static_cast<int>(SensorTier::Max));
}

void test_gnss_sample_defaults_invalid() {
    GnssSample g{};
    TEST_ASSERT_FALSE(g.valid);
}

void test_airspeed_sample_defaults_invalid() {
    AirspeedSample a{};
    TEST_ASSERT_FALSE(a.valid);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_tier_enum_ordered);
    RUN_TEST(test_gnss_sample_defaults_invalid);
    RUN_TEST(test_airspeed_sample_defaults_invalid);
    return UNITY_END();
}
