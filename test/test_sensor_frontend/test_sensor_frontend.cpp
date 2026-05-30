#include "unity.h"
#include "sensors/sensor_types.h"
#include "sensors/sensor_interfaces.h"
#include "sensors/mock_sensors.h"
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

void test_mock_gyroaccel_probe_and_read() {
    MockGyroAccel imu;
    imu.present = true;
    imu.sample.gyro_x = 1.5f; imu.sample.accel_z = 1.0f; imu.sample.valid = true;
    TEST_ASSERT_TRUE(imu.probe());
    TEST_ASSERT_TRUE(imu.init());
    ImuSample s{};
    TEST_ASSERT_TRUE(imu.read(s));
    TEST_ASSERT_EQUAL_FLOAT(1.5f, s.gyro_x);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, s.accel_z);
}

void test_mock_absent_probe_fails() {
    MockBaro baro;
    baro.present = false;
    TEST_ASSERT_FALSE(baro.probe());
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_tier_enum_ordered);
    RUN_TEST(test_gnss_sample_defaults_invalid);
    RUN_TEST(test_airspeed_sample_defaults_invalid);
    RUN_TEST(test_mock_gyroaccel_probe_and_read);
    RUN_TEST(test_mock_absent_probe_fails);
    return UNITY_END();
}
