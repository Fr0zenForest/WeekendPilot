#include "unity.h"
#include "sensors/sensor_types.h"
#include "sensors/sensor_interfaces.h"
#include "sensors/mock_sensors.h"
#include "sensors/sensor_frontend.h"
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

void test_interface_polymorphic_dispatch() {
    MockBaro baro;
    baro.present = true;
    baro.sample.altitude_m = 42.0f; baro.sample.valid = true;
    IBarometer* p = &baro;                 // 通过基类指针调用
    BaroSample s{};
    TEST_ASSERT_TRUE(p->probe());
    TEST_ASSERT_TRUE(p->read(s));
    TEST_ASSERT_EQUAL_FLOAT(42.0f, s.altitude_m);

    // read 失败时不污染 out
    baro.present = false;
    BaroSample keep{}; keep.altitude_m = 7.0f;
    TEST_ASSERT_FALSE(p->read(keep));
    TEST_ASSERT_EQUAL_FLOAT(7.0f, keep.altitude_m);   // 仍为原值
}

void test_frontend_passthrough_fills_bundle() {
    MockGyroAccel imu; imu.present = true;
    imu.sample.gyro_x = 2.0f; imu.sample.valid = true;
    MockBaro baro; baro.present = true;
    baro.sample.altitude_m = 123.0f; baro.sample.valid = true;

    SensorFrontend fe;
    fe.setGyroAccel(&imu);
    fe.setBarometer(&baro);
    fe.begin();                 // 调 probe+init

    SensorBundle b{};
    fe.poll(b);
    TEST_ASSERT_TRUE(b.imu.valid);
    TEST_ASSERT_EQUAL_FLOAT(2.0f, b.imu.gyro_x);
    TEST_ASSERT_TRUE(b.baro.valid);
    TEST_ASSERT_EQUAL_FLOAT(123.0f, b.baro.altitude_m);
}

void test_tier_base_requires_imu_mag_baro() {
    MockGyroAccel imu; MockMag mag; MockBaro baro;
    imu.present = mag.present = baro.present = true;
    SensorFrontend fe;
    fe.setGyroAccel(&imu); fe.setMagnetometer(&mag); fe.setBarometer(&baro);
    fe.begin();
    TEST_ASSERT_EQUAL_INT((int)SensorTier::Base, (int)fe.tier());
}

void test_tier_none_without_mag() {
    // 缺磁力计：达不到 base（设计 §6.0/附录 C：base 必须 9 轴）
    MockGyroAccel imu; MockBaro baro;
    imu.present = baro.present = true;
    SensorFrontend fe;
    fe.setGyroAccel(&imu); fe.setBarometer(&baro);
    fe.begin();
    TEST_ASSERT_EQUAL_INT((int)SensorTier::None, (int)fe.tier());
}

void test_tier_plus_with_gps() {
    MockGyroAccel imu; MockMag mag; MockBaro baro; MockGnss gps;
    imu.present = mag.present = baro.present = gps.present = true;
    SensorFrontend fe;
    fe.setGyroAccel(&imu); fe.setMagnetometer(&mag);
    fe.setBarometer(&baro); fe.setGnss(&gps);
    fe.begin();
    TEST_ASSERT_EQUAL_INT((int)SensorTier::Plus, (int)fe.tier());
}

void test_absent_sensor_leaves_bundle_invalid() {
    MockGyroAccel imu; imu.present = false;
    SensorFrontend fe;
    fe.setGyroAccel(&imu);
    fe.begin();
    SensorBundle b{};
    fe.poll(b);
    TEST_ASSERT_FALSE(b.imu.valid);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_tier_enum_ordered);
    RUN_TEST(test_gnss_sample_defaults_invalid);
    RUN_TEST(test_airspeed_sample_defaults_invalid);
    RUN_TEST(test_mock_gyroaccel_probe_and_read);
    RUN_TEST(test_mock_absent_probe_fails);
    RUN_TEST(test_interface_polymorphic_dispatch);
    RUN_TEST(test_frontend_passthrough_fills_bundle);
    RUN_TEST(test_tier_base_requires_imu_mag_baro);
    RUN_TEST(test_tier_none_without_mag);
    RUN_TEST(test_tier_plus_with_gps);
    RUN_TEST(test_absent_sensor_leaves_bundle_invalid);
    return UNITY_END();
}
