#include "unity.h"
#include "config/capabilities.h"
#include "config/board_config.h"
#include "config/config_store.h"
#include "controller.h"
using namespace wp;

void setUp() {} void tearDown() {}

void test_pc_test_build_enables_all_capabilities() {
    // PC/测试构建默认全开（算法都要在 SITL 验）
    TEST_ASSERT_TRUE(kHasImu);
    TEST_ASSERT_TRUE(kHasBaro);
    TEST_ASSERT_TRUE(kHasMag);
    TEST_ASSERT_TRUE(kHasGps);
    TEST_ASSERT_TRUE(kHasAirspeed);
}

void test_board_pins_defined() {
    // 板级 profile 暴露引脚常量（值见设计 §6.3）
    TEST_ASSERT_EQUAL_INT(18, kPinI2cSda);
    TEST_ASSERT_EQUAL_INT(39, kPinI2cScl);
    TEST_ASSERT_EQUAL_INT(0x68, kAddrImu);
    TEST_ASSERT_EQUAL_INT(0x76, kAddrBaro);
    TEST_ASSERT_EQUAL_INT(0x0E, kAddrMag);   // IST8310 挂主 I2C
    TEST_ASSERT_EQUAL_INT(5, kPinGpsRx);     // MCU 收 = 模块 TX
    TEST_ASSERT_EQUAL_INT(4, kPinGpsTx);     // MCU 发 = 模块 RX
    TEST_ASSERT_EQUAL_INT(38400, kGpsBaud);
    TEST_ASSERT_EQUAL_INT(0x28, kAddrAirspeed);  // MS4525DO 挂主 I2C
}

void test_serialize_roundtrip_preserves_config() {
    ControllerConfig cfg{};
    cfg.airframe = Airframe::Flaperon;
    cfg.glimit.soft_g = 5.5f;
    cfg.stab.angle_roll.kp = 0.099f;

    uint8_t buf[kConfigBlobSize];
    uint16_t n = serializeConfig(cfg, buf, sizeof(buf));
    TEST_ASSERT_TRUE(n > 0);

    ControllerConfig out{};
    bool ok = deserializeConfig(buf, n, out);
    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_INT((int)Airframe::Flaperon, (int)out.airframe);
    TEST_ASSERT_EQUAL_FLOAT(5.5f, out.glimit.soft_g);
    TEST_ASSERT_EQUAL_FLOAT(0.099f, out.stab.angle_roll.kp);
}

// 自动配平相关字段经序列化 round-trip 应保持（序列化是整 struct memcpy，POD 字段自动随之持久化）。
void test_trim_fields_roundtrip() {
    ControllerConfig cfg;
    cfg.roll_trim = 0.123f;
    cfg.pitch_trim = -0.077f;
    cfg.auto_trim_enabled = true;
    uint8_t buf[kConfigBlobSize];
    uint16_t n = serializeConfig(cfg, buf, sizeof(buf));
    TEST_ASSERT_TRUE(n > 0);
    ControllerConfig out;
    TEST_ASSERT_TRUE(deserializeConfig(buf, n, out));
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.123f, out.roll_trim);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, -0.077f, out.pitch_trim);
    TEST_ASSERT_TRUE(out.auto_trim_enabled);
}

void test_deserialize_rejects_bad_crc() {
    ControllerConfig cfg{};
    uint8_t buf[kConfigBlobSize];
    uint16_t n = serializeConfig(cfg, buf, sizeof(buf));
    buf[n - 1] ^= 0xFF;  // 破坏 CRC
    ControllerConfig out{};
    TEST_ASSERT_FALSE(deserializeConfig(buf, n, out));
}

void test_deserialize_rejects_wrong_version() {
    ControllerConfig cfg{};
    uint8_t buf[kConfigBlobSize];
    uint16_t n = serializeConfig(cfg, buf, sizeof(buf));
    buf[2] = 0xEE;  // 破坏 version 字节（magic 完好，确保走到 version 检查）
    ControllerConfig out{};
    TEST_ASSERT_FALSE(deserializeConfig(buf, n, out));
}

void test_deserialize_rejects_bad_magic() {
    ControllerConfig cfg{};
    uint8_t buf[kConfigBlobSize];
    uint16_t n = serializeConfig(cfg, buf, sizeof(buf));
    buf[0] = 0xEE;  // 破坏 magic 低字节
    ControllerConfig out{};
    TEST_ASSERT_FALSE(deserializeConfig(buf, n, out));
}

void test_backend_save_load_via_memory() {
    MemoryConfigBackend backend;
    ConfigStore store(&backend);
    ControllerConfig cfg{};
    cfg.glimit.hard_g = 8.0f;
    TEST_ASSERT_TRUE(store.save(cfg));
    ControllerConfig out{};
    TEST_ASSERT_TRUE(store.load(out));
    TEST_ASSERT_EQUAL_FLOAT(8.0f, out.glimit.hard_g);
}

void test_load_returns_false_when_empty() {
    MemoryConfigBackend backend;  // 空
    ConfigStore store(&backend);
    ControllerConfig out{};
    TEST_ASSERT_FALSE(store.load(out));
}

// 新增字段往返：landing_gear + gear_last_state 序列化后能原样读回
void test_landing_gear_config_roundtrip() {
    wp::ControllerConfig in;
    in.landing_gear.enabled = true;
    in.landing_gear.stall_current_a = 1.5f;
    in.landing_gear.stall_debounce_ms = 120;
    in.gear_last_state = 2;   // Deployed
    uint8_t buf[wp::kConfigBlobSize];
    uint16_t n = wp::serializeConfig(in, buf, sizeof(buf));
    TEST_ASSERT_TRUE(n > 0);
    wp::ControllerConfig out;
    TEST_ASSERT_TRUE(wp::deserializeConfig(buf, n, out));
    TEST_ASSERT_TRUE(out.landing_gear.enabled);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 1.5f, out.landing_gear.stall_current_a);
    TEST_ASSERT_EQUAL_UINT16(120, out.landing_gear.stall_debounce_ms);
    TEST_ASSERT_EQUAL_UINT8(2, out.gear_last_state);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_pc_test_build_enables_all_capabilities);
    RUN_TEST(test_board_pins_defined);
    RUN_TEST(test_serialize_roundtrip_preserves_config);
    RUN_TEST(test_trim_fields_roundtrip);
    RUN_TEST(test_deserialize_rejects_bad_crc);
    RUN_TEST(test_deserialize_rejects_wrong_version);
    RUN_TEST(test_deserialize_rejects_bad_magic);
    RUN_TEST(test_backend_save_load_via_memory);
    RUN_TEST(test_load_returns_false_when_empty);
    RUN_TEST(test_landing_gear_config_roundtrip);
    return UNITY_END();
}
