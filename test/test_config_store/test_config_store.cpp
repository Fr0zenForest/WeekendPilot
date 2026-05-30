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

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_pc_test_build_enables_all_capabilities);
    RUN_TEST(test_board_pins_defined);
    RUN_TEST(test_serialize_roundtrip_preserves_config);
    RUN_TEST(test_deserialize_rejects_bad_crc);
    RUN_TEST(test_deserialize_rejects_wrong_version);
    RUN_TEST(test_deserialize_rejects_bad_magic);
    RUN_TEST(test_backend_save_load_via_memory);
    RUN_TEST(test_load_returns_false_when_empty);
    return UNITY_END();
}
