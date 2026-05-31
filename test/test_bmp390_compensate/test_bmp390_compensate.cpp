#include "unity.h"
#include "sensors/bmp390_compensate.h"
#include <cstring>
using namespace wp;

void setUp() {} void tearDown() {}

// ⚠️ 这是一组构造的典型 trim（量级取自 datasheet §8.4 范围），非某颗实物 NVM。
// 仅用于验证补偿管线数学自洽 + 输出物理合理，非端到端精度校验。
static void make_nvm(uint8_t nvm[21]) {
    std::memset(nvm, 0, 21);
    nvm[0] = 0x6C; nvm[1] = 0x6B;          // par_t1: 0x6B6C = 27500 -> *256
    nvm[2] = 0x50; nvm[3] = 0x46;          // par_t2: 0x4650 = 18000
    nvm[4] = 0xFD;                          // par_t3: -3
    nvm[5] = 0x80; nvm[6] = 0x3E;          // par_p1: i16 0x3E80 = 16000
    nvm[7] = 0x74; nvm[8] = 0x40;          // par_p2: i16 0x4074 = 16500
    nvm[11] = 0xD4; nvm[12] = 0x30;        // par_p5: u16 0x30D4=12500 -> *8=100000 Pa offset（必须非零，否则压力恒负）
    // p3,p4,p6..p11 留 0（高阶小项，不影响物理合理判定）
}

void test_temperature_then_pressure_pipeline() {
    uint8_t nvm[21]; make_nvm(nvm);
    Bmp390Calib c = bmp390ParseCalib(nvm);
    // ⚠️ 构造 trim 下经验调整值：raw_temp=8388608, raw_press=6500000
    // 实测输出 tC≈22.59°C, pPa≈97651 Pa（物理合理，非精度校验）
    // par_p5 必须非零（见 make_nvm），否则压力偏置项缺失导致输出恒负。
    uint32_t raw_temp  = 8388608u;
    uint32_t raw_press = 6500000u;
    float tC = bmp390CompensateTemperature(raw_temp, c);   // 先温度（更新 t_lin）
    float pPa = bmp390CompensatePressure(raw_press, c);
    TEST_ASSERT_TRUE(tC > -40.0f && tC < 85.0f);
    TEST_ASSERT_TRUE(pPa > 30000.0f && pPa < 110000.0f);
}

void test_calib_parse_is_deterministic() {
    uint8_t nvm[21]; make_nvm(nvm);
    Bmp390Calib a = bmp390ParseCalib(nvm);
    Bmp390Calib b = bmp390ParseCalib(nvm);
    TEST_ASSERT_EQUAL_FLOAT(a.par_t1, b.par_t1);
    TEST_ASSERT_EQUAL_FLOAT(a.par_p1, b.par_p1);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 7040000.0f, a.par_t1);   // 27500*256
}

void test_decode_zero_raw_invalid() {
    uint8_t nvm[21]; make_nvm(nvm);
    Bmp390Calib c = bmp390ParseCalib(nvm);
    uint8_t data[6] = {0,0,0,0,0,0};
    Bmp390Reading r = bmp390Decode(data, c);
    TEST_ASSERT_FALSE(r.valid);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_temperature_then_pressure_pipeline);
    RUN_TEST(test_calib_parse_is_deterministic);
    RUN_TEST(test_decode_zero_raw_invalid);
    return UNITY_END();
}
