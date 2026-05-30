#include "unity.h"
#include "sensors/ms4525_decode.h"
#include <cmath>
using namespace wp;

void setUp() {} void tearDown() {}

// ⚠️ 公式与 ArduPilot AP_Airspeed_MS4525 对拍，非实物抓包。
// 零差压点是输出范围中点：dp_raw=0.5*16383≈8192（原注释误写为0.1*16383=1638，
// 1638实为量程下限，对应-P_max=-1psi，已更正）。
void test_zero_pressure_at_offset_point() {
    // dp_raw=8192 是公式零差压点；status=0；dT 任意非饱和（取 1024<<5）
    uint16_t dp = 8192, dT = 1024;
    uint8_t data[4] = {
        (uint8_t)((dp >> 8) & 0x3F),          // status=0 在高2位
        (uint8_t)(dp & 0xFF),
        (uint8_t)((dT << 5 >> 8) & 0xFF),
        (uint8_t)((dT << 5) & 0xFF)
    };
    Ms4525Raw r = ms4525Decode(data);
    TEST_ASSERT_TRUE(r.valid);
    TEST_ASSERT_FLOAT_WITHIN(2.0, 0.0f, r.diff_press_pa);  // 零点附近
}

void test_status_stale_marks_invalid() {
    // status=2 (0x80) -> 坏数据
    uint8_t data[4] = {0x80 | 0x06, 0x66, 0x20, 0x00};
    Ms4525Raw r = ms4525Decode(data);
    TEST_ASSERT_FALSE(r.valid);
}

void test_temperature_formula() {
    // dT_raw=2046 -> (200*2046/2047)-50 ≈ 149.9°C（2047=0x7FF是饱和哨兵，用2046避开）
    uint16_t dT = 2046;
    uint8_t data[4] = {0x06, 0x66, (uint8_t)((dT<<5>>8)&0xFF), (uint8_t)((dT<<5)&0xFF)};
    Ms4525Raw r = ms4525Decode(data);
    TEST_ASSERT_FLOAT_WITHIN(0.5, 150.0f, r.temperature_c);
}

void test_bernoulli_pa_to_ias() {
    // Δp = 0.5*ρ*v^2 ; v=20 -> Δp=0.5*1.225*400=245 Pa
    TEST_ASSERT_FLOAT_WITHIN(0.1, 20.0f, ms4525PaToIas(245.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.1, 20.0f, ms4525PaToIas(-245.0f)); // 取绝对值
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, ms4525PaToIas(0.0f));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_zero_pressure_at_offset_point);
    RUN_TEST(test_status_stale_marks_invalid);
    RUN_TEST(test_temperature_formula);
    RUN_TEST(test_bernoulli_pa_to_ias);
    return UNITY_END();
}
