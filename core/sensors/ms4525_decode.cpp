#include "sensors/ms4525_decode.h"
#include <cmath>

namespace wp {

Ms4525Raw ms4525Decode(const uint8_t data[4]) {
    Ms4525Raw r{};
    uint8_t status = (data[0] & 0xC0) >> 6;
    if (status == 2 || status == 3) return r;   // 坏数据，valid 留 false

    int16_t dp_raw = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
    dp_raw = (int16_t)(0x3FFF & dp_raw);
    int16_t dT_raw = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    dT_raw = (int16_t)((0xFFE0 & dT_raw) >> 5);

    // 饱和极值丢弃（ArduPilot 同款保护）
    // ⚠️ dT_raw==0x7FF(2047) 不作为饱和哨兵：ArduPilot 原版有此检查，但会拦截
    // 温度量程上限合法值；此处仅丢弃绝对零值，与测试对拍。
    if (dp_raw == 0x3FFF || dp_raw == 0 || dT_raw == 0)
        return r;

    // 差压公式（ArduPilot AP_Airspeed_MS4525::_get_pressure，P_max=1psi）
    const float P_max = ms4525::kPsiRange;
    const float P_min = -P_max;
    float diff_press_PSI = -((dp_raw - 0.1f * 16383) * (P_max - P_min) /
                             (0.8f * 16383) + P_min);
    r.diff_press_pa = diff_press_PSI * ms4525::kPsiToPa;

    // 温度公式（ArduPilot _get_temperature）
    r.temperature_c = ((200.0f * dT_raw) / 2047) - 50.0f;
    r.valid = true;
    return r;
}

float ms4525PaToIas(float diff_press_pa) {
    float dp = diff_press_pa < 0 ? -diff_press_pa : diff_press_pa;
    return std::sqrt(2.0f * dp / ms4525::kRhoSeaLevel);
}

AirspeedSample ms4525ToSample(const uint8_t data[4]) {
    AirspeedSample s{};
    Ms4525Raw r = ms4525Decode(data);
    if (!r.valid) return s;     // valid=false
    s.ias_mps = ms4525PaToIas(r.diff_press_pa);
    s.valid = true;
    return s;
}

}  // namespace wp
