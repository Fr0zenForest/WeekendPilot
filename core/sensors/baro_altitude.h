#pragma once

namespace wp {

// 国际标准大气海平面气压 (Pa)。
constexpr float kSeaLevelPa = 101325.0f;

// 气压(Pa) → 相对参考点的高度(m)。ISA 对流层公式：
//   h = 44330 * (1 - (p / p_ref)^0.1903)
// p_ref 为零点基准气压；传 kSeaLevelPa 得相对标准海平面的绝对高度（受天气误差）。
// p<=0 或 p_ref<=0 返回 0。
float baroPressureToAltitude(float pressure_pa, float p_ref_pa);

// 有状态封装：起飞前 setReference() 锁基准，之后 altitude() 出相对高度。
class BaroAltitude {
public:
    void setReference(float p_ref_pa) { p_ref_ = (p_ref_pa > 0.0f) ? p_ref_pa : kSeaLevelPa; }
    float reference() const { return p_ref_; }
    float altitude(float pressure_pa) const { return baroPressureToAltitude(pressure_pa, p_ref_); }
private:
    float p_ref_ = kSeaLevelPa;   // 默认未置零 -> 绝对(标准海平面)高度
};

}  // namespace wp
