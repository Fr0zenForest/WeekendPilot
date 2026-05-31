#pragma once
#include <cstdint>

namespace wp {

enum class GearState : uint8_t {
    Retracted = 0,   // 已收起（稳态）
    Deploying = 1,   // 正在放下（驱动中，等堵转）
    Deployed  = 2,   // 已放下（稳态）
    Retracting= 3,   // 正在收起（驱动中，等堵转）
    Fault     = 4,   // 行程超时无堵转（防烧，停驱动）
};

// 驱动方向语义（由 ILandingGearActuator 翻译成 PWM 或 H 桥电平）。
enum class GearDrive : uint8_t {
    Stop    = 0,   // 停（B: 1500µs；C: H桥置零）—— 软件停舵（D7）
    Deploy  = 1,   // 朝放下方向驱动
    Retract = 2,   // 朝收起方向驱动
};

struct LandingGearConfig {
    bool     enabled = false;          // 总开关（默认关，全写好先不使能）
    float    stall_current_a = 2.0f;   // 堵转判定电流阈值（A）
    uint16_t stall_debounce_ms = 80;   // 电流超阈持续多久判定到位（去抖）
    uint16_t timeout_ms = 4000;        // 行程超时无堵转 -> Fault（防烧）
};

struct LandingGearInputs {
    bool  deploy_cmd = false;   // 飞手指令：true=要放下 false=要收起
    float current_a = 0.0f;     // 本路电流采样（A，来自 INA3221）
    bool  alert = false;        // INA3221 硬件 ALERT（快速路径，可选；为 true 时等价电流超阈）
    float dt = 0.0f;            // 秒
    bool  link_ok = true;       // CRSF 链路有效；丢失时冻结指令沿处理（不收放/不退Fault）
};

struct LandingGearOutput {
    GearDrive drive = GearDrive::Stop;
    GearState state = GearState::Retracted;
};

// 起落架堵转检测状态机（纯逻辑，无硬件）。每周期 update() 一次。
class LandingGear {
public:
    void setConfig(const LandingGearConfig& c);
    // 用初始状态播种（上电从 NVS 恢复"上次状态"，不主动驱动 —— 见设计文档 §9）。
    void initState(GearState s);
    LandingGearOutput update(const LandingGearInputs& in);
    GearState state() const { return state_; }

private:
    LandingGearConfig cfg_;
    GearState state_ = GearState::Retracted;
    bool      last_deploy_cmd_ = false;   // 上拍指令（检测跳变沿）
    bool      have_last_cmd_ = false;     // 是否已记录过指令（防上电首拍误触发）
    float     stall_timer_ms_ = 0.0f;     // 电流超阈连续时间
    float     travel_timer_ms_ = 0.0f;    // 本次行程已耗时
};

// 起落架执行机构抽象（HAL 实现）。状态机产出 GearDrive，由实现翻译为物理驱动。
class ILandingGearActuator {
public:
    virtual ~ILandingGearActuator() = default;
    virtual void apply(GearDrive d) = 0;   // B: 写舵机 us；C: 设 H桥方向+使能
};

}  // namespace wp
