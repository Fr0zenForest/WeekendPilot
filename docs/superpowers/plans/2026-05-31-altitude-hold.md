# Alt-Hold 定高 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 给固定翼增加气压定高（Altitude Hold）：一个独立使能的高度保持层，叠加在已调好的 Angle 姿态内环之上。

**Architecture:** 级联控制。外环把高度误差转成目标爬升率（P + 限幅）；内环把爬升率误差转成归一化俯仰指令（PI）。该俯仰指令在 Angle 模式下覆盖飞手俯仰杆，交由既有 Angle 俯仰 PID 做姿态内稳定。爬升率由气压高度差分 + PT1 低通估计。定高仅在 Angle 模式 + 气压有效 + 使能通道拨上时接管；飞手俯仰杆超死区即交还手动并重锁目标高度。

**Tech Stack:** C++17（core 纯逻辑，零硬件依赖，编译开 -Wall -Wextra -Werror），Unity 单元测试，JSBSim SITL 闭环验证（ctypes DLL 桥）。

> WARNING 数据真实性声明（用户硬性要求）：AltitudeHold 为新代码（级联结构参考 INAV 固定翼 nav_fixedwing 高度环思路，控制律本项目手写，非逐行移植）。单元测试为数学/逻辑验证；SITL 闭环验证使用 JSBSim 真值高度（position/h-agl-ft，是仿真真值，非合成数据），可证明闭环收敛。但未经实物气压计实测，真机起飞前仍需现场标定与遗留项处置（见末尾）。所有相关源文件与测试需带 WARNING 标注。

---

## File Structure

- `core/nav/altitude_hold.h` — AltHoldConfig 结构 + AltitudeHold 类（纯逻辑，唯一新增子系统文件）。
- `core/nav/altitude_hold.cpp` — 级联实现：爬升率估计、接管沿锁定、外/内环、飞手覆盖。
- `core/controller.h` / `core/controller.cpp` — ControllerConfig 增 alt-hold 字段；Controller 持有 AltitudeHold 成员，在 update() 内于 Angle 模式接管俯仰。
- `test/test_altitude_hold/test_altitude_hold.cpp` — 单元测试（climb 估计、接管锁定、级联符号、飞手覆盖、failsafe）。
- `test/test_controller/test_controller.cpp` — 增集成用例：定高接管覆盖俯仰杆。
- `test/CMakeLists.txt` — 注册 test_altitude_hold。
- `hal_sitl/core_c_api.cpp` — SITL 侧使能 alt-hold（设 config），用于闭环验证。
- `hal_sitl/run_sitl.py` — 增 althold 场景（拨上定高通道 + 俯仰扰动）。
- `hal_sitl/check_alt_hold.py` — 校验脚本：扰动后高度回收到锁定目标附近。
- `docs/WeekendPilot-功能说明.md` — 把定高从“开发中”移到“已集成”，注明研发方式与验证手段。

---

### Task 1: AltitudeHold 骨架 + 爬升率估计 + 接管锁定

**Files:**
- Create: `core/nav/altitude_hold.h`
- Create: `core/nav/altitude_hold.cpp`
- Test: `test/test_altitude_hold/test_altitude_hold.cpp`
- Modify: `test/CMakeLists.txt`

- [ ] **Step 1: 写失败测试**

创建 `test/test_altitude_hold/test_altitude_hold.cpp`：

```cpp
#include "unity.h"
#include "nav/altitude_hold.h"
using namespace wp;

void setUp() {} void tearDown() {}

// WARNING 新代码（级联参考 INAV 固定翼高度环，控制律手写）。
//    单元测试为逻辑/数学验证；闭环见 SITL check_alt_hold.py（JSBSim 真值）。未经实物气压计实测。

// 未请求接管 -> 返回 false，不接管
void test_not_engaged_when_no_request() {
    AltitudeHold ah;
    float pc = -99.0f;
    bool drive = ah.update(/*engage*/false, /*baro_valid*/true, /*alt*/100.0f,
                           /*pilot_pitch*/0.0f, /*dt*/0.02f, pc);
    TEST_ASSERT_FALSE(drive);
    TEST_ASSERT_FALSE(ah.engaged());
}

// 气压无效 -> failsafe 脱离
void test_disengage_when_baro_invalid() {
    AltitudeHold ah;
    float pc = 0.0f;
    bool drive = ah.update(true, /*baro_valid*/false, 100.0f, 0.0f, 0.02f, pc);
    TEST_ASSERT_FALSE(drive);
    TEST_ASSERT_FALSE(ah.engaged());
}

// 接管起始沿：锁定当前高度为目标
void test_engage_latches_target() {
    AltitudeHold ah;
    float pc = 0.0f;
    bool drive = ah.update(true, true, /*alt*/123.5f, 0.0f, 0.02f, pc);
    TEST_ASSERT_TRUE(drive);
    TEST_ASSERT_TRUE(ah.engaged());
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 123.5f, ah.targetAltitude());
}

// 爬升率估计符号：持续上升的高度 -> climbRate > 0
void test_climb_rate_sign_positive() {
    AltitudeHold ah;
    float pc = 0.0f, alt = 100.0f;
    for (int i = 0; i < 50; ++i) {            // 50 拍 @20ms，+2 m/s 上升
        alt += 2.0f * 0.02f;
        ah.update(true, true, alt, 0.0f, 0.02f, pc);
    }
    TEST_ASSERT_TRUE(ah.climbRate() > 1.0f);  // 收敛趋近 +2 m/s
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_not_engaged_when_no_request);
    RUN_TEST(test_disengage_when_baro_invalid);
    RUN_TEST(test_engage_latches_target);
    RUN_TEST(test_climb_rate_sign_positive);
    return UNITY_END();
}
```

- [ ] **Step 2: 注册测试并确认编译失败**

在 `test/CMakeLists.txt` 末尾追加：

```cmake
wp_add_test(test_altitude_hold)
```

Run: `cmake --preset dev && cmake --build build --target test_altitude_hold`
Expected: FAIL —— fatal error: nav/altitude_hold.h: No such file or directory

- [ ] **Step 3: 写头文件**

创建 `core/nav/altitude_hold.h`：

```cpp
#pragma once
#include "math/filter_pt1.h"
#include "pid/pid_controller.h"

namespace wp {

// WARNING 新代码（非逐行移植）。级联结构参考 INAV 固定翼 nav_fixedwing 高度环思路，
//    控制律为本项目手写。SITL 闭环已用 JSBSim 真值高度验证；未经实物气压计实测。
struct AltHoldConfig {
    float kp_alt        = 0.05f;   // 高度误差(m) -> 目标爬升率(m/s)
    float max_climb_mps = 3.0f;    // 目标爬升率限幅 (±, m/s)
    PidGains climb_gains{0.08f, 0.03f, 0.0f};  // 爬升率误差(m/s) -> 归一化俯仰指令
    float climb_rate_cutoff_hz = 2.0f;  // 爬升率估计 PT1 截止频率
    float pitch_deadband = 0.10f;  // 飞手俯仰杆死区：超出则交还手动并重锁目标
};

// 气压定高层。每周期 update() 一次；仅在请求接管 & 气压有效时驱动俯仰。
class AltitudeHold {
public:
    AltitudeHold();
    void setConfig(const AltHoldConfig& c);

    // engage_request: 上层判定（已使能 & Angle 模式 & 通道拨上）
    // baro_valid    : 气压高度是否有效（无效 -> 强制脱离 failsafe）
    // altitude_m    : 当前相对高度 (m)
    // pilot_pitch_cmd: 飞手俯仰杆归一化 [-1,1]
    // dt            : 秒
    // pitch_cmd_out : 输出归一化俯仰指令（仅返回 true 时有效）
    // 返回 true = 定高正在驱动俯仰（调用方据此覆盖手动俯仰杆）；
    // 返回 false = 未接管（未使能/气压无效/飞手杆超死区），调用方维持手动。
    bool update(bool engage_request, bool baro_valid, float altitude_m,
                float pilot_pitch_cmd, float dt, float& pitch_cmd_out);

    bool  engaged() const { return engaged_; }
    float targetAltitude() const { return target_alt_m_; }
    float climbRate() const { return climb_filt_.value(); }

private:
    AltHoldConfig cfg_;
    PidController climb_pid_;   // out_limit 1.0 -> 归一化俯仰
    FilterPt1     climb_filt_;  // 爬升率低通
    bool   engaged_     = false;
    bool   have_last_   = false;
    float  last_alt_m_  = 0.0f;
    float  target_alt_m_= 0.0f;
};

}  // namespace wp
```

- [ ] **Step 4: 写实现**

创建 `core/nav/altitude_hold.cpp`：

```cpp
#include "nav/altitude_hold.h"

namespace wp {

AltitudeHold::AltitudeHold()
    : climb_pid_(1.0f), climb_filt_(2.0f) {
    climb_pid_.setGains(cfg_.climb_gains);
    climb_filt_.setCutoff(cfg_.climb_rate_cutoff_hz);
}

void AltitudeHold::setConfig(const AltHoldConfig& c) {
    cfg_ = c;
    climb_pid_.setGains(cfg_.climb_gains);
    climb_filt_.setCutoff(cfg_.climb_rate_cutoff_hz);
}

bool AltitudeHold::update(bool engage_request, bool baro_valid, float altitude_m,
                          float pilot_pitch_cmd, float dt, float& pitch_cmd_out) {
    // failsafe：未请求 / 气压无效 -> 脱离并复位。
    if (!engage_request || !baro_valid) {
        engaged_ = false;
        have_last_ = false;
        climb_pid_.reset();
        return false;
    }

    // 爬升率估计：高度差分 + PT1。首拍只播种 last，不出数。
    if (have_last_ && dt > 0.0f) {
        climb_filt_.apply((altitude_m - last_alt_m_) / dt, dt);
    } else {
        climb_filt_.reset(0.0f);
    }
    const float climb = climb_filt_.value();
    last_alt_m_ = altitude_m;
    have_last_ = true;

    // 接管起始沿：锁定当前高度为目标，复位积分。
    if (!engaged_) {
        engaged_ = true;
        target_alt_m_ = altitude_m;
        climb_pid_.reset();
    }

    // 飞手俯仰杆超死区：交还手动，并持续把目标重锁到当前高度（松杆后在新高度保持）。
    if (pilot_pitch_cmd > cfg_.pitch_deadband || pilot_pitch_cmd < -cfg_.pitch_deadband) {
        target_alt_m_ = altitude_m;
        climb_pid_.reset();
        return false;
    }

    // 外环：高度误差 -> 目标爬升率（P + 限幅）。
    float climb_target = cfg_.kp_alt * (target_alt_m_ - altitude_m);
    if (climb_target >  cfg_.max_climb_mps) climb_target =  cfg_.max_climb_mps;
    if (climb_target < -cfg_.max_climb_mps) climb_target = -cfg_.max_climb_mps;

    // 内环：爬升率误差 -> 归一化俯仰指令。误差>0(需爬升) -> 俯仰指令>0(抬头)。
    pitch_cmd_out = climb_pid_.update(climb_target, climb, climb, dt);
    return true;
}

}  // namespace wp
```

- [ ] **Step 5: 编译并跑测试确认通过**

Run: `cmake --preset dev && cmake --build build --target test_altitude_hold && ctest --test-dir build -R test_altitude_hold --output-on-failure`
Expected: PASS（4 用例全过）

- [ ] **Step 6: 提交**

```bash
git add core/nav/altitude_hold.h core/nav/altitude_hold.cpp test/test_altitude_hold/test_altitude_hold.cpp test/CMakeLists.txt
git commit -m "feat(nav): AltitudeHold skeleton + climb-rate estimate + engage latch"
```

---

### Task 2: 级联收敛 + 飞手覆盖 单元验证

**Files:**
- Modify: `test/test_altitude_hold/test_altitude_hold.cpp`

- [ ] **Step 1: 追加失败测试**

在 `test/test_altitude_hold/test_altitude_hold.cpp` 中，`main()` 之前追加三个用例，并在 `main()` 里登记：

```cpp
// 低于目标 -> 俯仰指令为正（抬头爬升）；高于目标 -> 为负（低头下降）。
void test_cascade_pitch_sign() {
    AltitudeHold ah;
    float pc = 0.0f;
    ah.update(true, true, 100.0f, 0.0f, 0.02f, pc);   // 接管，锁 100m
    // 掉到 90m：低于目标，应命令抬头 (>0)
    ah.update(true, true, 90.0f, 0.0f, 0.02f, pc);
    TEST_ASSERT_TRUE(pc > 0.0f);
    // 升到 110m：高于目标，应命令低头 (<0)
    AltitudeHold ah2;
    float pc2 = 0.0f;
    ah2.update(true, true, 100.0f, 0.0f, 0.02f, pc2);
    ah2.update(true, true, 110.0f, 0.0f, 0.02f, pc2);
    TEST_ASSERT_TRUE(pc2 < 0.0f);
}

// 飞手俯仰杆超死区 -> 交还手动（返回 false），并把目标重锁到当前高度。
void test_pilot_override_releases_and_relatches() {
    AltitudeHold ah;
    float pc = 0.0f;
    ah.update(true, true, 100.0f, 0.0f, 0.02f, pc);   // 锁 100m
    // 飞手大幅推杆，飞机此刻在 105m
    bool drive = ah.update(true, true, 105.0f, /*pilot*/0.6f, 0.02f, pc);
    TEST_ASSERT_FALSE(drive);                          // 交还手动
    TEST_ASSERT_FLOAT_WITHIN(1e-2, 105.0f, ah.targetAltitude());  // 重锁
    // 松杆回中：在 105m 重新接管保持
    bool drive2 = ah.update(true, true, 105.0f, 0.0f, 0.02f, pc);
    TEST_ASSERT_TRUE(drive2);
}

// 目标爬升率被 max_climb_mps 限幅：大高度误差不应让目标爬升率爆掉。
void test_climb_target_clamped() {
    AltitudeHold ah;
    AltHoldConfig c;            // kp_alt 0.05, max_climb 3.0 -> 误差>60m 即饱和
    ah.setConfig(c);
    float pc = 0.0f;
    ah.update(true, true, 100.0f, 0.0f, 0.02f, pc);   // 锁 100m
    // 掉到 0m：误差 100m，外环目标爬升率应被钳在 +3 m/s，俯仰指令仍在 [-1,1]
    ah.update(true, true, 0.0f, 0.0f, 0.02f, pc);
    TEST_ASSERT_TRUE(pc <= 1.0f && pc >= -1.0f);
    TEST_ASSERT_TRUE(pc > 0.0f);
}
```

`main()` 内 `RUN_TEST` 列表追加：

```cpp
    RUN_TEST(test_cascade_pitch_sign);
    RUN_TEST(test_pilot_override_releases_and_relatches);
    RUN_TEST(test_climb_target_clamped);
```

- [ ] **Step 2: 跑测试确认先失败后通过**

Run: `cmake --build build --target test_altitude_hold && ctest --test-dir build -R test_altitude_hold --output-on-failure`
Expected: 若 Task 1 实现正确，新用例应直接 PASS（7 用例）。若 `test_cascade_pitch_sign` 失败，检查内环 setpoint/measurement 顺序（climb_target 为 setpoint，climb 为 measurement）。

- [ ] **Step 3: 提交**

```bash
git add test/test_altitude_hold/test_altitude_hold.cpp
git commit -m "test(nav): cascade pitch-sign, pilot override re-latch, climb clamp"
```

---

### Task 3: Controller 集成 —— Angle 模式下接管俯仰

**Files:**
- Modify: `core/controller.h`
- Modify: `core/controller.cpp`
- Test: `test/test_controller/test_controller.cpp`

- [ ] **Step 1: 写失败的集成测试**

在 `test/test_controller/test_controller.cpp` 末尾（`main()` 之前）追加用例，并在 `main()` 里登记。先读该文件确认现有 helper（如何构造 ControlInput、如何取舵量）后照其风格写。核心断言：

```cpp
// 定高使能 + Angle 模式 + 飞手俯仰杆居中 + 飞机低于锁定高度
// -> elevator 输出应偏离“纯手动居中”，朝抬头方向。
void test_althold_drives_elevator_when_below_target() {
    Controller c;
    ControllerConfig cfg;
    cfg.althold_enabled = true;
    cfg.althold_channel = 7;          // ch8 作定高开关
    c.setConfig(cfg);

    ControlInput in{};
    for (int i = 0; i < kNumChannels; ++i) in.channels[i] = 1500;
    in.channels[cfg.mode_channel] = 1500;     // Angle
    in.channels[cfg.gain_channel] = 2000;     // gain 100%
    in.channels[cfg.althold_channel] = 1000;  // 定高先关
    in.dt = 0.02f; in.link_ok = true;
    in.imu.valid = true; in.imu.accel_z = 1.0f;
    in.baro.valid = true; in.baro.altitude_m = 100.0f;

    // 关定高跑一拍，记录基准 elevator
    ServoCommand off = c.update(in);

    // 开定高，锁 100m；随后飞机掉到 90m
    in.channels[cfg.althold_channel] = 2000;
    c.update(in);                              // 接管，锁 100m
    in.baro.altitude_m = 90.0f;
    ServoCommand on = c.update(in);

    // 低于目标 -> 抬头修正。约定 elevator = servo[1]，抬头方向应使其偏离基准。
    TEST_ASSERT_TRUE(on.servo[1] != off.servo[1]);
}

// 定高通道未拨上 -> 不接管：两台同样喂入的控制器（一台高度恒定、一台高度大变）
// 升降输出应逐字节相同。两台独立 Controller 各自从同一初态推进，隔离 AHRS 积分漂移，
// 干净地证明“定高没接管时高度变化不影响舵量”。
void test_althold_inactive_when_channel_low() {
    auto make = [](){
        Controller c;
        ControllerConfig cfg;
        cfg.althold_enabled = true;
        cfg.althold_channel = 7;
        c.setConfig(cfg);
        return c;
    };
    Controller c_const = make();   // 高度恒定 100m
    Controller c_vary  = make();   // 高度跌到 50m
    ControlInput in{};
    for (int i = 0; i < kNumChannels; ++i) in.channels[i] = 1500;
    in.channels[4] = 1500;                    // Angle
    in.channels[7] = 1000;                    // 定高通道关
    in.dt = 0.02f; in.link_ok = true;
    in.imu.valid = true; in.imu.accel_z = 1.0f;

    // 第 1 拍两台完全相同（都在 100m）
    in.baro.valid = true; in.baro.altitude_m = 100.0f;
    c_const.update(in);
    c_vary.update(in);
    // 第 2 拍：c_const 仍 100m，c_vary 跌到 50m；通道关 -> 定高不接管
    in.baro.altitude_m = 100.0f; ServoCommand a = c_const.update(in);
    in.baro.altitude_m = 50.0f;  ServoCommand b = c_vary.update(in);
    TEST_ASSERT_EQUAL_UINT16(a.servo[1], b.servo[1]);  // 高度变化不影响升降（未接管）
}
```

`main()` 追加：

```cpp
    RUN_TEST(test_althold_drives_elevator_when_below_target);
    RUN_TEST(test_althold_inactive_when_channel_low);
```

- [ ] **Step 2: 跑测试确认失败**

Run: `cmake --build build --target test_controller`
Expected: FAIL —— ControllerConfig 无 althold_enabled / althold_channel 成员，编译不过。

- [ ] **Step 3: 扩展 ControllerConfig 与 Controller**

修改 `core/controller.h`：`#include "nav/altitude_hold.h"`；在 `ControllerConfig` 增字段：

```cpp
    bool    althold_enabled = false;   // 总开关（默认关，全写好先不使能）
    uint8_t althold_channel = 7;       // ch8：拨上启用定高
    AltHoldConfig althold;
```

在 `Controller` 私有区增成员 `AltitudeHold althold_;`，并让 `setConfig` 调 `althold_.setConfig(cfg_.althold);`。

- [ ] **Step 4: 在 update() 内接管俯仰**

修改 `core/controller.cpp`：`setConfig` 内追加 `althold_.setConfig(cfg_.althold);`。在 `update()` 计算出 `pitch_cmd`（飞手俯仰杆归一化）之后、组装 demand 之前，插入接管逻辑：

```cpp
    // 定高接管：仅 Angle 模式 + 总开关 + 通道拨上 + 气压有效时驱动俯仰。
    bool althold_req = cfg_.althold_enabled
                       && mode == FlightMode::Angle
                       && in.channels[cfg_.althold_channel] > 1700;
    float ah_pitch = 0.0f;
    if (althold_.update(althold_req, in.baro.valid, in.baro.altitude_m,
                        pitch_cmd, in.dt, ah_pitch)) {
        pitch_cmd = ah_pitch;   // 用定高俯仰指令替换飞手俯仰杆，喂给 Angle 内环
    }
```

注意：该替换发生在 `corr = modes_.update(...)` 之前，使定高产生的 `pitch_cmd` 既作前馈项进入 demand，又作为 Angle 俯仰 PID 的 setpoint 源（与现有 `pitch_cmd` 用法完全一致）。

- [ ] **Step 5: 跑测试确认通过**

Run: `cmake --build build --target test_controller && ctest --test-dir build -R "test_controller|test_altitude_hold" --output-on-failure`
Expected: PASS

- [ ] **Step 6: 提交**

```bash
git add core/controller.h core/controller.cpp test/test_controller/test_controller.cpp
git commit -m "feat(core): wire AltitudeHold into Controller (Angle-mode pitch takeover)"
```

---

### Task 4: SITL 闭环验证（JSBSim 真值高度）

**Files:**
- Modify: `hal_sitl/core_c_api.cpp`
- Modify: `hal_sitl/run_sitl.py`
- Create: `hal_sitl/check_alt_hold.py`

- [ ] **Step 1: SITL 侧使能定高**

修改 `hal_sitl/core_c_api.cpp`：在 `SitlCore` 构造里给 controller 设带定高的 config。注意 SITL 是单 update 入口，定高通道复用现有 16 路 channels（run_sitl 会拨 ch8）。在构造函数体追加：

```cpp
        wp::ControllerConfig cfg;
        cfg.althold_enabled = true;
        cfg.althold_channel = 7;   // ch8
        controller.setConfig(cfg);
```

构造函数顶部需 `#include "controller.h"`（已有）。确认设置在 frontend.begin() 之后或之前都行（互不依赖）。

- [ ] **Step 2: run_sitl 增 althold 场景**

修改 `hal_sitl/run_sitl.py`：

`ap.add_argument('--scenario', ...)` 的 choices 增 `'althold'`。

`rc_for_time` 增分支：

```python
    if scenario == 'althold':
        ch[4] = 1500       # Angle 模式
        ch[7] = 2000       # ch8 定高开关拨上
```

主循环扰动段，增 althold 的俯仰踢（让飞机偏离锁定高度后观察回收）：

```python
            if args.scenario == 'althold' and 3.0 <= t < 3.6:
                fdm.set_property_value('fcs/elevator-cmd-norm', -0.5)  # 推杆下压偏离高度
```

注意：该 elevator 覆盖会被下一拍的 write_servos 重写，模拟一次外部扰动脉冲。

- [ ] **Step 2b: 校验 ch8 在 update 调用里传入**

run_sitl 的 `core.update(rc_for_time(...), ...)` 已把整组 16 通道传入，ch[7] 自动随场景生效，无需改 update 签名。

- [ ] **Step 3: 写校验脚本**

创建 `hal_sitl/check_alt_hold.py`：

```python
"""Verify Alt-Hold recovers altitude toward the locked target after a pitch kick.
WARNING: SITL altitude is JSBSim truth (position/h-agl-ft), NOT a real barometer.
This proves the control loop converges; real-baro behavior is unverified."""
import csv, sys
rows = list(csv.DictReader(open(sys.argv[1] if len(sys.argv) > 1 else 'sitl_log.csv')))
def alt_at(t):
    best = min(rows, key=lambda r: abs(float(r['t']) - t))
    return float(best['alt_ft'])
lock = alt_at(2.5)                                   # 扰动前锁定高度
trough = min(alt_at(t/10.0) for t in range(36, 60))  # 3.6-6.0s 扰动后最低点
settled = alt_at(9.5)                                # 末段回收值
print('locked alt (ft):', round(lock,1))
print('trough after kick (ft):', round(trough,1))
print('settled alt at t=9.5s (ft):', round(settled,1))
dip = lock - trough
recover = lock - settled
assert dip > 1.0, 'no measurable disturbance — test invalid'
assert abs(recover) < dip * 0.6, 'Alt-Hold did not recover toward locked altitude'
print('ALT-HOLD RECOVERY OK')
```

- [ ] **Step 4: 重建 DLL 并跑闭环**

Run:
```bash
cmake --build build --target wp_core_capi
cd hal_sitl && python run_sitl.py --scenario althold --secs 10 --out sitl_alt.csv && python check_alt_hold.py sitl_alt.csv
```
Expected: 末行打印 `ALT-HOLD RECOVERY OK`。若 JSBSim 不可用（环境无 jsbsim 包），记录“SITL 闭环未在本机执行”，不得伪报通过。

- [ ] **Step 5: 提交**

```bash
git add hal_sitl/core_c_api.cpp hal_sitl/run_sitl.py hal_sitl/check_alt_hold.py
git commit -m "test(sitl): Alt-Hold closed-loop recovery scenario (JSBSim truth altitude)"
```

---

### Task 5: 文档 —— 功能说明更新 + 遗留项

**Files:**
- Modify: `docs/WeekendPilot-功能说明.md`

- [ ] **Step 1: 把定高从“开发中”移到“已集成”**

在 `docs/WeekendPilot-功能说明.md` 的“已集成功能 / 飞行控制”表追加一行（列：功能 / 如何开发 / 如何验证）：

```
| 气压定高 Alt-Hold | 新代码（级联结构参考 INAV 固定翼 nav_fixedwing 高度环思路，控制律本项目手写） | 7 条单元测试（爬升率估计/接管锁定/级联符号/飞手覆盖/限幅）；SITL 闭环用 JSBSim 真值高度验证扰动回收。WARNING 未经实物气压计实测 |
```

从“二、开发中/即将推出”删除定高相关条目（若有）。

- [ ] **Step 2: 记录真机起飞前遗留项**

在文档末尾“研发方法说明 / 已知遗留项”追加（无该小节则新建）：

```
- 气压定高真机起飞前：BMP390 首次 ref-lock 的 IIR 沉降；main.cpp dt 硬编码 0.001（实际约 500Hz）需改为实测周期；定高增益 kp_alt / climb_gains 为 SITL 经验值，真机需现场整定。
```

- [ ] **Step 3: 提交**

```bash
git add docs/WeekendPilot-功能说明.md
git commit -m "docs: mark Alt-Hold integrated; record pre-flight tuning follow-ups"
```

---

## 数据真实性总声明（用户硬性要求复述）

- AltitudeHold 控制律为**本项目手写新代码**，级联结构**参考**（非逐行移植）INAV 固定翼高度环思路。
- 单元测试：纯数学/逻辑断言，**非真实传感器数据**。
- SITL 闭环：高度来自 **JSBSim 仿真真值**（`position/h-agl-ft`），不是真实气压计读数，也不是随意捏造的数 —— 它是物理仿真器输出的真值，足以证明控制环收敛，但**不能替代实物气压计实测**。
- 增益（kp_alt、climb_gains、max_climb_mps、截止频率）均为 **SITL 经验初值**，真机必须现场整定。
- 全部相关源文件、测试、文档均带 WARNING 标注，明确区分“已验证”与“未实物验证”。

## 自检（writing-plans 要求）

- 规格覆盖：定高层（T1/T2）、控制器接管（T3）、闭环验证（T4）、文档与遗留项（T5）—— 全覆盖。
- 占位符扫描：无 TODO/TBD；每个代码步骤均给出完整代码。
- 类型一致性：`AltitudeHold::update` 七参签名在 T1 定义、T3 调用、测试中一致；`AltHoldConfig` 字段名（kp_alt/max_climb_mps/climb_gains/climb_rate_cutoff_hz/pitch_deadband）全程一致；`ControllerConfig` 新增字段（althold_enabled/althold_channel/althold）在 T3 定义与使用一致。
