# Mahony 陀螺零偏估计（开 ki + anti-windup 护栏）实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 给 Mahony AHRS 的陀螺零偏积分项（ifb_）补上 anti-windup 硬限幅 + setBiasLimit setter，使已存在但被 two_ki_=0 关闭的零偏估计能力可安全启用；ki 默认保持关。

**Architecture:** 单文件算法改动 `core/ahrs/ahrs_mahony.{h,cpp}`：加 `ifb_limit_` 成员 + `clampf` helper + `setBiasLimit` setter，在 6DOF/9DOF 两处积分后夹 ifb_。ki 默认仍 0（setKi 已存在，setter 可调）。ControllerConfig 仅注释预留位（不占空间/不升 NVS version）。3 个新 AHRS 单测；现有 6 个 + SITL 基线零变动。

**Tech Stack:** C++17、Unity（PC 单测）、CMake+Ninja。纯 core 算法，无硬件依赖。

**设计文档：** `docs/superpowers/specs/2026-06-01-mahony-gyro-bias-design.md`

---

## 关键设计决策（实现前必读）

1. **零偏估计代码已存在**：`ifb_x/y/z` 积分反馈在 `ahrs_mahony.cpp:53-60`(6DOF) 和 `:110-117`(9DOF) 已完整实现，`two_ki_>0` 时积分、否则 else 恒清零。`setKi(float two_ki)` setter 已存在（`ahrs_mahony.h:11`）。本计划**不重写这些**。
2. **唯一新增逻辑 = ifb_ 硬限幅**：当前 ifb_ 无限幅，系统性偏差会让零偏估计无界爬升。补对称硬限幅 `[-ifb_limit_, +ifb_limit_]`。
3. **ki 默认保持 0**：不改 `two_ki_` 默认值。现有 6 个 AHRS 测试（全部默认 ki=0）+ SITL 基线（7.0/1.3、47.9/9.5）零变动。
4. **单位**：`ifb_` 与 `gx/gy/gz` 同单位 rad/s。`ifb_limit_` 默认 `0.1f` rad/s（≈5.7°/s 零偏上限）。
5. **限幅只夹 ifb_**，不夹最终 gx/gy/gz（kp 比例校正不受影响）。
6. **TDD + 频繁提交**：先写测试看失败，再实现。

## 构建命令
```bash
cd /c/Repository/WeekendPilot
cmake --preset dev          # 配置（改 CMakeLists 后；本计划不改 CMakeLists）
cmake --build build
ctest --test-dir build --output-on-failure
```

## 文件结构
**修改：**
- `core/ahrs/ahrs_mahony.h` — 加 `setBiasLimit` 声明 + `ifb_limit_` 成员
- `core/ahrs/ahrs_mahony.cpp` — 加 `clampf` helper + 两处积分后限幅
- `core/modes/stab_mode.h` — 注释预留位（不加字段）
- `test/test_ahrs/test_ahrs.cpp` — 追加 3 测试 + 注册（现有 6 个不动）

---

## Task 1: ifb_ 硬限幅 + setBiasLimit（核心改动，TDD）

**Files:**
- Modify: `core/ahrs/ahrs_mahony.h`
- Modify: `core/ahrs/ahrs_mahony.cpp`
- Modify: `test/test_ahrs/test_ahrs.cpp`

- [ ] **Step 1: 头文件加 setter 声明 + 成员**

`core/ahrs/ahrs_mahony.h`，在 `void setKi(float two_ki) { two_ki_ = two_ki; }`（line 11）之后加：

```cpp
    // 陀螺零偏积分(ifb_)对称硬限幅(rad/s)。防系统性偏差致零偏估计无界爬升。
    // lim<0 忽略(保持当前值)；lim==0 合法(等于把零偏估计夹死为 0)。
    void setBiasLimit(float lim) { if (lim >= 0.0f) ifb_limit_ = lim; }
```

在 private 成员区，`float two_ki_ = 2.0f * 0.0f;`（line 29）之后加：

```cpp
    float ifb_limit_ = 0.1f;    // 零偏积分硬限幅(rad/s)，≈5.7°/s。默认保守。
```

- [ ] **Step 2: 写失败测试**（见下方代码块 → 追加到 `test/test_ahrs/test_ahrs.cpp`）

- [ ] **Step 3: 跑测试确认失败**

Run: `cmake --build build 2>&1 | tail -10`
Expected: 编译失败（`setBiasLimit` 在 Step 2 测试里被调用，但若先加了 Step 1 的声明则编译过、测试逻辑失败）。若 Step 1 已应用，预期是 `test_bias_limit_clamps` 断言失败（ifb_ 未夹，发散）——确认限幅逻辑尚未加。

- [ ] **Step 4: 实现限幅**

`core/ahrs/ahrs_mahony.cpp`，在 `reset()` 之前（约 line 10，namespace 内顶部）加 helper：

```cpp
static inline float clampf(float v, float lim) {
    return v > lim ? lim : (v < -lim ? -lim : v);
}
```

在 6DOF 积分块（当前 `ahrs_mahony.cpp:54-56`）：

```cpp
            ifb_x_ += two_ki_ * halfex * dt;
            ifb_y_ += two_ki_ * halfey * dt;
            ifb_z_ += two_ki_ * halfez * dt;
```

改为（积分后立即夹）：

```cpp
            ifb_x_ = clampf(ifb_x_ + two_ki_ * halfex * dt, ifb_limit_);
            ifb_y_ = clampf(ifb_y_ + two_ki_ * halfey * dt, ifb_limit_);
            ifb_z_ = clampf(ifb_z_ + two_ki_ * halfez * dt, ifb_limit_);
```

在 9DOF 积分块（当前 `ahrs_mahony.cpp:111-113`）做**完全相同**的替换：

```cpp
            ifb_x_ = clampf(ifb_x_ + two_ki_ * halfex * dt, ifb_limit_);
            ifb_y_ = clampf(ifb_y_ + two_ki_ * halfey * dt, ifb_limit_);
            ifb_z_ = clampf(ifb_z_ + two_ki_ * halfez * dt, ifb_limit_);
```

- [ ] **Step 5: 跑测试确认通过**

Run: `cmake --build build && ctest --test-dir build -R test_ahrs --output-on-failure`
Expected: 9 个测试 PASS（原 6 + 新 3）。

- [ ] **Step 6: 全量回归**

Run: `ctest --test-dir build --output-on-failure 2>&1 | tail -5`
Expected: 全过（26 套）。

- [ ] **Step 7: Commit**

```bash
git add core/ahrs/ahrs_mahony.h core/ahrs/ahrs_mahony.cpp test/test_ahrs/test_ahrs.cpp
git commit -m "feat(ahrs): Mahony 零偏积分 ifb_ 加 anti-windup 硬限幅 + setBiasLimit + 3 单测"
```

---
**Step 2 测试代码** — 追加到 `test/test_ahrs/test_ahrs.cpp`，放在现有最后一个测试函数（`test_accel_gate_skips_correction_when_out_of_range`）之后、`int main()` 之前：

```cpp
// ki=0(默认)：喂带恒定零偏的陀螺 + 水平加速度，姿态仍由 kp 主导收敛到水平。
// 验默认行为不回归（ki 关时零偏靠 kp 比例项压住，稳态有小残差但有界）。
void test_ki_zero_keeps_level_bounded() {
    wp::AhrsMahony ahrs;                 // 默认 two_ki_=0
    wp::ImuSample s = level_imu();
    s.gyro_x = 2.0f;                     // +2 deg/s 恒定零偏
    for (int i = 0; i < 5000; ++i) ahrs.update(s, 0.001f);  // 6DOF
    wp::Attitude a = ahrs.attitude();
    // ki 关：kp 把 roll 压在小角度（默认 two_kp=1，2°/s 零偏下稳态残差小且有界）
    TEST_ASSERT_TRUE(a.roll_deg > -10.0f && a.roll_deg < 10.0f);
}

// ki>0：注入恒定陀螺零偏，零偏被 ifb_ 估出对消，姿态收敛到水平（优于 ki=0 的残差）。
// 这是 ki 的核心价值验证。
void test_ki_estimates_constant_bias() {
    wp::AhrsMahony ahrs;
    ahrs.setKi(2.0f * 0.1f);             // 开零偏估计
    wp::ImuSample s = level_imu();
    s.gyro_x = 2.0f; s.gyro_y = 1.0f;    // 恒定零偏 roll+2/pitch+1 deg/s
    for (int i = 0; i < 20000; ++i) ahrs.update(s, 0.001f);  // 20s 足够收敛
    wp::Attitude a = ahrs.attitude();
    // 零偏被对消后姿态回到水平
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, a.roll_deg);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, a.pitch_deg);
}

// 限幅：ki 开 + 极小 bias limit + 持续误差，姿态保持有界不发散（限幅生效）。
// setBiasLimit 把 ifb_ 夹在很小范围，使其无法完全对消大零偏，但系统仍稳定有界。
void test_bias_limit_keeps_bounded() {
    wp::AhrsMahony ahrs;
    ahrs.setKi(2.0f * 0.5f);             // 较大 ki，若无限幅 ifb_ 会冲很高
    ahrs.setBiasLimit(0.02f);            // 很小限幅(rad/s)
    wp::ImuSample s = level_imu();
    s.gyro_x = 10.0f;                    // 大恒定零偏 +10 deg/s
    for (int i = 0; i < 20000; ++i) ahrs.update(s, 0.001f);
    wp::Attitude a = ahrs.attitude();
    // 限幅下 ifb_ 不能完全对消 10°/s，但姿态必须有界（不发散到 ±90）
    TEST_ASSERT_TRUE(a.roll_deg > -45.0f && a.roll_deg < 45.0f);
    // 且不是 NaN
    TEST_ASSERT_TRUE(a.roll_deg == a.roll_deg);
}
```

并把 `main()` 的 `UNITY_BEGIN();` 之后、`return UNITY_END();` 之前，在现有 6 个 RUN_TEST 后追加：

```cpp
    RUN_TEST(test_ki_zero_keeps_level_bounded);
    RUN_TEST(test_ki_estimates_constant_bias);
    RUN_TEST(test_bias_limit_keeps_bounded);
```

> 注：`test_ki_estimates_constant_bias` 的收敛拍数（20000@1ms=20s）和 ki 值（2*0.1）是经验设定。若实现正确但该测试不收敛到 1° 内，**先放宽到 2° 或增加拍数确认是收敛速度问题而非逻辑错**，不要改限幅逻辑。若 ki=0 的 `test_ki_zero_keeps_level_bounded` 残差超 ±10°，说明 kp 默认值理解有误——停下报告，不要为凑测试改 kp。

---

## Task 2: ControllerConfig 注释预留位

不加字段、不占空间、不升 NVS version。纯文档性预留，标记将来实物调参提到 config 时的落点。

**Files:**
- Modify: `core/modes/stab_mode.h`

- [ ] **Step 1: 读 stab_mode.h 找 StabConfig**

读 `core/modes/stab_mode.h`，定位 `StabConfig` 结构体（持有 PID 增益等可调参数，被 ControllerConfig 持有）。

- [ ] **Step 2: 在 StabConfig 末尾字段后加注释（不加字段）**

在 StabConfig 最后一个成员之后、结构体右括号之前，加一行注释：

```cpp
    // 预留：ahrs_ki / ahrs_bias_limit —— 将来实物调参提到 config 时在此加字段。
    // 当前经 AhrsMahony::setKi/setBiasLimit + 代码默认配置；注意 sizeof(ControllerConfig) 余量(现 226/255)。
```

- [ ] **Step 3: 构建 + 回归验证（确认 sizeof 不变）**

Run: `cmake --build build && ctest --test-dir build --output-on-failure 2>&1 | tail -5`
Expected: 全过（26 套）。test_config_store 的 `static_assert(sizeof(ControllerConfig) <= 255)` 仍成立（纯注释，sizeof 不变）。

- [ ] **Step 4: Commit**

```bash
git add core/modes/stab_mode.h
git commit -m "docs(config): StabConfig 加 ahrs_ki/bias_limit 预留注释（不占空间）"
```

---

## Task 3: SITL 基线回归确认 + 文档台账

**Files:**
- Modify: `docs/superpowers/specs/2026-06-01-mahony-gyro-bias-design.md`
- Modify: 记忆 `weekendpilot-dev-progress.md`（Write 工具）

- [ ] **Step 1: SITL 基线回归（数值零变动）**

ki 默认关，SITL C API 走默认配置。若 SITL 环境可用（见 [[weekendpilot-build-convention]] 的 ctypes/DLL 注意），跑现有 SITL 检查脚本确认基线不变（扰动改出 7.0°→1.3°、协调转弯 47.9°/9.5°）。

Run（若 SITL 可跑）：项目现有的 SITL check 脚本（如 `check_alt_hold.py` 同目录下的姿态/转弯检查）。
Expected: 数值与基线一致（ki 默认关，估计层行为未变）。
若 SITL 环境不可用：跳过并在报告中说明"仅 PC 单测 + 推理确认 ki 默认关不改变估计输出"。

- [ ] **Step 2: 设计文档状态改"已实现"**

把 spec 头部 `- 状态：设计待评审` 改为 `- 状态：已实现（PC 单测 9/26 套全绿 + SITL 基线零回归）`。

- [ ] **Step 3: 更新进度台账记忆**

在 `weekendpilot-dev-progress.md` 已完成区追加一段（仿现有风格）：Mahony 零偏估计 anti-windup 落地——ifb_ 加对称硬限幅 + setBiasLimit，ki 默认仍关(setter 可调)，源于 EKF/UKF/ADRC 评估(当前刻意 Mahony+PID，EKF/UKF 无 GPS 无导航无收益，ADRC 待单轴实验 A)，3 新单测，SITL 基线零回归，⚠️ 真实零偏纠正只能实物验。

- [ ] **Step 4: Commit**

```bash
git add docs/superpowers/specs/2026-06-01-mahony-gyro-bias-design.md
git commit -m "docs: Mahony 零偏估计设计文档标记已实现"
```

---

## 完成标准（Definition of Done）
- [ ] PC ctest 全绿：test_ahrs 6→9 例，总 26 套不变（test_ahrs 内部 +3）。
- [ ] ki 默认仍 0：现有 6 个 AHRS 测试与 SITL 基线零变动。
- [ ] ifb_ 限幅生效：大零偏下姿态有界不发散。
- [ ] ControllerConfig sizeof 不变（纯注释预留）。

## 已知限制（⚠️）
- ki 默认关，真实抗零偏价值须实物调 ki 后观察，SITL 合成 IMU 无真实零偏不可验。
- ifb_limit_ 默认 0.1 rad/s 为经验值，实物 IMU 零偏量级确认后可调。
- 限幅为对称硬限幅非泄漏型；缓变零偏跟踪需求待实物评估（YAGNI，本轮不做）。
