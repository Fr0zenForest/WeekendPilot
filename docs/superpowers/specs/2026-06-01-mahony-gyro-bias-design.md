# Mahony 陀螺零偏估计（开 ki + anti-windup 护栏）— 设计文档

- 日期：2026-06-01
- 状态：设计待评审
- 关联：[[weekendpilot-ahrs-observability]]、[[weekendpilot-dev-progress]]、主设计 `docs/superpowers/specs/2026-05-30-weekendpilot-design.md`
- 定位：估计层小改（B）。源于评估"我们飞控里有没有 EKF/UKF/ADRC"——结论是没有、当前刻意用 Mahony+PID；EKF/UKF 现阶段无可兑现收益（无 GPS、不做导航），UKF 纯刷参数表，ADRC 唯一值得做但作为单轴实验（A，另起一轮）。本轮 B 是 0 成本摘果子：把 Mahony 已写好但被关掉的零偏估计能力（ki）开成可调，并补上它缺的 anti-windup 护栏。

## 0. 背景与动机

Mahony 互补滤波的积分反馈项（`ifb_x/y/z`）能估计并对消陀螺零偏（抗温漂/安装零偏），相当于"EKF 零偏状态"的极简版。当前代码里这部分**已完整实现**，但 `two_ki_` 默认 0 使其走 else 分支恒清零、从不积分（`ahrs_mahony.cpp:53-60, 110-117`）。`setKi` setter **也已存在**（`ahrs_mahony.h:11`）。

唯一缺的、也是唯一的真问题：**`ifb_` 积分没有任何限幅（anti-windup）**。一旦传感器有系统性偏差，零偏估计会无界缓慢爬升。这是 ki 经典坑，必须连 ki 一起补护栏。

EKF/UKF/ADRC 的完整评估见对话记录；本 spec 只做 B。

## 1. 确认的设计决策

| # | 决策点 | 选定 |
|---|---|---|
| D1 | ki 默认值 | 默认仍 0（关）。能力写好可调，实物标定期用 setKi 调开。现有 6 个 AHRS 测试 + SITL 基线零变动（"全写好先不使能"）。 |
| D2 | anti-windup 护栏 | 对 `ifb_x/y/z` 各自加对称硬限幅（INAV/Betaflight 做法）。 |
| D3 | 配置粒度 | ki + 限幅做成 AhrsMahony 的 setter（setKi 已存在，新增 setBiasLimit）+ 代码常量默认。不进 ControllerConfig，不动 NVS。ControllerConfig 仅加注释预留位（不占空间、不升 version）。 |

约束记录：`sizeof(ControllerConfig)=226`，距 `static_assert(<=255)` 仅余 29 字节——D3 选 setter 正是为避免吃这个余量 + 升 NVS version。

## 2. 架构与改动

全部改动在单文件 `core/ahrs/ahrs_mahony.{h,cpp}`，不碰 controller/mixer/config 序列化/main.cpp/HAL。

### 2.1 加零偏硬限幅
- 新增成员 `float ifb_limit_`，默认保守值。默认取 `0.1f`（rad/s，因 `ifb_` 与 `gx/gy/gz` 同单位 rad/s；0.1 rad/s ≈ 5.7°/s 的零偏上限，足够覆盖常见 IMU 零偏，又防失控爬升）。
- 新增 setter `void setBiasLimit(float lim)`。lim 语义见 §3（负值忽略、0 合法）。
- 在两处积分更新后（6DOF `ahrs_mahony.cpp:54-56`、9DOF `:111-113`），对 `ifb_x/y/z` 各自夹到 `[-ifb_limit_, +ifb_limit_]`。提取一个文件内 `static float clampf(float v, float lim)` helper，两处复用，避免重复。

### 2.2 ki 默认保持 0
- 不改 `two_ki_` 默认值（仍 `2.0f * 0.0f`）。现有行为/测试/基线零变动。

### 2.3 ControllerConfig 预留位（不占空间）
- 在 `StabConfig`（`core/modes/stab_mode.h` 内，已被 ControllerConfig 持有）加一行注释占位，例如：
  `// 预留：ahrs_ki / ahrs_bias_limit —— 将来实物调参提到 config 时填（注意 sizeof(ControllerConfig) 余量，现 226/255）。`
- 不加实际字段。sizeof 不变，kConfigVersion 不升。

## 3. 错误处理 / 边界
- `setBiasLimit(lim)`：lim 应为正。约定调用方传正值；为稳健，实现里若 `lim < 0` 则忽略（保持当前 `ifb_limit_`）。lim==0 等价"零偏估计被夹死为 0"，是合法的（等于软关闭零偏），不特殊处理。
- ki=0 时 else 分支已把 `ifb_` 清零，限幅不参与（无副作用）。
- 限幅只夹 `ifb_`，不夹最终 `gx/gy/gz`（kp 比例项不受影响，姿态校正能力不变）。

## 4. 测试（`test/test_ahrs/test_ahrs.cpp` 追加，现有 6 个不动）

- `test_ki_zero_keeps_ifb_zero`：默认 ki=0，喂带恒定零偏的陀螺（如 gyro_x=+2°/s）+ 水平加速度，跑若干拍。断言行为与现状一致（姿态由 kp 主导收敛；ki 关时 ifb_ 不参与）。验 D1/else 分支不回归。
- `test_ki_estimates_constant_bias`：`setKi(2.0f*0.1f)` + 水平 + 各轴注入恒定零偏（如 +2°/s），跑足够长（如 20000 拍 @1ms）。断言姿态仍收敛到水平（`TEST_ASSERT_FLOAT_WITHIN(1.0, 0, roll/pitch)`）——零偏被 ifb_ 估出对消。这是 ki 的核心价值验证。
- `test_bias_limit_clamps`：`setKi` 开 + `setBiasLimit(small)`（如 0.02f）+ 持续大误差（恒定大零偏，使 ifb_ 想超过限幅），跑足够长。断言估计不发散、姿态有界（限幅生效，系统不失稳）。
- 测试用 setKi/setBiasLimit 在用例内开启，不动全局默认。

## 5. SITL
- ki 默认关 → SITL C API 走默认 → 基线（扰动改出 7.0°→1.3°、协调转弯 47.9°/9.5°）**零回归**，只验"不破坏"。
- ⚠️ **真实零偏纠正效果只能实物验**：SITL 合成 IMU 无真实零偏；`test_ki_estimates_constant_bias` 用人工注入的恒定零偏验算法正确性，不代表实物表现。实物标定期再调 ki/limit 并观察。

## 6. 实施顺序
1. 加 `ifb_limit_` 成员 + `clampf` helper + `setBiasLimit` setter（头+实现）。
2. 两处积分后加限幅。
3. 追加 3 个单测（先写失败/行为锚点，再确认通过）。
4. ControllerConfig/StabConfig 注释预留位。
5. SITL 基线回归确认（数值不变）。
6. 文档 + 台账更新。

## 7. 已知限制（⚠️）
- ki 默认关，本轮不在飞行中启用；真实抗零偏价值须实物调 ki 后观察，SITL 不可验。
- `ifb_limit_` 默认值（0.1 rad/s）为经验值，实物 IMU 零偏量级确认后可调。
- 不做温度补偿/在线零偏标定流程（那是更大的标定 spec，本轮只让 Mahony 的运行时零偏估计可用+安全）。
- 限幅为对称硬限幅，非泄漏型；若实物发现零偏缓变需跟踪，再评估加泄漏（YAGNI，本轮不做）。

## 附录：与现有代码衔接
- `ahrs_mahony.h:11` `setKi` → 已存在，复用。
- `ahrs_mahony.h:26-31` 成员区 → 加 `ifb_limit_`。
- `ahrs_mahony.cpp:53-60` / `:110-117` 积分块 → 后接限幅。
- `core/modes/stab_mode.h` StabConfig → 注释预留位。
- `test/test_ahrs/test_ahrs.cpp` → 追加 3 测试 + 注册。
