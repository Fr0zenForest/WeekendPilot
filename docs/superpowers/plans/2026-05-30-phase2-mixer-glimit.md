# 阶段 2：Mixer + 外设直通 + G-limit 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把写死在 `controller.cpp` 里的"手动+增稳"混控抽成真正的 Mixer（预设→规则表展开→统一引擎），支持 Standard/Flaperon/VTail/Elevon 四种布局；加 G-limit 过载限制（软限上方线性衰减拉杆）；加外设通道直通。做完能接舵机做 HIL 地面验证。

**Architecture:** core 新增两个无状态纯函数模块：`core/mixer/`（轴需求 src[5] → servo[8]）和 `core/safety/glimit.h`（法向 G 衰减 pitch 需求）。`controller.cpp` 的数据流改为 `demand → G-limit → mixer → 外设直通覆盖`。C API 与对外 `ServoCommand{servo[8]}` 不变。

**Tech Stack:** C++17 | CMake（`cmake --preset dev`，Ninja+GCC）| Unity（单测，`wp_add_test` 自动 GLOB core 的 .cpp）| 参考源码：INAV `src/main/flight/servos.{h,c}`（mixer 规则表+累加引擎）、ArduPlane `ArduPlane/Attitude.cpp`（G-limit 思路）。

**设计依据：** `docs/superpowers/specs/2026-05-30-weekendpilot-design.md` 附录 D。

---

## 文件结构

| 文件 | 职责 |
|---|---|
| `core/mixer/mixer.h`（新建） | `MixSource`/`Airframe` 枚举、`MixRule`/`MixerConfig` 结构、`Mixer` 类声明（`setAirframe`/`mix`） |
| `core/mixer/mixer.cpp`（新建） | `expand(Airframe)` 四种预设展开；`mix(src[5]) → servo[8]` 累加+clamp+油门单边 |
| `core/safety/glimit.h`（新建） | `GLimitConfig` 结构 + `applyGLimit(pitch_demand, accel_z, cfg)` 内联纯函数 |
| `core/types.h`（改） | 在 `ControllerConfig` 用不到这里；仅可能加 mixer 相关无须改。**本计划不改 types.h** |
| `core/controller.h`（改） | `ControllerConfig` 加 `Airframe airframe`、`GLimitConfig glimit`、外设表、`flap_channel`；`Controller` 持有 `Mixer mixer_` |
| `core/controller.cpp`（改） | update 数据流改为 demand→G-limit→mixer→外设直通 |
| `test/test_mixer/test_mixer.cpp`（新建） | Standard 复刻、各预设展开与符号、饱和 clamp、油门单边 |
| `test/test_glimit/test_glimit.cpp`（新建） | 软限下无变化、软硬限间线性、硬限清零、推杆不限 |
| `test/CMakeLists.txt`（改） | 加 `wp_add_test(test_mixer)` `wp_add_test(test_glimit)` |

**关键约定（与现有代码对齐）：**
- servo 物理口：`0=副翼/左混` `1=升降/右混` `2=油门` `3=方向`，与 `controller.cpp` 现有 `mix(0,roll)`/`mix(1,pitch)`/`mix(3,yaw)`、servo[2] 油门一致。
- 归一化：轴需求 src 用 `[-1,1]`（与 `channelToNorm` 一致）；油门 src 用 `[0,1]`。
- 输出：非油门口 `normToServoUs(clamp(acc,-1,1))`（中位 1500）；油门口 `1000+1000*clamp(acc,0,1)`。
- `kNumServos=8` 不变。

---

## Task 1: Mixer 骨架 + Standard 预设（复刻现有行为）

**Files:**
- Create: `core/mixer/mixer.h`
- Create: `core/mixer/mixer.cpp`
- Create: `test/test_mixer/test_mixer.cpp`
- Modify: `test/CMakeLists.txt`

- [ ] **Step 1: 写 mixer.h**

```cpp
#pragma once
#include <cstdint>
#include "types.h"

namespace wp {

enum class MixSource : uint8_t { Roll = 0, Pitch = 1, Yaw = 2, Throttle = 3, Flap = 4, Count = 5 };
enum class Airframe  : uint8_t { Standard = 0, Flaperon = 1, VTail = 2, Elevon = 3 };

struct MixRule {
    uint8_t   out;       // 输出 servo 索引 [0,kNumServos)
    MixSource src;       // 取哪一路轴需求
    float     weight;    // 权重（含符号）
};

constexpr int kMaxMixRules = 24;

class Mixer {
public:
    Mixer() { setAirframe(Airframe::Standard); }
    void setAirframe(Airframe af);

    // src 下标用 MixSource：roll/pitch/yaw ∈ [-1,1]，throttle/flap ∈ [0,1]。
    // 输出写入 servo[kNumServos]（us 值）。
    void mix(const float src[static_cast<int>(MixSource::Count)],
             uint16_t servo[kNumServos]) const;

private:
    MixRule rules_[kMaxMixRules];
    uint8_t num_rules_ = 0;
    bool    is_throttle_[kNumServos] = {};
    uint8_t num_outputs_ = kNumServos;
};

}  // namespace wp
```

- [ ] **Step 2: 写 test_mixer.cpp 的第一个失败测试（Standard 复刻）**

```cpp
#include <unity.h>
#include "mixer/mixer.h"

void setUp() {}
void tearDown() {}

static void zero_src(float s[5]) { for (int i = 0; i < 5; ++i) s[i] = 0.0f; }

void test_standard_centered_outputs_1500() {
    wp::Mixer m;  // 默认 Standard
    float src[5]; zero_src(src);
    uint16_t servo[wp::kNumServos];
    m.mix(src, servo);
    TEST_ASSERT_EQUAL_UINT16(1500, servo[0]);  // 副翼
    TEST_ASSERT_EQUAL_UINT16(1500, servo[1]);  // 升降
    TEST_ASSERT_EQUAL_UINT16(1000, servo[2]);  // 油门 src=0 -> 1000
    TEST_ASSERT_EQUAL_UINT16(1500, servo[3]);  // 方向
}

void test_standard_roll_maps_to_aileron() {
    wp::Mixer m;
    float src[5]; zero_src(src);
    src[(int)wp::MixSource::Roll] = 0.5f;       // 半舵右
    uint16_t servo[wp::kNumServos];
    m.mix(src, servo);
    TEST_ASSERT_EQUAL_UINT16(1750, servo[0]);   // 1500 + 0.5*500
    TEST_ASSERT_EQUAL_UINT16(1500, servo[1]);
    TEST_ASSERT_EQUAL_UINT16(1500, servo[3]);
}

void test_standard_throttle_full() {
    wp::Mixer m;
    float src[5]; zero_src(src);
    src[(int)wp::MixSource::Throttle] = 1.0f;
    uint16_t servo[wp::kNumServos];
    m.mix(src, servo);
    TEST_ASSERT_EQUAL_UINT16(2000, servo[2]);   // 1000 + 1.0*1000
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_standard_centered_outputs_1500);
    RUN_TEST(test_standard_roll_maps_to_aileron);
    RUN_TEST(test_standard_throttle_full);
    return UNITY_END();
}
```

- [ ] **Step 3: 注册测试，运行确认编译失败**

把这行加进 `test/CMakeLists.txt`（在 `wp_add_test(test_mode_controller)` 之后）：

```cmake
wp_add_test(test_mixer)
```

Run: `cmake --preset dev && cmake --build build`
Expected: 链接失败 / `undefined reference to wp::Mixer::setAirframe` 和 `wp::Mixer::mix`（mixer.cpp 还没实现）。

- [ ] **Step 4: 写 mixer.cpp 最小实现（Standard + 引擎）**

```cpp
#include "mixer/mixer.h"
#include "modes/stab_mode.h"   // normToServoUs

namespace wp {

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

void Mixer::setAirframe(Airframe af) {
    for (int i = 0; i < kNumServos; ++i) is_throttle_[i] = false;
    num_rules_ = 0;
    num_outputs_ = kNumServos;
    auto add = [&](uint8_t out, MixSource src, float w) {
        rules_[num_rules_++] = MixRule{out, src, w};
    };
    switch (af) {
        case Airframe::Standard:
        default:
            add(0, MixSource::Roll,     +1.0f);
            add(1, MixSource::Pitch,    +1.0f);
            add(2, MixSource::Throttle, +1.0f); is_throttle_[2] = true;
            add(3, MixSource::Yaw,      +1.0f);
            break;
    }
}

void Mixer::mix(const float src[static_cast<int>(MixSource::Count)],
                uint16_t servo[kNumServos]) const {
    float acc[kNumServos] = {0};
    for (int i = 0; i < num_rules_; ++i) {
        const MixRule& r = rules_[i];
        acc[r.out] += r.weight * src[static_cast<int>(r.src)];
    }
    for (int i = 0; i < kNumServos; ++i) {
        if (is_throttle_[i]) {
            servo[i] = static_cast<uint16_t>(1000.0f + 1000.0f * clampf(acc[i], 0.0f, 1.0f));
        } else {
            servo[i] = normToServoUs(clampf(acc[i], -1.0f, 1.0f));
        }
    }
}

}  // namespace wp
```

- [ ] **Step 5: 运行测试确认通过**

Run: `cmake --build build && ctest --test-dir build -R test_mixer --output-on-failure`
Expected: PASS（3 个测试全过）。

- [ ] **Step 6: Commit**

```bash
git add core/mixer/ test/test_mixer/ test/CMakeLists.txt
git commit -m "feat(mixer): rule-table engine + Standard preset"
```

---

## Task 2: VTail / Elevon / Flaperon 预设

**Files:**
- Modify: `core/mixer/mixer.cpp`（在 switch 里加三个 case）
- Modify: `test/test_mixer/test_mixer.cpp`（加展开与符号测试）

- [ ] **Step 1: 加失败测试（V尾 / elevon / flaperon）**

在 `test_mixer.cpp` 的 main 之前加：

```cpp
#include "mixer/mixer.h"   // 已 include，无需重复

void test_vtail_pitch_both_tails_same_dir() {
    wp::Mixer m; m.setAirframe(wp::Airframe::VTail);
    float src[5]; zero_src(src);
    src[(int)wp::MixSource::Pitch] = 0.4f;     // 拉升降
    uint16_t servo[wp::kNumServos];
    m.mix(src, servo);
    // 两片 V 尾 (servo1, servo3) 同向偏转 = 升降
    TEST_ASSERT_EQUAL_UINT16(1700, servo[1]);  // 1500 + 0.4*500
    TEST_ASSERT_EQUAL_UINT16(1700, servo[3]);
}

void test_vtail_yaw_tails_opposite() {
    wp::Mixer m; m.setAirframe(wp::Airframe::VTail);
    float src[5]; zero_src(src);
    src[(int)wp::MixSource::Yaw] = 0.4f;       // 方向
    uint16_t servo[wp::kNumServos];
    m.mix(src, servo);
    // 两片 V 尾差动 = 方向：一片 +，一片 -
    TEST_ASSERT_EQUAL_UINT16(1700, servo[1]);  // +0.4
    TEST_ASSERT_EQUAL_UINT16(1300, servo[3]);  // -0.4
}

void test_elevon_roll_opposite_pitch_same() {
    wp::Mixer m; m.setAirframe(wp::Airframe::Elevon);
    float src[5]; zero_src(src);
    // 纯 roll：两片副翼差动
    src[(int)wp::MixSource::Roll] = 0.4f;
    uint16_t servo[wp::kNumServos];
    m.mix(src, servo);
    TEST_ASSERT_EQUAL_UINT16(1700, servo[0]);  // +0.4 roll
    TEST_ASSERT_EQUAL_UINT16(1300, servo[1]);  // -0.4 roll
    // 纯 pitch：两片同向
    zero_src(src);
    src[(int)wp::MixSource::Pitch] = 0.4f;
    m.mix(src, servo);
    TEST_ASSERT_EQUAL_UINT16(1700, servo[0]);  // +0.4 pitch
    TEST_ASSERT_EQUAL_UINT16(1700, servo[1]);  // +0.4 pitch
}

void test_flaperon_flap_both_ailerons_same() {
    wp::Mixer m; m.setAirframe(wp::Airframe::Flaperon);
    float src[5]; zero_src(src);
    src[(int)wp::MixSource::Flap] = 0.4f;      // 放襟翼
    uint16_t servo[wp::kNumServos];
    m.mix(src, servo);
    // 左右副翼 (servo0, servo4) 同向下垂当襟翼。
    // servo0: +0.4 flap ; servo4: roll(-1)*0 + flap(+1)*0.4 = +0.4
    TEST_ASSERT_EQUAL_UINT16(1700, servo[0]);
    TEST_ASSERT_EQUAL_UINT16(1700, servo[4]);
}

void test_flaperon_roll_ailerons_differential() {
    wp::Mixer m; m.setAirframe(wp::Airframe::Flaperon);
    float src[5]; zero_src(src);
    src[(int)wp::MixSource::Roll] = 0.4f;
    uint16_t servo[wp::kNumServos];
    m.mix(src, servo);
    // servo0: roll(+1)*0.4 = +0.4 ; servo4: roll(-1)*0.4 = -0.4 -> 差动滚转
    TEST_ASSERT_EQUAL_UINT16(1700, servo[0]);
    TEST_ASSERT_EQUAL_UINT16(1300, servo[4]);
}
```

并在 main 里注册：

```cpp
    RUN_TEST(test_vtail_pitch_both_tails_same_dir);
    RUN_TEST(test_vtail_yaw_tails_opposite);
    RUN_TEST(test_elevon_roll_opposite_pitch_same);
    RUN_TEST(test_flaperon_flap_both_ailerons_same);
    RUN_TEST(test_flaperon_roll_ailerons_differential);
```

- [ ] **Step 2: 运行确认失败**

Run: `cmake --build build && ctest --test-dir build -R test_mixer --output-on-failure`
Expected: FAIL（V尾/elevon/flaperon 还走 default=Standard，servo3 在 vtail_pitch 测试里会是 1500 而非 1700）。

- [ ] **Step 3: 在 mixer.cpp 的 switch 里加三个 case**

在 `case Airframe::Standard` 之前插入（注意 default 仍落到 Standard）：

```cpp
        case Airframe::Flaperon:
            add(0, MixSource::Roll, +1.0f); add(0, MixSource::Flap, +1.0f);
            add(1, MixSource::Pitch,    +1.0f);
            add(2, MixSource::Throttle, +1.0f); is_throttle_[2] = true;
            add(3, MixSource::Yaw,      +1.0f);
            add(4, MixSource::Roll, -1.0f); add(4, MixSource::Flap, +1.0f);
            break;
        case Airframe::VTail:
            add(0, MixSource::Roll,     +1.0f);
            add(1, MixSource::Pitch, +1.0f); add(1, MixSource::Yaw, +1.0f);
            add(2, MixSource::Throttle, +1.0f); is_throttle_[2] = true;
            add(3, MixSource::Pitch, +1.0f); add(3, MixSource::Yaw, -1.0f);
            break;
        case Airframe::Elevon:
            add(0, MixSource::Roll, +1.0f); add(0, MixSource::Pitch, +1.0f);
            add(1, MixSource::Roll, -1.0f); add(1, MixSource::Pitch, +1.0f);
            add(2, MixSource::Throttle, +1.0f); is_throttle_[2] = true;
            add(3, MixSource::Yaw,      +1.0f);
            break;
```

- [ ] **Step 4: 运行测试确认通过**

Run: `cmake --build build && ctest --test-dir build -R test_mixer --output-on-failure`
Expected: PASS（全部 8 个测试）。

- [ ] **Step 5: Commit**

```bash
git add core/mixer/mixer.cpp test/test_mixer/test_mixer.cpp
git commit -m "feat(mixer): Flaperon/VTail/Elevon presets"
```

---

## Task 3: 饱和 clamp（逐通道）

**Files:**
- Modify: `test/test_mixer/test_mixer.cpp`（加饱和测试）

> 引擎已在 Task 1 实现 `clampf`，本任务用测试锁住饱和行为，确认 elevon 同时 roll+pitch 打满时逐通道 clamp。

- [ ] **Step 1: 加失败/验证测试**

```cpp
void test_elevon_saturation_clamps_per_channel() {
    wp::Mixer m; m.setAirframe(wp::Airframe::Elevon);
    float src[5]; zero_src(src);
    src[(int)wp::MixSource::Roll]  = 0.8f;
    src[(int)wp::MixSource::Pitch] = 0.8f;
    // servo0 = +0.8 +0.8 = 1.6 -> clamp 1.0 -> 2000
    // servo1 = -0.8 +0.8 = 0.0 -> 1500
    uint16_t servo[wp::kNumServos];
    m.mix(src, servo);
    TEST_ASSERT_EQUAL_UINT16(2000, servo[0]);
    TEST_ASSERT_EQUAL_UINT16(1500, servo[1]);
}

void test_throttle_clamps_below_zero() {
    wp::Mixer m;  // Standard
    float src[5]; zero_src(src);
    src[(int)wp::MixSource::Throttle] = -0.5f;   // 异常负值
    uint16_t servo[wp::kNumServos];
    m.mix(src, servo);
    TEST_ASSERT_EQUAL_UINT16(1000, servo[2]);     // clamp 到 0 -> 1000
}
```

注册：

```cpp
    RUN_TEST(test_elevon_saturation_clamps_per_channel);
    RUN_TEST(test_throttle_clamps_below_zero);
```

- [ ] **Step 2: 运行测试**

Run: `cmake --build build && ctest --test-dir build -R test_mixer --output-on-failure`
Expected: PASS（引擎已有 clamp，应直接过；若不过说明 clamp 逻辑有 bug，需修 mixer.cpp）。

- [ ] **Step 3: Commit**

```bash
git add test/test_mixer/test_mixer.cpp
git commit -m "test(mixer): per-channel saturation clamp"
```

---

## Task 4: G-limit 模块

**Files:**
- Create: `core/safety/glimit.h`
- Create: `test/test_glimit/test_glimit.cpp`
- Modify: `test/CMakeLists.txt`

- [ ] **Step 1: 写 glimit.h**

```cpp
#pragma once

namespace wp {

struct GLimitConfig {
    bool  enabled   = true;
    float soft_g    = 6.0f;    // 软限：超过开始衰减拉杆
    float hard_g    = 10.0f;   // 硬限：拉杆贡献清零
    // 拉杆方向约定：正 pitch 需求 = 抬头 = 加载（正法向 G 增大）。
    // 若实测舵面/IMU 装反，把 pitch_loads_positive 置 false。
    bool  pitch_loads_positive = true;
};

// 返回经 G-limit 衰减后的 pitch 需求。
// accel_z：IMU 法向载荷（单位 g，平飞 ~+1，正向抬头加载）。
// 只衰减"加载方向"的拉杆；卸载方向（推杆）不限。
inline float applyGLimit(float pitch_demand, float accel_z, const GLimitConfig& cfg) {
    if (!cfg.enabled) return pitch_demand;
    float g = cfg.pitch_loads_positive ? accel_z : -accel_z;
    bool pulling = cfg.pitch_loads_positive ? (pitch_demand > 0.0f) : (pitch_demand < 0.0f);
    if (g <= cfg.soft_g || !pulling) return pitch_demand;
    float span = cfg.hard_g - cfg.soft_g;
    float k = (span > 0.0f) ? (1.0f - (g - cfg.soft_g) / span) : 0.0f;
    if (k < 0.0f) k = 0.0f;
    if (k > 1.0f) k = 1.0f;
    return pitch_demand * k;
}

}  // namespace wp
```

- [ ] **Step 2: 写 test_glimit.cpp 失败测试**

```cpp
#include <unity.h>
#include "safety/glimit.h"

void setUp() {}
void tearDown() {}

void test_below_soft_limit_unchanged() {
    wp::GLimitConfig cfg;   // soft 6, hard 10
    float out = wp::applyGLimit(0.8f, 3.0f, cfg);   // 3G < 6G
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.8f, out);
}

void test_midway_linear_attenuation() {
    wp::GLimitConfig cfg;
    // 8G：k = 1 - (8-6)/(10-6) = 0.5
    float out = wp::applyGLimit(0.8f, 8.0f, cfg);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.4f, out);
}

void test_at_hard_limit_zeroed() {
    wp::GLimitConfig cfg;
    float out = wp::applyGLimit(0.8f, 10.0f, cfg);   // k=0
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, out);
}

void test_beyond_hard_limit_zeroed() {
    wp::GLimitConfig cfg;
    float out = wp::applyGLimit(0.8f, 15.0f, cfg);   // k clamp 0
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, out);
}

void test_push_stick_not_limited() {
    wp::GLimitConfig cfg;
    // 推杆（负 pitch 需求）即使在高 G 也不限（高 G 来自别处）
    float out = wp::applyGLimit(-0.8f, 9.0f, cfg);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -0.8f, out);
}

void test_disabled_passes_through() {
    wp::GLimitConfig cfg; cfg.enabled = false;
    float out = wp::applyGLimit(0.8f, 12.0f, cfg);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.8f, out);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_below_soft_limit_unchanged);
    RUN_TEST(test_midway_linear_attenuation);
    RUN_TEST(test_at_hard_limit_zeroed);
    RUN_TEST(test_beyond_hard_limit_zeroed);
    RUN_TEST(test_push_stick_not_limited);
    RUN_TEST(test_disabled_passes_through);
    return UNITY_END();
}
```

- [ ] **Step 3: 注册测试，运行确认失败再通过**

`test/CMakeLists.txt` 加：

```cmake
wp_add_test(test_glimit)
```

Run: `cmake --preset dev && cmake --build build && ctest --test-dir build -R test_glimit --output-on-failure`
Expected: glimit.h 是 header-only 纯函数，编译即实现 → 应直接 PASS（6 个测试）。若某条不过，修 glimit.h 公式。

- [ ] **Step 4: Commit**

```bash
git add core/safety/glimit.h test/test_glimit/ test/CMakeLists.txt
git commit -m "feat(safety): G-limit pull-stick attenuation"
```

---

## Task 5: 接线进 controller（demand → G-limit → mixer → 外设直通）

**Files:**
- Modify: `core/controller.h`
- Modify: `core/controller.cpp`
- Modify: `test/test_controller/test_controller.cpp`（加回归 + 新行为测试）

- [ ] **Step 1: 改 controller.h**

把 `controller.h` 替换为（在现有基础上加 mixer/glimit/外设表/flap）：

```cpp
#pragma once
#include "types.h"
#include "ahrs/ahrs_mahony.h"
#include "modes/mode_controller.h"
#include "modes/stab_mode.h"
#include "mixer/mixer.h"
#include "safety/glimit.h"

namespace wp {

constexpr int kMaxPeripherals = 4;

// 外设直通：把某 RC 通道裸 us 直接覆盖到某 servo 输出口（起落架/灯/襟翼开关）。
struct PeripheralMap {
    uint8_t servo_out = 0;
    uint8_t rc_channel = 0;
    bool    enabled = false;
};

struct ControllerConfig {
    bool enabled = true;
    uint8_t mode_channel = 4;      // ch5
    uint8_t gain_channel = 5;      // ch6
    uint8_t throttle_channel = 2;  // ch3
    uint8_t roll_channel = 0;
    uint8_t pitch_channel = 1;
    uint8_t yaw_channel = 3;
    uint8_t flap_channel = 6;      // ch7，无则保持中位（flap 需求 0）
    bool    flap_enabled = false;  // 默认不引入 flap 需求
    float throttle_low_us = 1100.0f;
    Airframe airframe = Airframe::Standard;
    GLimitConfig glimit;
    PeripheralMap peripherals[kMaxPeripherals];
    StabConfig stab;
};

class Controller {
public:
    Controller();
    void setConfig(const ControllerConfig& cfg);
    ServoCommand update(const ControlInput& in);
    Attitude attitude() const { return ahrs_.attitude(); }

private:
    ControllerConfig cfg_;
    AhrsMahony ahrs_;
    ModeController modes_;
    Mixer mixer_;
};

}  // namespace wp
```

- [ ] **Step 2: 改 controller.cpp**

完整替换 `controller.cpp`：

```cpp
#include "controller.h"

namespace wp {

Controller::Controller() {
    modes_.setConfig(cfg_.stab);
    mixer_.setAirframe(cfg_.airframe);
}

void Controller::setConfig(const ControllerConfig& cfg) {
    cfg_ = cfg;
    modes_.setConfig(cfg_.stab);
    mixer_.setAirframe(cfg_.airframe);
}

ServoCommand Controller::update(const ControlInput& in) {
    ServoCommand out{};
    // 默认直通所有通道
    for (int i = 0; i < kNumServos; ++i) out.servo[i] = in.channels[i];

    FlightMode mode = modeFromChannel(in.channels[cfg_.mode_channel]);

    // 直通条件：未启用 / 链路丢失 / IMU 无效 / Off 模式
    if (!cfg_.enabled || !in.link_ok || !in.imu.valid || mode == FlightMode::Off) {
        return out;
    }

    ahrs_.update(in.imu, in.mag, in.dt);

    float roll_cmd  = channelToNorm(in.channels[cfg_.roll_channel]);
    float pitch_cmd = channelToNorm(in.channels[cfg_.pitch_channel]);
    float yaw_cmd   = channelToNorm(in.channels[cfg_.yaw_channel]);
    float gain      = gainFromChannel(in.channels[cfg_.gain_channel]);
    bool throttle_low = in.channels[cfg_.throttle_channel] < cfg_.throttle_low_us;

    StabCorrection corr = modes_.update(mode, roll_cmd, pitch_cmd, yaw_cmd,
                                        ahrs_.attitude(), in.imu, in.dt, throttle_low);

    // 轴需求 = 手动 + gain*PID修正
    float demand[static_cast<int>(MixSource::Count)] = {0};
    demand[static_cast<int>(MixSource::Roll)]  = roll_cmd  + gain * corr.roll;
    demand[static_cast<int>(MixSource::Pitch)] = pitch_cmd + gain * corr.pitch;
    demand[static_cast<int>(MixSource::Yaw)]   = yaw_cmd   + gain * corr.yaw;
    // 油门：归一化 [0,1]（throttle 通道 us -> 0..1）
    demand[static_cast<int>(MixSource::Throttle)] =
        gainFromChannel(in.channels[cfg_.throttle_channel]);
    // 襟翼需求（可选）
    demand[static_cast<int>(MixSource::Flap)] = cfg_.flap_enabled
        ? gainFromChannel(in.channels[cfg_.flap_channel]) : 0.0f;

    // G-limit：衰减加载方向的 pitch 需求
    demand[static_cast<int>(MixSource::Pitch)] =
        applyGLimit(demand[static_cast<int>(MixSource::Pitch)], in.imu.accel_z, cfg_.glimit);

    // 混控
    mixer_.mix(demand, out.servo);

    // 外设直通：用裸 RC us 覆盖指定输出口
    for (int i = 0; i < kMaxPeripherals; ++i) {
        const PeripheralMap& p = cfg_.peripherals[i];
        if (p.enabled && p.servo_out < kNumServos)
            out.servo[p.servo_out] = in.channels[p.rc_channel];
    }
    return out;
}

}  // namespace wp
```

> **行为变化说明（review 关注点）：** 油门口 servo[2] 现在由 mixer 从 throttle 需求算出（`1000+1000*throttle_norm`），不再是裸通道直通。throttle 通道 1500 → 0.5 → servo 1500；1000→1000；2000→2000。与旧"直通"在中位/端点一致，但中段映射从"通道 us 直拷"变成"归一化重算"，数值相同（throttle 通道本就是 1000~2000）。Standard 下 roll/pitch/yaw 与旧 `mix()` 逐位等价。

- [ ] **Step 3: 加回归 + 新测试到 test_controller.cpp**

在现有测试基础上追加（main 里记得 RUN_TEST）：

```cpp
void test_standard_throttle_passthrough_equivalent() {
    wp::Controller c;
    wp::ControlInput in{}; fill_centered(in);
    in.channels[4] = 1500;          // Angle（启用 mixer 路径）
    in.channels[2] = 1700;          // throttle
    wp::ServoCommand out = c.update(in);
    TEST_ASSERT_EQUAL_UINT16(1700, out.servo[2]);   // 1000 + 0.7*1000
}

void test_peripheral_passthrough_overrides_servo() {
    wp::ControllerConfig cfg;
    cfg.peripherals[0] = wp::PeripheralMap{5, 7, true};  // servo5 <- ch8 裸值
    wp::Controller c; c.setConfig(cfg);
    wp::ControlInput in{}; fill_centered(in);
    in.channels[4] = 1500;          // Angle
    in.channels[7] = 1234;          // 起落架开关位置
    wp::ServoCommand out = c.update(in);
    TEST_ASSERT_EQUAL_UINT16(1234, out.servo[5]);
}

void test_glimit_softens_pull_in_high_g() {
    wp::Controller c;
    wp::ControlInput in{}; fill_centered(in);
    in.channels[4] = 1500;          // Angle
    in.channels[5] = 0;             // gain 0 -> 纯手动，隔离 PID 影响
    in.channels[1] = 2000;          // 升降满拉 -> pitch_cmd = +1
    in.imu.accel_z = 10.0f;         // 硬限 -> 拉杆贡献清零
    wp::ServoCommand out = c.update(in);
    // pitch 需求被清零 -> 升降回中位
    TEST_ASSERT_EQUAL_UINT16(1500, out.servo[1]);
}
```

并注册：

```cpp
    RUN_TEST(test_standard_throttle_passthrough_equivalent);
    RUN_TEST(test_peripheral_passthrough_overrides_servo);
    RUN_TEST(test_glimit_softens_pull_in_high_g);
```

> **注意 pitch 符号：** `test_glimit_softens_pull_in_high_g` 假设升降通道 2000→`channelToNorm=+1`→`pitch_demand>0`→被 G-limit 当"拉杆加载"衰减。若实测发现升降满拉对应的是负 pitch_cmd，需在 `GLimitConfig.pitch_loads_positive` 翻转，并相应改测试期望。先按正约定写，review 时确认。

- [ ] **Step 4: 运行全部测试**

Run: `cmake --build build && ctest --test-dir build --output-on-failure`
Expected: 全绿（含原有 test_ahrs/test_mode/test_pid 等不回归）。

- [ ] **Step 5: Commit**

```bash
git add core/controller.h core/controller.cpp test/test_controller/test_controller.cpp
git commit -m "feat(core): wire Mixer + G-limit + peripheral passthrough into controller"
```

---

## Task 6: SITL 不回归验证 + 暴露 Airframe（可选配置）

**Files:**
- 验证现有 `hal_sitl/run_sitl.py` 的 recover/turn 场景在新数据流下不回归（默认 Standard，行为应等价）。

- [ ] **Step 1: 构建 SITL 共享库**

Run: `cmake --build build`
Expected: `libwp_core_capi` 链接成功（C API 未改，应直接过）。

- [ ] **Step 2: 跑角度恢复场景**

Run: `cd hal_sitl && python3 run_sitl.py --scenario recover && python3 check_angle_hold.py`
Expected: `ANGLE-HOLD RECOVERY OK`（与 phase1.5 结论一致，7°→~1.3°）。

- [ ] **Step 3: 跑转弯保持场景**

Run: `python3 run_sitl.py --scenario turn && python3 check_turn_hold.py`
Expected: `TURN-HOLD TRACKING OK`（truth ~47°，est 跟踪误差 <15°）。

- [ ] **Step 4: 若两个 SITL 验证都过，Commit（仅在有改动时）**

> 本任务通常无代码改动（默认 Standard 等价旧行为）。若 SITL 因油门重算等出现数值差异，定位后修正再提交；否则跳过 commit。

```bash
# 仅当有修正改动时
git add -A && git commit -m "test(sitl): verify Phase 2 mixer path doesn't regress recover/turn"
```

---

## Self-Review 记录

- **Spec 覆盖：** 附录 D.3 Mixer（Task 1-3）、D.4 G-limit（Task 4）、D.5 外设直通（Task 5）、D.6 文件与测试（贯穿）、D.2 数据流 demand→G-limit→mixer→外设（Task 5）、D.1 失效回退（Task 5 保留早返回）。SITL 不回归（Task 6）。
- **类型一致性：** `MixSource`/`Airframe`/`MixRule`/`MixerConfig`(在类内实际用 `rules_`/`num_rules_`/`is_throttle_`)、`GLimitConfig`/`applyGLimit`、`PeripheralMap`、`Controller::mixer_` 全程一致。`mix(src, servo)` 签名在声明、实现、调用处一致。
- **饱和：** 选逐通道 clamp（D.3 确认）。按组比例缩放为远期（Betaflight 备查），本计划不实现。
- **已知风险（留给 review）：** (1) pitch 符号与 G-limit 加载方向约定，用 `pitch_loads_positive` 兜底；(2) 油门由 mixer 重算 vs 旧直通的数值等价性，已在 Task 5 说明并加回归测试。
