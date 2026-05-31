# 多路 PWM 输出扩展 + 起落架控制 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 给 WeekendPilot 加可扩展多路 PWM 输出层（原生 LEDC + I2C PCA9685）和两层起落架控制（现成控制器直通 + INA3221 堵转检测状态机），全部门控默认不使能。

**Architecture:** 沿用项目两层架构——core/ 纯算法（PC ctest 全测），HAL 写芯片驱动（编译期 WP_HAS_* 宏 + 运行时 probe 双层门控）。新增 `IServoOutput` 输出抽象 + `CompositeServoOutput` 路由（core，可测）；HAL 实现 `LedcOutput`/`Pca9685Output`。起落架：core `LandingGear` 纯状态机 + `ILandingGearActuator` 抽象接口（core），HAL 实现 `Ina3221`/`GearServo`/`GearHbridge`，由 HAL 主循环编排（不污染 Controller::update）。

**Tech Stack:** C++17、ESP32-S3 Arduino（LEDC/Wire）、PCA9685（I2C PWM）、INA3221（I2C 3 路电流监测）、Unity（PC 单测）、CMake+Ninja（PC）、PlatformIO（板载）。

**设计文档：** `docs/superpowers/specs/2026-06-01-pwm-output-landing-gear-design.md`

---

## 关键设计决策（实现前必读）

1. **不改 `kNumServos`（现 8）**。主增稳混控仍 8 路。扩展输出口（PCA9685 的 16 路）定位为"外设/慢速面/起落架"的路由目标，通过 `CompositeServoOutput` 的全局逻辑索引访问。改 `kNumServos` 会牵动 mixer/blackbox/NVS 全链布局，本计划明确不含。
   - **层 1 起落架（现成控制器）的真实边界**：现有 `PeripheralMap`（`controller.cpp:111-115`）被 `p.servo_out < kNumServos` 限制，且 `ServoCommand` 只有 8 路 —— 故现成控制器只能接 **LEDC 的 0~7 路**（标准布局主舵面用 0~3，4~7 空闲可用），这一路**今天就能用，零新代码**。设计文档 §3.1 说"PeripheralMap 自然路由到扩展口"不准确，本计划据实修正：现成控制器走空闲 LEDC 通道；扩展通道（PCA9685, ≥8）的"外设路由目标"由**层 2 的 `GearServo` 经 `g_out.writeUs` 直接驱动**来证明可用，不绕 `PeripheralMap`。把普通外设（灯等）也路由到扩展通道是后续扩展（YAGNI，本计划不含）。
2. **CompositeServoOutput 索引映射稳定**：各后端按注册顺序占据连续全局索引区间，区间宽度 = 后端 `channelCount()`（标称值，与 probe 结果无关）。PCA9685 未焊（probe 失败）时，其区间仍保留，写入被静默丢弃——保证 `PeripheralMap`/配置引用的输出索引不因缺芯片而错位。主 8 路走 LEDC（0~7）恒有效。
3. **起落架在 HAL 主循环编排，不进 `Controller::update`**：`Controller::update` 产出的 `ServoCommand` 只有 8 路，无法承载扩展通道；且把电流采样塞进 `ControlInput` 会污染纯控制核。故 core 只提供纯 `LandingGear` 状态机 + `ILandingGearActuator` 接口；HAL 主循环读 RC 收放通道 + 读 INA3221 电流 → 喂状态机 → 经 `CompositeServoOutput`/H桥驱动。这是对设计文档 §4 集成点的细化（设计文档原写"接入 controller.cpp:111 之后"）。
4. **停止 = 软件停舵**（D7）：连续旋转舵机写 1500µs；H 桥置零。不加高边开关 MOS。
5. **TDD + 频繁提交**：core 纯逻辑件先写失败测试再实现；HAL 件无法 PC 测，只做 `pio run` 编译检查（沿用项目惯例）。

## 构建命令（每个 core 任务用）

```bash
cd /c/Repository/WeekendPilot
cmake --preset dev                              # 配置（改了 CMakeLists 后重跑）
cmake --build build                             # 编译
ctest --test-dir build --output-on-failure      # 测试
```

## 板载编译检查命令（每个 HAL 任务用）

```bash
cd /c/Repository/WeekendPilot
pio run -e weekendpilot_s3                                          # 默认门控（扩展功能关）
pio run -e weekendpilot_s3 -- -DWP_HAS_PWM_EXPANDER=1               # 开 PCA9685 编译检查
```
> 注：PlatformIO 加临时 build_flag 用 `PLATFORMIO_BUILD_FLAGS` 环境变量更稳：
> `PLATFORMIO_BUILD_FLAGS="-DWP_HAS_PWM_EXPANDER=1" pio run -e weekendpilot_s3`

## 文件结构总览

**新建（core）：**
- `core/io/servo_output.h` — `IServoOutput` 纯接口
- `core/io/composite_servo_output.h` / `.cpp` — `CompositeServoOutput` 多后端路由
- `core/sensors/ina3221_decode.h` / `.cpp` — INA3221 shunt 寄存器 → 安培（纯函数）
- `core/nav/landing_gear.h` / `.cpp` — `LandingGear` 状态机 + `ILandingGearActuator` 接口

**新建（HAL，门控）：**
- `hal_esp32/src/io/ledc_output.h` / `.cpp` — `LedcOutput`（无条件可用）
- `hal_esp32/src/io/pca9685_output.h` / `.cpp` — `Pca9685Output`（WP_HAS_PWM_EXPANDER）
- `hal_esp32/src/drivers/ina3221.h` / `.cpp` — `Ina3221`（WP_HAS_LANDING_GEAR）
- `hal_esp32/src/drivers/gear_actuators.h` / `.cpp` — `GearServo` + `GearHbridge`（WP_HAS_LANDING_GEAR）

**新建（测试）：**
- `test/test_composite_servo_output/test_composite_servo_output.cpp`
- `test/test_ina3221_decode/test_ina3221_decode.cpp`
- `test/test_landing_gear/test_landing_gear.cpp`

**修改：**
- `core/controller.h` — `ControllerConfig` 加 `LandingGearConfig landing_gear` + `uint8_t gear_last_state`
- `core/config/config_store.h` — `kConfigVersion` 1 → 2
- `core/config/board_config.h` — 加 `kAddrPwmExpander`/`kAddrCurrentSense`/`kPinGearAlert`/H桥脚
- `core/config/capabilities.h` — 加 `WP_HAS_PWM_EXPANDER`/`WP_HAS_LANDING_GEAR` + constexpr 镜像
- `hal_esp32/src/main.cpp` — 用 `CompositeServoOutput` 替换裸 `ledcWrite`；加起落架编排
- `test/CMakeLists.txt` — 注册 3 个新测试

---

## Task 1: 能力宏门控 + 板级常量

新增两个编译期能力宏与板级地址/引脚常量。纯声明，不改行为，为后续任务铺路。

**Files:**
- Modify: `core/config/capabilities.h`
- Modify: `core/config/board_config.h`

- [ ] **Step 1: 在 capabilities.h 的"PC/测试默认全开"块加两宏**

`core/config/capabilities.h`，在 `#if !defined(BOARD_DEVKIT_S3)` 块内（约 line 32 的 `WP_HAS_BLACKBOX` 之后）追加：

```cpp
  #ifndef WP_HAS_PWM_EXPANDER
    #define WP_HAS_PWM_EXPANDER 1
  #endif
  #ifndef WP_HAS_LANDING_GEAR
    #define WP_HAS_LANDING_GEAR 1
  #endif
```

- [ ] **Step 2: 在 capabilities.h 的"兜底默认 0"块加两宏**

在 `#ifndef WP_HAS_BLACKBOX ... #endif`（约 line 53-55）之后追加：

```cpp
#ifndef WP_HAS_PWM_EXPANDER
  #define WP_HAS_PWM_EXPANDER 0
#endif
#ifndef WP_HAS_LANDING_GEAR
  #define WP_HAS_LANDING_GEAR 0
#endif
```

- [ ] **Step 3: 在 capabilities.h 的 constexpr 镜像块加两镜像**

在 `namespace wp {` 块内 `kHasBlackbox` 之后（约 line 64）追加：

```cpp
constexpr bool kHasPwmExpander = WP_HAS_PWM_EXPANDER;
constexpr bool kHasLandingGear = WP_HAS_LANDING_GEAR;
```

- [ ] **Step 4: 在 board_config.h 加扩展芯片地址与引脚常量**

`core/config/board_config.h`，在 `namespace wp {` 内 `kPinStatusLed` 之后（约 line 48）追加：

```cpp
// PCA9685 PWM 扩展（挂主 I2C 总线）。默认地址 0x40；A0~A5 跳线可改。
constexpr int kAddrPwmExpander = 0x40;
// PCA9685 提供的输出路数（单颗）。CompositeServoOutput 据此分配全局索引区间。
constexpr int kPwmExpanderChannels = 16;

// INA3221 三路电流监测（挂主 I2C 总线）。默认地址 0x40 会撞 PCA9685，故用 0x41。
// （INA3221 地址 0x40~0x43 由 A0 脚接 GND/VS/SDA/SCL 选）
constexpr int kAddrCurrentSense = 0x41;
// INA3221 Critical-Alert 引脚 -> ESP32 GPIO（堵转硬件中断，可选）。
// GPIO40 当前仅声明 kPinImuInt 未实际使用；若 IMU INT 启用需另选自由脚（实物核对）。
constexpr int kPinGearAlert = 41;

// 起落架 H 桥方向脚（仅类型 C 裸电机用；类型 B 连续旋转舵机不占）。
// 单电机 PWM+DIR：PWM 走 CompositeServoOutput，DIR 走此 GPIO。
constexpr int kPinGearHbridgeDir = 42;
```

- [ ] **Step 5: 构建验证（PC 全开门控编译通过）**

Run:
```bash
cmake --build build 2>&1 | tail -5
```
Expected: 编译通过（新宏在 PC 侧 =1，常量未被使用不报错，因 `-Wall -Wextra` 对未用 constexpr 不报）。

- [ ] **Step 6: 板载默认门控编译通过（功能关）**

Run:
```bash
cd /c/Repository/WeekendPilot && pio run -e weekendpilot_s3 2>&1 | tail -5
```
Expected: SUCCESS。`WP_HAS_PWM_EXPANDER`/`WP_HAS_LANDING_GEAR` 在板载默认为 0。

- [ ] **Step 7: Commit**

```bash
git add core/config/capabilities.h core/config/board_config.h
git commit -m "feat(config): 加 WP_HAS_PWM_EXPANDER/WP_HAS_LANDING_GEAR 门控 + 板级地址常量"
```

---

## Task 2: IServoOutput 抽象接口（core，纯接口）

定义输出抽象，把"把 us 值送到物理通道"从具体硬件解耦。纯头文件，无 .cpp。

**Files:**
- Create: `core/io/servo_output.h`

- [ ] **Step 1: 写接口头**

`core/io/servo_output.h`：

```cpp
#pragma once
#include <cstdint>

namespace wp {

// 舵机/PWM 输出后端抽象。把"逻辑通道 us 值 -> 物理 PWM"与硬件解耦。
// 实现方：LedcOutput（原生 8 路）、Pca9685Output（I2C 16 路）、（将来）RP2040 协处理器。
class IServoOutput {
public:
    virtual ~IServoOutput() = default;
    // 初始化并探测硬件。返回 false = 不可用（如 I2C 探测失败）。
    virtual bool begin() = 0;
    // 本后端提供的通道数（标称值，与 probe 结果无关，用于全局索引分配）。
    virtual int  channelCount() const = 0;
    // 设置 PWM 刷新频率（Hz）。后端不支持调频则忽略或夹取到能力范围。
    virtual void setFrequencyHz(uint16_t hz) = 0;
    // 写第 ch 路（后端本地索引 [0,channelCount)）的脉宽（µs）。实现内部夹 [1000,2000]。
    virtual void writeUs(int ch, uint16_t us) = 0;
};

}  // namespace wp
```

- [ ] **Step 2: 构建验证（头文件编入不破坏现有构建）**

Run: `cmake --build build 2>&1 | tail -3`
Expected: 编译通过（纯头新增，无引用方，wp_core 不变）。

- [ ] **Step 3: Commit**

```bash
git add core/io/servo_output.h
git commit -m "feat(io): 加 IServoOutput 输出后端抽象接口"
```

---

## Task 3: CompositeServoOutput 多后端路由（core，TDD）

把多个 IServoOutput 后端按注册顺序拼成连续的全局逻辑索引空间。这是路数可扩展的核心，必须 PC 单测。

**Files:**
- Create: `core/io/composite_servo_output.h`
- Create: `core/io/composite_servo_output.cpp`
- Test: `test/test_composite_servo_output/test_composite_servo_output.cpp`
- Modify: `test/CMakeLists.txt`

- [ ] **Step 1: 写头文件**

`core/io/composite_servo_output.h`：

```cpp
#pragma once
#include <cstdint>
#include "io/servo_output.h"

namespace wp {

constexpr int kMaxServoBackends = 3;  // LEDC + PCA9685 + 余量（将来 RP2040）

// 把多个后端按注册顺序拼成连续全局索引空间：
//   后端0 占 [0, c0)，后端1 占 [c0, c0+c1)，...（c_i = 后端 channelCount()）
// 区间宽度用标称 channelCount()，与 begin() 是否成功无关 —— 缺芯片时写入静默丢弃，
// 但索引不错位（PeripheralMap/配置引用的全局口稳定）。
class CompositeServoOutput {
public:
    // 注册一个后端（不取所有权）。返回 false = 已满。注册顺序决定索引区间。
    bool addBackend(IServoOutput* backend);
    // 对所有已注册后端调用 begin()。返回"至少一个成功"。各后端可用性单独记录。
    bool begin();
    // 全局通道总数（所有后端 channelCount 之和）。
    int  channelCount() const;
    // 对所有后端设频率。
    void setFrequencyHz(uint16_t hz);
    // 写全局索引 ch 的脉宽。越界或目标后端 begin 失败 -> 静默丢弃。
    void writeUs(int ch, uint16_t us);
    // 某全局索引是否可写（在范围内 且 所属后端 begin 成功）。
    bool channelReady(int ch) const;

private:
    IServoOutput* backends_[kMaxServoBackends] = {};
    bool          ready_[kMaxServoBackends] = {};
    int           count_ = 0;
};

}  // namespace wp
```

- [ ] **Step 2: 写失败测试**（见下方测试代码块，落到 `test/test_composite_servo_output/test_composite_servo_output.cpp`）

- [ ] **Step 2b: 注册测试到 CMake**

`test/CMakeLists.txt` 末尾（`wp_add_test(test_status_line)` 后）加：

```cmake
wp_add_test(test_composite_servo_output)
```

- [ ] **Step 3: 跑测试确认失败（实现未写，链接错误）**

Run: `cmake --preset dev && cmake --build build 2>&1 | tail -10`
Expected: 链接失败（`CompositeServoOutput::addBackend` 等未定义）。

**Step 2 测试代码** — `test/test_composite_servo_output/test_composite_servo_output.cpp`：

```cpp
#include "unity.h"
#include "io/composite_servo_output.h"
using namespace wp;

void setUp() {} void tearDown() {}

// 测试替身：记录写入，可配 begin 成功/失败、通道数。
class FakeOutput : public IServoOutput {
public:
    FakeOutput(int n, bool ok) : n_(n), ok_(ok) {}
    bool begin() override { begun_ = true; return ok_; }
    int  channelCount() const override { return n_; }
    void setFrequencyHz(uint16_t hz) override { freq_ = hz; }
    void writeUs(int ch, uint16_t us) override { if (ch >= 0 && ch < 32) last_[ch] = us; }
    int n_; bool ok_; bool begun_ = false; uint16_t freq_ = 0; uint16_t last_[32] = {};
};

// 两后端拼接：全局索引连续，channelCount 求和
void test_global_index_is_concatenated() {
    FakeOutput a(8, true), b(16, true);
    CompositeServoOutput c;
    TEST_ASSERT_TRUE(c.addBackend(&a));
    TEST_ASSERT_TRUE(c.addBackend(&b));
    c.begin();
    TEST_ASSERT_EQUAL_INT(24, c.channelCount());
}

// 写全局索引路由到正确后端的本地索引
void test_write_routes_to_correct_backend() {
    FakeOutput a(8, true), b(16, true);
    CompositeServoOutput c; c.addBackend(&a); c.addBackend(&b); c.begin();
    c.writeUs(0, 1111);     // 后端 a 本地 0
    c.writeUs(7, 1777);     // 后端 a 本地 7
    c.writeUs(8, 1888);     // 后端 b 本地 0
    c.writeUs(23, 1999);    // 后端 b 本地 15
    TEST_ASSERT_EQUAL_UINT16(1111, a.last_[0]);
    TEST_ASSERT_EQUAL_UINT16(1777, a.last_[7]);
    TEST_ASSERT_EQUAL_UINT16(1888, b.last_[0]);
    TEST_ASSERT_EQUAL_UINT16(1999, b.last_[15]);
}

// 后端 begin 失败：其区间仍占位（索引不错位），但写入丢弃、channelReady=false
void test_failed_backend_keeps_index_but_drops_write() {
    FakeOutput a(8, true), b(16, false);   // b 探测失败（未焊 PCA9685）
    CompositeServoOutput c; c.addBackend(&a); c.addBackend(&b); c.begin();
    TEST_ASSERT_EQUAL_INT(24, c.channelCount());   // 区间仍保留
    TEST_ASSERT_TRUE(c.channelReady(7));           // a 可用
    TEST_ASSERT_FALSE(c.channelReady(8));          // b 不可用
    c.writeUs(8, 1888);                            // 静默丢弃，不崩
    TEST_ASSERT_EQUAL_UINT16(0, b.last_[0]);       // 未写入
}

// 越界写入静默丢弃，不崩
void test_out_of_range_write_is_dropped() {
    FakeOutput a(8, true);
    CompositeServoOutput c; c.addBackend(&a); c.begin();
    c.writeUs(99, 1500);   // 越界
    c.writeUs(-1, 1500);   // 负
    TEST_ASSERT_FALSE(c.channelReady(99));
}

// 频率转发到所有后端
void test_set_frequency_forwarded() {
    FakeOutput a(8, true), b(16, true);
    CompositeServoOutput c; c.addBackend(&a); c.addBackend(&b); c.begin();
    c.setFrequencyHz(50);
    TEST_ASSERT_EQUAL_UINT16(50, a.freq_);
    TEST_ASSERT_EQUAL_UINT16(50, b.freq_);
}

// 后端注册上限：超出返回 false
void test_backend_capacity_limit() {
    FakeOutput a(1,true), b(1,true), d(1,true), e(1,true);
    CompositeServoOutput c;
    TEST_ASSERT_TRUE(c.addBackend(&a));
    TEST_ASSERT_TRUE(c.addBackend(&b));
    TEST_ASSERT_TRUE(c.addBackend(&d));
    TEST_ASSERT_FALSE(c.addBackend(&e));   // kMaxServoBackends=3
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_global_index_is_concatenated);
    RUN_TEST(test_write_routes_to_correct_backend);
    RUN_TEST(test_failed_backend_keeps_index_but_drops_write);
    RUN_TEST(test_out_of_range_write_is_dropped);
    RUN_TEST(test_set_frequency_forwarded);
    RUN_TEST(test_backend_capacity_limit);
    return UNITY_END();
}
```

- [ ] **Step 4: 写实现**

`core/io/composite_servo_output.cpp`：

```cpp
#include "io/composite_servo_output.h"

namespace wp {

bool CompositeServoOutput::addBackend(IServoOutput* backend) {
    if (count_ >= kMaxServoBackends || backend == nullptr) return false;
    backends_[count_++] = backend;
    return true;
}

bool CompositeServoOutput::begin() {
    bool any = false;
    for (int i = 0; i < count_; ++i) {
        ready_[i] = backends_[i]->begin();
        any = any || ready_[i];
    }
    return any;
}

int CompositeServoOutput::channelCount() const {
    int sum = 0;
    for (int i = 0; i < count_; ++i) sum += backends_[i]->channelCount();
    return sum;
}

void CompositeServoOutput::setFrequencyHz(uint16_t hz) {
    for (int i = 0; i < count_; ++i) backends_[i]->setFrequencyHz(hz);
}

// 把全局索引解析为 (后端下标, 本地索引)。返回后端下标，local 写出本地索引；越界返回 -1。
static int locate(IServoOutput* const* b, int count, int ch, int& local) {
    if (ch < 0) return -1;
    int base = 0;
    for (int i = 0; i < count; ++i) {
        int n = b[i]->channelCount();
        if (ch < base + n) { local = ch - base; return i; }
        base += n;
    }
    return -1;
}

void CompositeServoOutput::writeUs(int ch, uint16_t us) {
    int local = 0;
    int idx = locate(backends_, count_, ch, local);
    if (idx < 0 || !ready_[idx]) return;   // 越界或后端不可用 -> 静默丢弃
    backends_[idx]->writeUs(local, us);
}

bool CompositeServoOutput::channelReady(int ch) const {
    int local = 0;
    int idx = locate(backends_, count_, ch, local);
    return idx >= 0 && ready_[idx];
}

}  // namespace wp
```

- [ ] **Step 5: 跑测试确认通过**

Run: `cmake --build build && ctest --test-dir build -R test_composite_servo_output --output-on-failure`
Expected: PASS（6 个测试全过）。

- [ ] **Step 6: 全量回归（不破坏现有 22 套）**

Run: `ctest --test-dir build --output-on-failure 2>&1 | tail -5`
Expected: 全部通过（23 套）。

- [ ] **Step 7: Commit**

```bash
git add core/io/composite_servo_output.h core/io/composite_servo_output.cpp \
        test/test_composite_servo_output/ test/CMakeLists.txt
git commit -m "feat(io): CompositeServoOutput 多后端路由 + 6 单测"
```

---

## Task 4: LedcOutput HAL 驱动（从 main.cpp 抽出，行为不变）

把 `main.cpp` 现有的 `setupPwm`/`writeServoUs` 逻辑包成 `IServoOutput` 实现，行为零变化。HAL 件无法 PC 测，做 pio 编译检查。

**Files:**
- Create: `hal_esp32/src/io/ledc_output.h`
- Create: `hal_esp32/src/io/ledc_output.cpp`

- [ ] **Step 1: 写头文件**

`hal_esp32/src/io/ledc_output.h`：

```cpp
#pragma once
#include "io/servo_output.h"

namespace wp {

// 原生 ESP32-S3 LEDC 输出（8 路）。包装现 main.cpp 的 setupPwm/writeServoUs。
// 频率默认 50Hz、16-bit 分辨率（与现行为一致）。begin() 恒成功（无外部硬件可探测）。
class LedcOutput : public IServoOutput {
public:
    // pins: 8 个 GPIO；count: 路数（<=8）。
    LedcOutput(const int* pins, int count);
    bool begin() override;
    int  channelCount() const override { return count_; }
    void setFrequencyHz(uint16_t hz) override;
    void writeUs(int ch, uint16_t us) override;

private:
    const int* pins_;
    int        count_;
    uint16_t   freq_hz_ = 50;
};

}  // namespace wp
```

- [ ] **Step 2: 写实现（保持现 main.cpp 的占空比换算逻辑）**

`hal_esp32/src/io/ledc_output.cpp`：

```cpp
#include "io/ledc_output.h"
#include <Arduino.h>

namespace wp {

LedcOutput::LedcOutput(const int* pins, int count) : pins_(pins), count_(count) {}

bool LedcOutput::begin() {
    for (int i = 0; i < count_; ++i) {
        ledcSetup(i, freq_hz_, 16);
        ledcAttachPin(pins_[i], i);
        writeUs(i, 1500);   // 上电中位
    }
    return true;   // 无外部硬件，恒可用
}

void LedcOutput::setFrequencyHz(uint16_t hz) {
    freq_hz_ = hz;
    for (int i = 0; i < count_; ++i) ledcSetup(i, freq_hz_, 16);
}

void LedcOutput::writeUs(int ch, uint16_t us) {
    if (ch < 0 || ch >= count_) return;
    if (us < 1000) us = 1000;
    if (us > 2000) us = 2000;
    // 占空比 = us / 周期；周期 = 1e6/freq µs。沿用现 main.cpp 的 16-bit 满量程换算。
    uint32_t period_us = 1000000UL / freq_hz_;
    uint32_t duty = (uint32_t)((double)us / (double)period_us * 65535.0);
    ledcWrite(ch, duty);
}

}  // namespace wp
```

> 注：现 main.cpp 硬编码 `20000.0`（=1e6/50）。这里用 `period_us = 1e6/freq` 泛化，freq=50 时数值等价（20000），行为不变。

- [ ] **Step 3: 板载编译检查**

Run: `pio run -e weekendpilot_s3 2>&1 | tail -5`
Expected: SUCCESS（ledc_output.cpp 编入；注意此时 main.cpp 尚未改用它，仅验证可编译）。

> 若 `pio` 报 `ledc_output.cpp` 未被纳入编译：确认 `build_src_filter = +<*> +<../../core/>` 覆盖 `hal_esp32/src/io/`（递归 `+<*>` 已覆盖子目录，应无需改）。

- [ ] **Step 4: Commit**

```bash
git add hal_esp32/src/io/ledc_output.h hal_esp32/src/io/ledc_output.cpp
git commit -m "feat(hal): LedcOutput 包装原生 8 路 LEDC 为 IServoOutput"
```

---

## Task 5: Pca9685Output HAL 驱动（I2C 扩展，门控）

PCA9685 I2C 驱动，门控 `WP_HAS_PWM_EXPANDER`。寄存器逻辑参考成熟实现（RobTillaart/PCA9685_RT），标注 provenance（[[weekendpilot-port-mature-fc]]）。

**Files:**
- Create: `hal_esp32/src/io/pca9685_output.h`
- Create: `hal_esp32/src/io/pca9685_output.cpp`

- [ ] **Step 1: 写头文件**

`hal_esp32/src/io/pca9685_output.h`：

```cpp
#pragma once
#if WP_HAS_PWM_EXPANDER
#include "io/servo_output.h"
#include <Wire.h>

namespace wp {

// PCA9685 16 路 12-bit PWM 扩展（I2C）。全局单一频率（24~1526Hz）。
// 寄存器序列参考 NXP PCA9685 datasheet §7 + RobTillaart/PCA9685_RT（provenance）。
class Pca9685Output : public IServoOutput {
public:
    Pca9685Output(TwoWire& wire, uint8_t addr);
    bool begin() override;                         // probe（读 MODE1）+ 设频率
    int  channelCount() const override { return 16; }
    void setFrequencyHz(uint16_t hz) override;     // 写 PRE_SCALE（需先进睡眠）
    void writeUs(int ch, uint16_t us) override;    // us -> 12-bit tick -> LEDn_ON/OFF

private:
    bool writeReg(uint8_t reg, uint8_t val);
    bool readReg(uint8_t reg, uint8_t& val);
    TwoWire& wire_;
    uint8_t  addr_;
    uint16_t freq_hz_ = 50;
    bool     ok_ = false;
};

}  // namespace wp
#endif  // WP_HAS_PWM_EXPANDER
```

- [ ] **Step 2: 写实现**（见下方代码块）

`hal_esp32/src/io/pca9685_output.cpp`：

```cpp
#include "io/pca9685_output.h"
#if WP_HAS_PWM_EXPANDER

namespace wp {

// 寄存器地址（datasheet §7.3）
static constexpr uint8_t REG_MODE1     = 0x00;
static constexpr uint8_t REG_LED0_ON_L = 0x06;  // 每路 4 字节：ON_L/ON_H/OFF_L/OFF_H
static constexpr uint8_t REG_PRESCALE  = 0xFE;
static constexpr uint8_t MODE1_SLEEP   = 0x10;
static constexpr uint8_t MODE1_AI      = 0x20;  // 自动递增
static constexpr uint8_t MODE1_RESTART = 0x80;

Pca9685Output::Pca9685Output(TwoWire& wire, uint8_t addr) : wire_(wire), addr_(addr) {}

bool Pca9685Output::writeReg(uint8_t reg, uint8_t val) {
    wire_.beginTransmission(addr_);
    wire_.write(reg);
    wire_.write(val);
    return wire_.endTransmission() == 0;
}

bool Pca9685Output::readReg(uint8_t reg, uint8_t& val) {
    wire_.beginTransmission(addr_);
    wire_.write(reg);
    if (wire_.endTransmission(false) != 0) return false;
    if (wire_.requestFrom((int)addr_, 1) != 1) return false;
    val = wire_.read();
    return true;
}

bool Pca9685Output::begin() {
    uint8_t m1 = 0;
    if (!readReg(REG_MODE1, m1)) { ok_ = false; return false; }  // probe：读不到 = 未焊
    if (!writeReg(REG_MODE1, MODE1_AI)) { ok_ = false; return false; }
    setFrequencyHz(freq_hz_);
    ok_ = true;
    return true;
}

void Pca9685Output::setFrequencyHz(uint16_t hz) {
    freq_hz_ = hz;
    // prescale = round(25MHz / (4096 * freq)) - 1（datasheet §7.3.5）
    if (hz < 24) hz = 24;
    if (hz > 1526) hz = 1526;
    uint8_t prescale = (uint8_t)((25000000.0 / (4096.0 * hz)) - 0.5);
    uint8_t m1 = 0;
    readReg(REG_MODE1, m1);
    writeReg(REG_MODE1, (uint8_t)((m1 & ~MODE1_RESTART) | MODE1_SLEEP));  // 进睡眠才能写 prescale
    writeReg(REG_PRESCALE, prescale);
    writeReg(REG_MODE1, (uint8_t)(m1 | MODE1_AI));                         // 唤醒
    delayMicroseconds(500);
    writeReg(REG_MODE1, (uint8_t)(m1 | MODE1_AI | MODE1_RESTART));         // restart
}

void Pca9685Output::writeUs(int ch, uint16_t us) {
    if (!ok_ || ch < 0 || ch >= 16) return;
    if (us < 1000) us = 1000;
    if (us > 2000) us = 2000;
    // us -> tick（12-bit @ freq）：tick = us / (1e6/freq) * 4096
    uint32_t period_us = 1000000UL / freq_hz_;
    uint16_t off = (uint16_t)((uint32_t)us * 4096UL / period_us);
    if (off > 4095) off = 4095;
    uint8_t reg = (uint8_t)(REG_LED0_ON_L + 4 * ch);
    wire_.beginTransmission(addr_);
    wire_.write(reg);
    wire_.write(0x00); wire_.write(0x00);              // ON = 0
    wire_.write((uint8_t)(off & 0xFF));                // OFF_L
    wire_.write((uint8_t)((off >> 8) & 0x0F));         // OFF_H
    wire_.endTransmission();
}

}  // namespace wp
#endif  // WP_HAS_PWM_EXPANDER
```

- [ ] **Step 3: 板载编译检查（功能关，应不编入）**

Run: `pio run -e weekendpilot_s3 2>&1 | tail -5`
Expected: SUCCESS（`WP_HAS_PWM_EXPANDER=0`，整个 .cpp 被 `#if` 包空，不编入）。

- [ ] **Step 4: 板载编译检查（开门控）**

Run: `PLATFORMIO_BUILD_FLAGS="-DWP_HAS_PWM_EXPANDER=1" pio run -e weekendpilot_s3 2>&1 | tail -8`
Expected: SUCCESS（PCA9685 驱动编入，无未定义符号/警告）。

- [ ] **Step 5: Commit**

```bash
git add hal_esp32/src/io/pca9685_output.h hal_esp32/src/io/pca9685_output.cpp
git commit -m "feat(hal): Pca9685Output I2C 16路 PWM 扩展驱动（门控 WP_HAS_PWM_EXPANDER）"
```

---

## Task 6: main.cpp 接入 CompositeServoOutput（替换裸 ledcWrite）

把 main.cpp 的输出路径从直接 `ledcWrite` 改为经 `CompositeServoOutput`。主 8 路走 LEDC，PCA9685（开门控时）接在其后。行为对前 8 路不变。

**Files:**
- Modify: `hal_esp32/src/main.cpp`

- [ ] **Step 1: 加 include 与全局对象**

`main.cpp` 顶部 include 区（约 line 18 `status_line.h` 后）加：

```cpp
#include "io/composite_servo_output.h"
#include "io/ledc_output.h"
#if WP_HAS_PWM_EXPANDER
#include "io/pca9685_output.h"
#endif
```

全局对象区（约 line 26 `g_frontend` 附近）加：

```cpp
static wp::CompositeServoOutput g_out;
static wp::LedcOutput g_ledc(kServoPins, wp::kNumServos);
#if WP_HAS_PWM_EXPANDER
static wp::Pca9685Output g_pca(Wire, wp::kAddrPwmExpander);
#endif
```

- [ ] **Step 2: 删除旧 setupPwm/writeServoUs，改用 g_out**

删掉 `setupPwm()`（line 49-55）和 `writeServoUs()`（line 57-61）两个函数。

- [ ] **Step 3: setup() 内注册后端并初始化**

`setup()` 内把原 `setupPwm();`（约 line 116）替换为：

```cpp
    g_out.addBackend(&g_ledc);
#if WP_HAS_PWM_EXPANDER
    g_out.addBackend(&g_pca);
#endif
    g_out.begin();
    g_out.setFrequencyHz(50);   // D2：默认 50Hz，将来从 NVS config 读
```

- [ ] **Step 4: loop() 内改用 g_out.writeUs**

`loop()` 内原（约 line 152）：

```cpp
    for (int i = 0; i < wp::kNumServos; ++i) writeServoUs(i, out.servo[i]);
```

替换为：

```cpp
    for (int i = 0; i < wp::kNumServos; ++i) g_out.writeUs(i, out.servo[i]);
```

- [ ] **Step 5: 板载编译检查（默认门控）**

Run: `pio run -e weekendpilot_s3 2>&1 | tail -6`
Expected: SUCCESS。RAM/Flash 与基线相近（仅多一层薄封装）。

- [ ] **Step 6: 板载编译检查（开 PCA9685）**

Run: `PLATFORMIO_BUILD_FLAGS="-DWP_HAS_PWM_EXPANDER=1" pio run -e weekendpilot_s3 2>&1 | tail -6`
Expected: SUCCESS。

- [ ] **Step 7: Commit**

```bash
git add hal_esp32/src/main.cpp
git commit -m "refactor(hal): main.cpp 输出走 CompositeServoOutput（前8路LEDC行为不变，PCA9685可门控接入）"
```

---

## Task 7: INA3221 寄存器解码（core 纯函数，TDD）

INA3221 shunt 电压寄存器 → 电流（安培）的纯换算，PC 可测（沿用 ist8310_decode/ms4525_decode 套路）。寄存器格式来自 TI INA3221 datasheet（provenance）。

**Files:**
- Create: `core/sensors/ina3221_decode.h`
- Create: `core/sensors/ina3221_decode.cpp`
- Test: `test/test_ina3221_decode/test_ina3221_decode.cpp`
- Modify: `test/CMakeLists.txt`

- [ ] **Step 1: 写头文件**

`core/sensors/ina3221_decode.h`：

```cpp
#pragma once
#include <cstdint>

namespace wp {

// INA3221 shunt-voltage 寄存器（每通道一个，0x01/0x03/0x05）。
// 格式：13-bit 有符号，存于 bit15..3（bit2..0 恒 0）。LSB = 40µV（datasheet §8.6.1）。
// raw16 = I2C 读到的 16-bit 大端寄存器值（高字节先到，调用方组装为 uint16）。
// 返回 shunt 电压（V）。
float ina3221ShuntVolt(uint16_t raw16);

// shunt 电压（V）+ shunt 电阻（Ω）-> 电流（A）。I = Vshunt / Rshunt。
float ina3221Current(uint16_t raw16, float shunt_ohm);

}  // namespace wp
```

- [ ] **Step 2: 写失败测试**（见下方代码块，落到 `test/test_ina3221_decode/test_ina3221_decode.cpp`）

- [ ] **Step 2b: 注册测试**

`test/CMakeLists.txt` 末尾加：

```cmake
wp_add_test(test_ina3221_decode)
```

- [ ] **Step 3: 跑测试确认失败**

Run: `cmake --preset dev && cmake --build build 2>&1 | tail -10`
Expected: 链接失败（`ina3221ShuntVolt` 未定义）。

**Step 2 测试代码** — `test/test_ina3221_decode/test_ina3221_decode.cpp`：

```cpp
#include "unity.h"
#include "sensors/ina3221_decode.h"
using namespace wp;

void setUp() {} void tearDown() {}

// LSB = 40µV，值在 bit15..3。raw=0x07D0(=2000) -> v=2000>>3=250 -> 250*40µV=0.01V
void test_shunt_volt_positive() {
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.01f, ina3221ShuntVolt(0x07D0));
}

// 负值：13-bit 有符号需正确符号扩展。-0.01V -> v=-250 -> (-250<<3)=0xF830
void test_shunt_volt_negative() {
    TEST_ASSERT_FLOAT_WITHIN(1e-6, -0.01f, ina3221ShuntVolt(0xF830));
}

// 零
void test_shunt_volt_zero() {
    TEST_ASSERT_FLOAT_WITHIN(1e-9, 0.0f, ina3221ShuntVolt(0x0000));
}

// bit2..0 应被忽略（恒 0，但即便有杂位也不影响）
void test_low_bits_ignored() {
    // 0x07D0 | 0x0007 = 0x07D7；>>3 仍 = 250
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.01f, ina3221ShuntVolt(0x07D7));
}

// 电流：I = Vshunt / Rshunt。0.01V / 0.01Ω = 1.0A
void test_current_positive() {
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 1.0f, ina3221Current(0x07D0, 0.01f));
}

// 负电流
void test_current_negative() {
    TEST_ASSERT_FLOAT_WITHIN(1e-4, -1.0f, ina3221Current(0xF830, 0.01f));
}

// shunt=0 防护：返回 0 而非 inf/nan
void test_current_zero_shunt_guarded() {
    float i = ina3221Current(0x07D0, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-9, 0.0f, i);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_shunt_volt_positive);
    RUN_TEST(test_shunt_volt_negative);
    RUN_TEST(test_shunt_volt_zero);
    RUN_TEST(test_low_bits_ignored);
    RUN_TEST(test_current_positive);
    RUN_TEST(test_current_negative);
    RUN_TEST(test_current_zero_shunt_guarded);
    return UNITY_END();
}
```

- [ ] **Step 4: 写实现**

`core/sensors/ina3221_decode.cpp`：

```cpp
#include "sensors/ina3221_decode.h"

namespace wp {

float ina3221ShuntVolt(uint16_t raw16) {
    // 值在 bit15..3，13-bit 有符号。用 int16_t 算术右移 3 保符号。
    int16_t v = static_cast<int16_t>(raw16);
    v = static_cast<int16_t>(v >> 3);   // 13-bit 有符号，范围 [-4096,4095]
    return static_cast<float>(v) * 40e-6f;   // LSB = 40µV
}

float ina3221Current(uint16_t raw16, float shunt_ohm) {
    if (shunt_ohm <= 0.0f) return 0.0f;   // 防 0 除
    return ina3221ShuntVolt(raw16) / shunt_ohm;
}

}  // namespace wp
```

- [ ] **Step 5: 跑测试确认通过**

Run: `cmake --build build && ctest --test-dir build -R test_ina3221_decode --output-on-failure`
Expected: PASS（7 个测试）。

- [ ] **Step 6: 全量回归**

Run: `ctest --test-dir build --output-on-failure 2>&1 | tail -5`
Expected: 全过（24 套）。

- [ ] **Step 7: Commit**

```bash
git add core/sensors/ina3221_decode.h core/sensors/ina3221_decode.cpp \
        test/test_ina3221_decode/ test/CMakeLists.txt
git commit -m "feat(sensors): INA3221 shunt电压/电流解码纯函数 + 7 单测"
```

---

## Task 8: LandingGear 状态机 + ILandingGearActuator 接口（core，TDD）

起落架核心逻辑：纯状态机，喂"收/放指令 + 电流采样"，产出"驱动指令 + 状态"。支持类型 B（连续旋转舵机，PWM 跨 1500 中点）与类型 C（H 桥，方向枚举）——驱动差异由 `ILandingGearActuator` 隔离，状态机只产出抽象 `drive` 语义。

**Files:**
- Create: `core/nav/landing_gear.h`
- Create: `core/nav/landing_gear.cpp`
- Test: `test/test_landing_gear/test_landing_gear.cpp`
- Modify: `test/CMakeLists.txt`

- [ ] **Step 1: 写头文件**

`core/nav/landing_gear.h`：

```cpp
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
```

- [ ] **Step 2: 写失败测试** — `test/test_landing_gear/test_landing_gear.cpp`：

```cpp
#include "unity.h"
#include "nav/landing_gear.h"
using namespace wp;

void setUp() {} void tearDown() {}

static LandingGearConfig cfg() {
    LandingGearConfig c;
    c.enabled = true;
    c.stall_current_a = 2.0f;
    c.stall_debounce_ms = 80;
    c.timeout_ms = 4000;
    return c;
}

// 未使能：永远 Stop，状态不变
void test_disabled_always_stop() {
    LandingGear g; LandingGearConfig c = cfg(); c.enabled = false; g.setConfig(c);
    LandingGearInputs in; in.deploy_cmd = true; in.dt = 0.02f;
    LandingGearOutput o = g.update(in);
    TEST_ASSERT_EQUAL(GearDrive::Stop, o.drive);
    TEST_ASSERT_EQUAL(GearState::Retracted, o.state);
}

// 上电首拍不因 deploy_cmd 初值误触发：have_last_cmd 播种，需跳变沿才动
void test_no_trigger_on_first_tick() {
    LandingGear g; g.setConfig(cfg());
    LandingGearInputs in; in.deploy_cmd = true; in.dt = 0.02f;  // 上电就是 true
    LandingGearOutput o = g.update(in);
    TEST_ASSERT_EQUAL(GearState::Retracted, o.state);   // 不动
    TEST_ASSERT_EQUAL(GearDrive::Stop, o.drive);
}

// 收->放跳变沿：进入 Deploying 并输出 Deploy 驱动
void test_deploy_command_starts_deploying() {
    LandingGear g; g.setConfig(cfg());
    LandingGearInputs in; in.dt = 0.02f;
    in.deploy_cmd = false; g.update(in);     // 播种"收"
    in.deploy_cmd = true;  LandingGearOutput o = g.update(in);  // 跳变 -> 放
    TEST_ASSERT_EQUAL(GearState::Deploying, o.state);
    TEST_ASSERT_EQUAL(GearDrive::Deploy, o.drive);
}

// Deploying 中电流超阈持续够久（去抖）-> Deployed + Stop
void test_stall_debounce_reaches_deployed() {
    LandingGear g; g.setConfig(cfg());
    LandingGearInputs in; in.dt = 0.02f;
    in.deploy_cmd = false; g.update(in);
    in.deploy_cmd = true;  g.update(in);     // Deploying
    in.current_a = 3.0f;                      // 超阈 2.0
    // 80ms 去抖 @20ms/拍 = 需 >=4 拍持续超阈
    LandingGearOutput o;
    for (int i = 0; i < 5; ++i) o = g.update(in);
    TEST_ASSERT_EQUAL(GearState::Deployed, o.state);
    TEST_ASSERT_EQUAL(GearDrive::Stop, o.drive);
}

// 电流抖动（未连续超阈去抖时长）不应误判到位
void test_current_glitch_does_not_latch() {
    LandingGear g; g.setConfig(cfg());
    LandingGearInputs in; in.dt = 0.02f;
    in.deploy_cmd = false; g.update(in);
    in.deploy_cmd = true;  g.update(in);     // Deploying
    in.current_a = 3.0f; g.update(in);        // 1 拍超阈(20ms)
    in.current_a = 0.5f;                      // 回落
    LandingGearOutput o = g.update(in);
    TEST_ASSERT_EQUAL(GearState::Deploying, o.state);   // 未到 80ms，仍在 Deploying
}

// 硬件 ALERT 为快速路径：alert=true 等价电流超阈累计去抖
void test_alert_flag_counts_as_overcurrent() {
    LandingGear g; g.setConfig(cfg());
    LandingGearInputs in; in.dt = 0.02f;
    in.deploy_cmd = false; g.update(in);
    in.deploy_cmd = true;  g.update(in);
    in.current_a = 0.0f; in.alert = true;     // 电流读数低但 ALERT 触发
    LandingGearOutput o;
    for (int i = 0; i < 5; ++i) o = g.update(in);
    TEST_ASSERT_EQUAL(GearState::Deployed, o.state);
}

// Deployed 后收到"收"跳变 -> Retracting -> 堵转 -> Retracted
void test_retract_cycle() {
    LandingGear g; g.setConfig(cfg());
    LandingGearInputs in; in.dt = 0.02f;
    in.deploy_cmd = false; g.update(in);
    in.deploy_cmd = true;  g.update(in);
    in.current_a = 3.0f; for (int i=0;i<5;++i) g.update(in);   // -> Deployed
    in.current_a = 0.0f; in.deploy_cmd = false;                 // 跳变"收"
    LandingGearOutput o = g.update(in);
    TEST_ASSERT_EQUAL(GearState::Retracting, o.state);
    TEST_ASSERT_EQUAL(GearDrive::Retract, o.drive);
    in.current_a = 3.0f; for (int i=0;i<5;++i) o = g.update(in);   // 堵转
    TEST_ASSERT_EQUAL(GearState::Retracted, o.state);
    TEST_ASSERT_EQUAL(GearDrive::Stop, o.drive);
}

// 行程超时无堵转 -> Fault + Stop（防烧）
void test_timeout_enters_fault() {
    LandingGear g; g.setConfig(cfg());   // timeout 4000ms
    LandingGearInputs in; in.dt = 0.02f;
    in.deploy_cmd = false; g.update(in);
    in.deploy_cmd = true;  g.update(in);     // Deploying
    in.current_a = 0.0f;                      // 永不堵转
    LandingGearOutput o;
    for (int i = 0; i < 205; ++i) o = g.update(in);   // 205*20ms=4100ms > 4000
    TEST_ASSERT_EQUAL(GearState::Fault, o.state);
    TEST_ASSERT_EQUAL(GearDrive::Stop, o.drive);
}

// initState 播种：上电恢复 Deployed，不主动驱动
void test_init_state_no_drive() {
    LandingGear g; g.setConfig(cfg());
    g.initState(GearState::Deployed);
    LandingGearInputs in; in.deploy_cmd = true; in.dt = 0.02f;   // 指令与状态一致
    LandingGearOutput o = g.update(in);
    TEST_ASSERT_EQUAL(GearState::Deployed, o.state);
    TEST_ASSERT_EQUAL(GearDrive::Stop, o.drive);   // 不动
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_disabled_always_stop);
    RUN_TEST(test_no_trigger_on_first_tick);
    RUN_TEST(test_deploy_command_starts_deploying);
    RUN_TEST(test_stall_debounce_reaches_deployed);
    RUN_TEST(test_current_glitch_does_not_latch);
    RUN_TEST(test_alert_flag_counts_as_overcurrent);
    RUN_TEST(test_retract_cycle);
    RUN_TEST(test_timeout_enters_fault);
    RUN_TEST(test_init_state_no_drive);
    return UNITY_END();
}
```

- [ ] **Step 2b: 注册测试**

`test/CMakeLists.txt` 末尾加：

```cmake
wp_add_test(test_landing_gear)
```

- [ ] **Step 3: 跑测试确认失败**

Run: `cmake --preset dev && cmake --build build 2>&1 | tail -10`
Expected: 链接失败（`LandingGear::update` 未定义）。

- [ ] **Step 4: 写实现**

`core/nav/landing_gear.cpp`：

```cpp
#include "nav/landing_gear.h"

namespace wp {

void LandingGear::setConfig(const LandingGearConfig& c) { cfg_ = c; }

void LandingGear::initState(GearState s) {
    state_ = s;
    stall_timer_ms_ = 0.0f;
    travel_timer_ms_ = 0.0f;
    // 不动 last_deploy_cmd_/have_last_cmd_：上电首拍仍需播种指令、靠跳变沿才动作。
}

LandingGearOutput LandingGear::update(const LandingGearInputs& in) {
    LandingGearOutput out;
    if (!cfg_.enabled) {
        out.drive = GearDrive::Stop;
        out.state = state_;
        return out;
    }

    // 指令跳变沿检测：首拍只播种，不动作（防上电误触发，设计文档 §9）。
    bool edge_deploy = false, edge_retract = false;
    if (!have_last_cmd_) {
        have_last_cmd_ = true;
    } else if (in.deploy_cmd != last_deploy_cmd_) {
        edge_deploy  = in.deploy_cmd;     // false->true：放下
        edge_retract = !in.deploy_cmd;    // true->false：收起
    }
    last_deploy_cmd_ = in.deploy_cmd;

    // 跳变触发状态切换（任意稳态/行程态收到反向指令都重启对应行程）。
    if (edge_deploy && state_ != GearState::Deploying) {
        state_ = GearState::Deploying;
        stall_timer_ms_ = 0.0f; travel_timer_ms_ = 0.0f;
    } else if (edge_retract && state_ != GearState::Retracting) {
        state_ = GearState::Retracting;
        stall_timer_ms_ = 0.0f; travel_timer_ms_ = 0.0f;
    }

    const float dt_ms = in.dt * 1000.0f;
    const bool overcurrent = in.alert || (in.current_a >= cfg_.stall_current_a);

    switch (state_) {
        case GearState::Deploying:
        case GearState::Retracting: {
            travel_timer_ms_ += dt_ms;
            if (overcurrent) stall_timer_ms_ += dt_ms;
            else             stall_timer_ms_ = 0.0f;   // 抖动复位去抖计时

            if (stall_timer_ms_ >= cfg_.stall_debounce_ms) {
                state_ = (state_ == GearState::Deploying) ? GearState::Deployed
                                                          : GearState::Retracted;
                out.drive = GearDrive::Stop;
            } else if (travel_timer_ms_ >= cfg_.timeout_ms) {
                state_ = GearState::Fault;             // 行程超时无堵转 -> 防烧
                out.drive = GearDrive::Stop;
            } else {
                out.drive = (state_ == GearState::Deploying) ? GearDrive::Deploy
                                                             : GearDrive::Retract;
            }
            break;
        }
        case GearState::Retracted:
        case GearState::Deployed:
        case GearState::Fault:
        default:
            out.drive = GearDrive::Stop;               // 稳态/故障：软件停舵
            break;
    }

    out.state = state_;
    return out;
}

}  // namespace wp
```

- [ ] **Step 5: 跑测试确认通过**

Run: `cmake --build build && ctest --test-dir build -R test_landing_gear --output-on-failure`
Expected: PASS（9 个测试）。

- [ ] **Step 6: 全量回归**

Run: `ctest --test-dir build --output-on-failure 2>&1 | tail -5`
Expected: 全过（25 套）。

- [ ] **Step 7: Commit**

```bash
git add core/nav/landing_gear.h core/nav/landing_gear.cpp \
        test/test_landing_gear/ test/CMakeLists.txt
git commit -m "feat(nav): LandingGear 堵转检测状态机 + ILandingGearActuator 接口 + 9 单测"
```

---

## Task 9: INA3221 驱动 + 起落架执行机构（HAL，门控）

HAL 侧：INA3221 I2C 读电流（用 Task 7 的解码）、`GearServo`（类型 B）和 `GearHbridge`（类型 C）两个 `ILandingGearActuator` 实现。门控 `WP_HAS_LANDING_GEAR`。无法 PC 测，做 pio 编译检查。

**Files:**
- Create: `hal_esp32/src/drivers/ina3221.h`
- Create: `hal_esp32/src/drivers/ina3221.cpp`
- Create: `hal_esp32/src/drivers/gear_actuators.h`
- Create: `hal_esp32/src/drivers/gear_actuators.cpp`

- [ ] **Step 1: 写 INA3221 驱动头**

`hal_esp32/src/drivers/ina3221.h`：

```cpp
#pragma once
#if WP_HAS_LANDING_GEAR
#include <cstdint>
#include <Wire.h>

namespace wp {

// INA3221 三路电流监测（I2C）。读 shunt 电压寄存器，经 ina3221_decode 换算电流。
class Ina3221 {
public:
    Ina3221(TwoWire& wire, uint8_t addr) : wire_(wire), addr_(addr) {}
    bool begin();                                  // probe（读 manufacturer ID 0xFE）
    // 读通道 ch（0..2）电流（A）。shunt_ohm 为该路采样电阻。失败返回 false。
    bool readCurrent(int ch, float shunt_ohm, float& out_a);

private:
    bool readReg(uint8_t reg, uint16_t& val);
    TwoWire& wire_;
    uint8_t  addr_;
    bool     ok_ = false;
};

}  // namespace wp
#endif  // WP_HAS_LANDING_GEAR
```

- [ ] **Step 2: 写 INA3221 驱动实现**

`hal_esp32/src/drivers/ina3221.cpp`：

```cpp
#include "drivers/ina3221.h"
#if WP_HAS_LANDING_GEAR
#include "sensors/ina3221_decode.h"

namespace wp {

// shunt 电压寄存器：ch0=0x01, ch1=0x03, ch2=0x05（datasheet §8.6）
static constexpr uint8_t REG_SHUNT[3] = {0x01, 0x03, 0x05};
static constexpr uint8_t REG_MANUF_ID = 0xFE;   // 应读回 0x5449 ('TI')

bool Ina3221::readReg(uint8_t reg, uint16_t& val) {
    wire_.beginTransmission(addr_);
    wire_.write(reg);
    if (wire_.endTransmission(false) != 0) return false;
    if (wire_.requestFrom((int)addr_, 2) != 2) return false;
    uint8_t hi = wire_.read();
    uint8_t lo = wire_.read();
    val = (uint16_t)((hi << 8) | lo);   // 大端
    return true;
}

bool Ina3221::begin() {
    uint16_t id = 0;
    ok_ = readReg(REG_MANUF_ID, id) && (id == 0x5449);
    return ok_;
}

bool Ina3221::readCurrent(int ch, float shunt_ohm, float& out_a) {
    if (!ok_ || ch < 0 || ch > 2) return false;
    uint16_t raw = 0;
    if (!readReg(REG_SHUNT[ch], raw)) return false;
    out_a = ina3221Current(raw, shunt_ohm);
    return true;
}

}  // namespace wp
#endif  // WP_HAS_LANDING_GEAR
```

- [ ] **Step 3: 写执行机构头（类型 B + C）**

`hal_esp32/src/drivers/gear_actuators.h`：

```cpp
#pragma once
#if WP_HAS_LANDING_GEAR
#include "nav/landing_gear.h"
#include "io/composite_servo_output.h"

namespace wp {

// 类型 B：连续旋转舵机。Deploy=放下方向 us，Retract=收起方向 us，Stop=1500µs（软件停舵）。
// 经 CompositeServoOutput 的某全局通道输出。
class GearServo : public ILandingGearActuator {
public:
    // out: 输出路由；ch: 全局输出通道；deploy_us/retract_us: 两个方向的速度指令。
    GearServo(CompositeServoOutput& out, int ch,
              uint16_t deploy_us = 2000, uint16_t retract_us = 1000)
        : out_(out), ch_(ch), deploy_us_(deploy_us), retract_us_(retract_us) {}
    void apply(GearDrive d) override;

private:
    CompositeServoOutput& out_;
    int      ch_;
    uint16_t deploy_us_, retract_us_;
};

// 类型 C：裸直流电机 + H 桥。PWM（全速 us）走 CompositeServoOutput，方向走 DIR GPIO。
// Stop：写 1500µs/占空比0 + DIR 任意；Deploy/Retract：全速 us + DIR 高/低。
class GearHbridge : public ILandingGearActuator {
public:
    GearHbridge(CompositeServoOutput& out, int pwm_ch, int dir_pin)
        : out_(out), pwm_ch_(pwm_ch), dir_pin_(dir_pin) {}
    void begin();   // pinMode(dir_pin, OUTPUT)
    void apply(GearDrive d) override;

private:
    CompositeServoOutput& out_;
    int pwm_ch_, dir_pin_;
};

}  // namespace wp
#endif  // WP_HAS_LANDING_GEAR
```

- [ ] **Step 4: 写执行机构实现**

`hal_esp32/src/drivers/gear_actuators.cpp`：

```cpp
#include "drivers/gear_actuators.h"
#if WP_HAS_LANDING_GEAR
#include <Arduino.h>

namespace wp {

void GearServo::apply(GearDrive d) {
    switch (d) {
        case GearDrive::Deploy:  out_.writeUs(ch_, deploy_us_);  break;
        case GearDrive::Retract: out_.writeUs(ch_, retract_us_); break;
        case GearDrive::Stop:
        default:                 out_.writeUs(ch_, 1500);        break;  // 软件停舵
    }
}

void GearHbridge::begin() {
    pinMode(dir_pin_, OUTPUT);
    digitalWrite(dir_pin_, LOW);
}

void GearHbridge::apply(GearDrive d) {
    switch (d) {
        case GearDrive::Deploy:
            digitalWrite(dir_pin_, HIGH);
            out_.writeUs(pwm_ch_, 2000);   // 全速
            break;
        case GearDrive::Retract:
            digitalWrite(dir_pin_, LOW);
            out_.writeUs(pwm_ch_, 2000);   // 全速（反向由 DIR 决定）
            break;
        case GearDrive::Stop:
        default:
            out_.writeUs(pwm_ch_, 1000);   // 占空比最小，电机停
            break;
    }
}

}  // namespace wp
#endif  // WP_HAS_LANDING_GEAR
```

> 注：H 桥"全速"用 2000µs 仅作占位语义；实际 H 桥 PWM 占空比映射需按驱动 IC（DRV8871 PWM 输入）实物标定。类型 C 标注"确有裸电机需求再启用"（设计文档 §8 步骤 7）。

- [ ] **Step 5: 板载编译检查（功能关）**

Run: `pio run -e weekendpilot_s3 2>&1 | tail -5`
Expected: SUCCESS（`WP_HAS_LANDING_GEAR=0`，全部 .cpp 被 `#if` 包空）。

- [ ] **Step 6: 板载编译检查（开门控）**

Run: `PLATFORMIO_BUILD_FLAGS="-DWP_HAS_LANDING_GEAR=1 -DWP_HAS_PWM_EXPANDER=1" pio run -e weekendpilot_s3 2>&1 | tail -8`
Expected: SUCCESS（INA3221 + 两执行机构编入）。

- [ ] **Step 7: Commit**

```bash
git add hal_esp32/src/drivers/ina3221.h hal_esp32/src/drivers/ina3221.cpp \
        hal_esp32/src/drivers/gear_actuators.h hal_esp32/src/drivers/gear_actuators.cpp
git commit -m "feat(hal): INA3221 电流驱动 + GearServo/GearHbridge 执行机构（门控 WP_HAS_LANDING_GEAR）"
```

---

## Task 10: ControllerConfig 加 LandingGearConfig + NVS version bump

把 `LandingGearConfig` 与上电恢复用的 `gear_last_state` 加进 `ControllerConfig`，升 NVS 版本号，更新静态守卫。core 件，影响序列化布局。

**Files:**
- Modify: `core/controller.h`
- Modify: `core/config/config_store.h`
- Test: `test/test_config_store/test_config_store.cpp`（加一条往返测试）

- [ ] **Step 1: controller.h 引入 landing_gear.h 并加字段**

`core/controller.h` 顶部 include 区（约 line 11 `blackbox.h` 后）加：

```cpp
#include "nav/landing_gear.h"
```

`ControllerConfig` 末尾（`BlackboxConfig blackbox;` 后，约 line 46）加：

```cpp
    LandingGearConfig landing_gear;     // 起落架堵转检测（默认 enabled=false）
    uint8_t gear_last_state = 0;        // 上电恢复：GearState 序号（0=Retracted）
```

> 检查：`LandingGearConfig` 是 POD（全 bool/float/uint，无指针/虚函数），不破坏 `is_trivially_copyable` 守卫。`GearState` 不直接进 struct（用 uint8_t 存序号），避免 enum 底层类型变更风险。

- [ ] **Step 2: 升 NVS 版本号**

`core/config/config_store.h` 约 line 11：

```cpp
constexpr uint8_t  kConfigVersion = 2;   // was 1: 加 LandingGearConfig + gear_last_state
```

- [ ] **Step 3: 加配置往返测试**

`test/test_config_store/test_config_store.cpp` 加一条测试（在 main() 前），并在 main() 注册：

```cpp
// 新增字段往返：landing_gear + gear_last_state 序列化后能原样读回
void test_landing_gear_config_roundtrip() {
    wp::ControllerConfig in;
    in.landing_gear.enabled = true;
    in.landing_gear.stall_current_a = 1.5f;
    in.landing_gear.stall_debounce_ms = 120;
    in.gear_last_state = 2;   // Deployed
    uint8_t buf[wp::kConfigBlobSize];
    uint16_t n = wp::serializeConfig(in, buf, sizeof(buf));
    TEST_ASSERT_TRUE(n > 0);
    wp::ControllerConfig out;
    TEST_ASSERT_TRUE(wp::deserializeConfig(buf, n, out));
    TEST_ASSERT_TRUE(out.landing_gear.enabled);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 1.5f, out.landing_gear.stall_current_a);
    TEST_ASSERT_EQUAL_UINT16(120, out.landing_gear.stall_debounce_ms);
    TEST_ASSERT_EQUAL_UINT8(2, out.gear_last_state);
}
```

在 `main()` 的 `UNITY_BEGIN()` 后加：

```cpp
    RUN_TEST(test_landing_gear_config_roundtrip);
```

- [ ] **Step 4: 跑测试确认通过（含静态守卫）**

Run: `cmake --build build 2>&1 | tail -8`
Expected: 编译通过——`static_assert(sizeof(ControllerConfig) <= 255)` 与 `is_trivially_copyable` 仍满足（新增约 12 字节，远未触 255 上限；当前 136 字节）。

> 若 `sizeof` 超 255：按 config_store.cpp 注释把 size 字段扩 uint16 + 再升 version。本次新增很小，预期不会触发。

Run: `ctest --test-dir build -R test_config_store --output-on-failure`
Expected: PASS（含新往返测试）。

- [ ] **Step 5: 全量回归**

Run: `ctest --test-dir build --output-on-failure 2>&1 | tail -5`
Expected: 全过（25 套，含 config_store 内新增条目）。

- [ ] **Step 6: Commit**

```bash
git add core/controller.h core/config/config_store.h test/test_config_store/
git commit -m "feat(config): ControllerConfig 加 LandingGearConfig + gear_last_state，NVS version 1->2"
```

---

## Task 11: main.cpp 起落架编排（HAL 主循环）

在 HAL 主循环编排起落架（不进 Controller::update，见关键决策 #3）：读收放通道 + 读 INA3221 电流 → 喂 LandingGear → 经执行机构驱动。全程门控 `WP_HAS_LANDING_GEAR`。

**Files:**
- Modify: `hal_esp32/src/main.cpp`

- [ ] **Step 1: include 与全局对象（门控）**

`main.cpp` include 区加：

```cpp
#if WP_HAS_LANDING_GEAR
#include "nav/landing_gear.h"
#include "drivers/ina3221.h"
#include "drivers/gear_actuators.h"
#endif
```

全局对象区加：

```cpp
#if WP_HAS_LANDING_GEAR
static wp::LandingGear g_gear;
static wp::Ina3221 g_ina(Wire, wp::kAddrCurrentSense);
// 起落架输出通道（全局逻辑索引）：开 PCA9685 时用第一个扩展口（=kNumServos）；
// 没开扩展时退回 LEDC 空闲口 7（标准布局 0~3 主舵面，7 空闲）。
#if WP_HAS_PWM_EXPANDER
constexpr int kGearOutCh = wp::kNumServos;   // PCA9685 的 0 号口
#else
constexpr int kGearOutCh = 7;                // LEDC 空闲口
#endif
static wp::GearServo g_gear_act(g_out, kGearOutCh);
constexpr int    kGearCurrentCh = 0;       // INA3221 通道 0
constexpr float  kGearShuntOhm  = 0.01f;   // 采样电阻（实物标定）
constexpr uint8_t kGearRcChannel = 8;      // ch9：收放拨杆（0-based 索引 8）
#endif
```

- [ ] **Step 2: setup() 内初始化（门控）**

`setup()` 内 `g_out.begin();` 之后加：

```cpp
#if WP_HAS_LANDING_GEAR
    {
        wp::LandingGearConfig gc;   // 默认 enabled=false（全写好先不使能）
        g_gear.setConfig(gc);
        g_ina.begin();
        // 上电不主动驱动：状态从默认 Retracted 起（将来从 NVS gear_last_state 恢复）。
        pinMode(wp::kPinGearAlert, INPUT_PULLUP);
    }
#endif
```

- [ ] **Step 3: loop() 内编排（门控）**

`loop()` 内主舵输出 `for (... g_out.writeUs ...)` 之后加：

```cpp
#if WP_HAS_LANDING_GEAR
    {
        wp::LandingGearInputs gi;
        // 收放指令：拨杆 > 1500µs 视为"放下"。link 丢失时维持上一指令（不强制收放）。
        gi.deploy_cmd = link_ok ? (g_channels[kGearRcChannel] > 1500) : g_gear.state() == wp::GearState::Deployed;
        float amps = 0.0f;
        if (g_ina.readCurrent(kGearCurrentCh, kGearShuntOhm, amps)) gi.current_a = amps;
        gi.alert = (digitalRead(wp::kPinGearAlert) == LOW);   // INA3221 ALERT 低有效
        gi.dt = 0.002f;   // 与 loop delay(2) 一致量级
        wp::LandingGearOutput go = g_gear.update(gi);
        g_gear_act.apply(go.drive);
    }
#endif
```

> 注：`deploy_cmd` 在 link 丢失时保持当前到位状态（不误动）。这是 failsafe 取舍——起落架在链路丢失时不应自行收放。

- [ ] **Step 4: 板载编译检查（功能关）**

Run: `pio run -e weekendpilot_s3 2>&1 | tail -5`
Expected: SUCCESS（起落架块全部 `#if` 包空，main 行为同 Task 6）。

- [ ] **Step 5: 板载编译检查（全开门控）**

Run: `PLATFORMIO_BUILD_FLAGS="-DWP_HAS_LANDING_GEAR=1 -DWP_HAS_PWM_EXPANDER=1" pio run -e weekendpilot_s3 2>&1 | tail -8`
Expected: SUCCESS（起落架编排 + INA3221 + GearServo + PCA9685 全编入，无未定义符号）。

- [ ] **Step 6: Commit**

```bash
git add hal_esp32/src/main.cpp
git commit -m "feat(hal): main.cpp 起落架编排（INA3221 电流->LandingGear状态机->GearServo，门控）"
```

---

## Task 12: 文档与进度台账更新

更新设计文档状态 + 记忆台账，标注实物待整定项。

**Files:**
- Modify: `docs/superpowers/specs/2026-06-01-pwm-output-landing-gear-design.md`
- Modify: 记忆 `weekendpilot-dev-progress.md`（经 Write 工具）

- [ ] **Step 1: 设计文档状态改"已实现"**

把设计文档头部 `- 状态：设计待评审` 改为 `- 状态：已实现（PC 测试全绿 + 板载门控编译通过）`。

- [ ] **Step 2: 更新进度台账记忆**

在 `weekendpilot-dev-progress.md` 的"已完成"区追加一段（仿现有条目风格），记录：多路 PWM 输出层（IServoOutput/CompositeServoOutput/PCA9685）+ 起落架两层（PeripheralMap 直通 + INA3221/LandingGear 状态机）已落地、门控默认关、单测数增量、⚠️ 堵转电流阈值/H桥占空比/上电默认态须实物整定。

- [ ] **Step 3: Commit**

```bash
git add docs/superpowers/specs/2026-06-01-pwm-output-landing-gear-design.md
git commit -m "docs: PWM输出+起落架设计文档标记已实现"
```

---

## 完成标准（Definition of Done）

- [ ] PC ctest 全绿：原 22 套 + 新增 3 套（composite_servo_output / ina3221_decode / landing_gear）+ config_store 内新增往返 = 25 套。
- [ ] 板载 `pio run -e weekendpilot_s3` 默认门控 SUCCESS（功能关，行为同基线）。
- [ ] 板载全开门控（`-DWP_HAS_LANDING_GEAR=1 -DWP_HAS_PWM_EXPANDER=1`）SUCCESS。
- [ ] 前 8 路 LEDC 输出行为与改造前一致（CompositeServoOutput 透明封装）。
- [ ] 起落架与 PWM 扩展默认 `enabled=false`/门控关，符合"全写好先不使能"。

## 实物待整定（⚠️ 本计划不覆盖，须硬件到位后做）

- INA3221 shunt 电阻值 `kGearShuntOhm` 与堵转电流阈值 `stall_current_a` 须按实际起落架电机标定。
- PCA9685 输出示波器核对 50Hz/脉宽；INA3221 已知负载电流标定。
- 类型 C（H 桥）PWM 占空比→转速映射须按 DRV8871/TB6612 实物调（当前为占位语义）。
- 上电默认起落架状态：建议从 NVS `gear_last_state` 恢复且不主动驱动；须实物验证拨杆上电初值不误触发。
- INA3221 ALERT 引脚 `kPinGearAlert=41` 与 PCA9685/其他外设引脚的实物冲突核对。
