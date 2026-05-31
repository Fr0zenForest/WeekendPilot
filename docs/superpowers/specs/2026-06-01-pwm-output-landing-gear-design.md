# 多路 PWM 输出扩展 + 起落架控制 — 设计文档

- 日期：2026-06-01
- 状态：设计待评审
- 关联：[[weekendpilot-config-sensor-architecture]]、[[weekendpilot-dev-progress]]、主设计 `docs/superpowers/specs/2026-05-30-weekendpilot-design.md`
- 阶段定位：阶段 3 外设扩展（与 Alt-Hold / Auto-trim 同级，均为"全写好先不使能"门控功能）

## 0. 背景与认知前提

两个独立但硬件上耦合的功能（共用主 I2C 总线、共用舵机供电）：

1. **多路 PWM 输出扩展**：突破 ESP32-S3 原生 8 路 PWM 限制。
2. **起落架控制**：支持现成控制器（一路 PWM 直通）+ 自管堵转检测（3 路电流检测）。

### 必须先澄清的两个认知点

- **输出路数 ≠ ELRS 通道数**。ELRS/CRSF 空口固定 16 通道 11-bit（`main.cpp:67` 解析 `0x16` 帧即此），那是*遥控输入*。飞控混控后的*舵机输出*路数完全独立，不受 16 限制。
- **ESP32-S3 原生只有 8 路 LEDC PWM**（非初代 ESP32 的 16 路）。当前 `kServoPins`（`main.cpp:37`、`board_config.h:41`）正好用满 8 路。稳超 8 路必须外挂扩展芯片。

### 设计总纲

按"输出层/驱动层抽象 + 路数可配 + 全写好先不使能"落地，与项目既有工作法一致：
- core 定义纯算法/纯抽象接口，PC 全测；
- HAL 写具体芯片驱动，编译期 `WP_HAS_*` 宏 + 运行时 probe 双层门控；
- 没焊扩展芯片时自动退回原生 LEDC，core 零改动。

## 1. 确认的设计决策（来自需求澄清）

| # | 决策点 | 选定 |
|---|---|---|
| D1 | PWM 输出规模 | 不定死，做成可扩展输出层抽象 |
| D2 | 舵机刷新频率 | 做成可配，默认 50Hz |
| D3 | PWM 扩展芯片首选 | PCA9685（I2C，16 路/颗，0 额外 GPIO）；原生 LEDC 保留给主舵面高频片 |
| D4 | RP2040 协处理器 | 暂不做（YAGNI），只在 IServoOutput 抽象上留口子 |
| D5 | 起落架层 1 | 现成控制器，复用已有 PeripheralMap，输出一路 PWM |
| D6 | 起落架层 2 选型 | INA3221（I2C，3 路电流检测，硬件 ALERT 中断，0 额外 GPIO） |
| D7 | 堵转后停止方式 | 软件停舵（不加高边开关 MOS，不真断电） |
| D8 | 起落架执行机构 | B（连续旋转舵机/PWM）+ C（裸电机/H 桥）都支持，驱动可插拔 |


## 2. 方向一：多路 PWM 输出扩展

### 2.1 方案对比（已评审通过）

**方案 A — PCA9685（选定主路径）**
- I2C 接口，复用现有 `kPinI2cSda=18 / kPinI2cScl=39`，**0 个额外 GPIO**。
- 地址 A0~A5 共 6 位可选 → 最多 62 颗级联（992 路），级联仅改地址，仍占两根线。
- 频率全局可调 24~1526Hz（寄存器设定），契合 D2"可配默认 50Hz"。
- **限制：全局单一频率**，同一颗芯片 16 路必须同频，无法芯片内混 50Hz 舵机与 333Hz 数字舵机。
- 分辨率 12-bit @ 50Hz ≈ 4.07µs/步，对舵机够用（舵机机械分辨率约 1µs 已极限）。
- 封装 TSSOP-28（约 9.7×4.4mm），外围一颗去耦 + I2C 上拉；模块版自带 V+ 舵机供电端子。

**方案 B — RP2040 协处理器（暂不做，留抽象）**
- Pixhawk/ArduPilot 的 IOMCU 标准架构：主控不亲自打 PWM，外挂协处理器专管 RC IO + PWM，走串口通信。
- RP2040 PIO 可软件生成 16+ 路*各自独立频率*的精确 PWM，彻底解决混频，且不占主控实时性。
- 代价大：第二颗 MCU 固件 + 自定义串口协议 + 占 1 UART + PCB 翻倍复杂度。对当前阶段与"紧凑布局"是过度设计。
- **决策：YAGNI，仅在 IServoOutput 抽象上留实现位，将来想上只是再加一个驱动实现，core 零改动。**

**方案 C — 纯原生 LEDC 8 路（降级档）**
- 即现状。作为"≤8 路 / 未焊扩展芯片"时编译期关掉扩展芯片的回退路径保留。

### 2.2 混频策略

PCA9685 全局单频率，故主舵面（需高刷新/高分辨率）与慢速面分片处理：
- **原生 LEDC（高频片）**：留给主舵面（roll/pitch/yaw）+ 油门 ESC，保住 16-bit 分辨率与可独立调高频能力。
- **PCA9685（50Hz 片）**：慢速面（襟翼）、起落架、灯等外设。

### 2.3 频率/控制环关系（回答"更新频率是多少"）

- 模拟舵机 PWM 输出频率 = 50Hz（周期 20ms），即 PCA9685 全局频率寄存器设定值。
- 主控制环频率 ≈ 500Hz（`main.cpp` 标称 dt=0.001，实测 ~500Hz，见进度台账遗留项）。
- 二者解耦：控制环高速算出舵量，PWM 输出按各自频率取最新值刷新。

### 2.4 软件设计：IServoOutput 抽象层

新增 core 抽象，把"如何把 us 值送到物理引脚"从 `main.cpp` 直接 `ledcWrite` 中解耦：

```cpp
// core/io/servo_output.h（新增，纯接口，零硬件依赖）
namespace wp {
class IServoOutput {
public:
    virtual ~IServoOutput() = default;
    virtual bool begin() = 0;                       // 初始化，返回是否成功（probe）
    virtual int  channelCount() const = 0;          // 本后端提供多少路
    virtual void setFrequencyHz(uint16_t hz) = 0;   // D2 可配频率（后端不支持则忽略/夹取）
    virtual void writeUs(int ch, uint16_t us) = 0;   // 写第 ch 路（us，内部夹 [1000,2000]）
};
}  // namespace wp
```

实现（HAL 侧，门控编译）：
- `hal_esp32/src/io/ledc_output.{h,cpp}`：包装现有 LEDC 8 路逻辑（从 `main.cpp` 抽出 `setupPwm`/`writeServoUs`）。无条件可用。
- `hal_esp32/src/io/pca9685_output.{h,cpp}`：I2C 驱动，门控 `WP_HAS_PWM_EXPANDER`。寄存器实现参考成熟库（RobTillaart/PCA9685_RT 或移植，遵循 [[weekendpilot-port-mature-fc]] 标注 provenance）。

聚合：`CompositeServoOutput`（core，纯逻辑可测）——把"逻辑输出索引 [0, N)"路由到多个后端。例：索引 0~7 → LEDC，8~23 → PCA9685。N 在编译期由可用后端之和决定。
- probe 失败（未焊 PCA9685）→ 自动只暴露 LEDC 的 8 路，core 上层照常工作。

### 2.5 路数可配（D1）

- `core/types.h` 的 `kNumServos` 当前硬编码 8。**不直接改它**（牵动 mixer/blackbox/controller 全链 struct 尺寸与 NVS 序列化）。
- 改为：保持 mixer/主舵面在 `kNumServos`（≤8）域内不变；扩展输出口（AUX/外设/起落架）通过 `CompositeServoOutput` 的逻辑索引 ≥ kNumServos 访问，由 `PeripheralMap` 与起落架模块显式路由。
- 即：**核心增稳混控仍 8 路；扩展路定位为"外设/慢速面"路由目标**，避免动 NVS 布局。后续若真需要 >8 路参与混控，另开 struct 版本迁移任务（本设计不含）。


## 3. 方向二：起落架控制

### 3.1 层 1：现成电动起落架控制器（最简，先做）

市售"电动起落架 sequencer"自带堵转/限位逻辑，飞控只需输出一路标准 PWM：1000µs=收 / 2000µs=放（或反）。

- **信号通信**：一路 PWM 输出口，复用 §2.4 的 `IServoOutput`。
- **本质已存在**：`controller.h:18` 的 `PeripheralMap`（RC 通道裸 us 直通到某 servo 口，`controller.cpp:111-115` 已实现）即此功能。只需把某路 PWM 接到起落架控制器，把一个 RC 通道（如拨杆 ch9）映射过去。
- **硬件成本**：0。需做的仅是确认 `PeripheralMap` 能路由到扩展芯片输出口（§2.4 输出层做好后自然支持）。

### 3.2 层 2：飞控自管堵转检测（3 路电流检测）

#### 3.2.1 "反转舵机"的本质 — 执行机构差异

舵机无"反转模式"开关，"反转"只是发不同 PWM 值；具体取决于执行机构（D8 选定支持 B+C）：

| 类型 | 怎么"反转" | 要电流检测 | 额外硬件 |
|---|---|---|---|
| A 位置舵机 | 发 1000 vs 2000µs（自带限位到位自停） | 不需要 | 无（不在本层范围，归层 1/直通） |
| **B 连续旋转舵机** | PWM 跨 1500µs 中点：1000=收/2000=放/1500=停 | **需要**（无位置反馈，靠电流判到头） | 无（一根 PWM 线） |
| **C 裸直流齿轮电机** | 翻转 H 桥两个方向脚电平 | 需要 | H 桥 IC（DRV8871/TB6612）+ 2 方向脚 |

层 2 同时支持 B、C，由可插拔的 `ILandingGearActuator` 抽象隔离驱动差异。

#### 3.2.2 选型：INA3221（3 路一颗）

[TI INA3221](https://www.ti.com/product/INA3221)：3 通道高边电流+电压监测，I2C，13-bit。
- **0 个额外 GPIO**（挂主 I2C）。地址 0x40~0x43 可选，与 IMU 0x68 / Baro 0x76 / Mag 0x0E / Airspeed 0x28 不撞。
- **硬件 Critical-Alert 门限比较器**：每路设电流阈值，超限芯片自拉 ALERT 引脚（无需 ESP32 持续轮询 I2C）→ 堵转检测最干净的实现。ALERT 接一个 ESP32 空闲 GPIO 触发中断。
- 替代权衡：INA226（单路 16-bit 更准，3 路需 3 颗）；ACS712（霍尔，占 ADC，精度差）。3 路场景 INA3221 一颗最紧凑。
- ⚠️ INA3221 高边采样，单路连续电流受 shunt 与封装限制；起落架舵机堵转电流通常 1~3A，选合适 shunt 即可。大电流裸电机需校核 shunt 功率与走线。

#### 3.2.3 电路设计（高边采样 + shunt）

```
舵机/电机电源 V+ ──[shunt Rs]──┬── 起落架执行机构
                    IN+   IN-  │
                     └──INA3221─┘  (测 Rs 压降 → 电流)
                        │ ALERT ──→ ESP32 GPIO（堵转中断）
                        │ SDA/SCL ─→ 复用主 I2C 总线
```
- 每路一个 shunt（典型 0.01~0.05Ω）串在该路供电正极；INA3221 测压降算电流。
- ALERT 接 ESP32 空闲 GPIO（候选见 §5）。

#### 3.2.4 供电 / 停止方式（D7 = 软件停舵）

- INA3221 只**测**电流，不切断。D7 选"软件停舵"：堵转后 core 状态机停发驱动 PWM（连续旋转舵机收 1500µs 停转 / H 桥置零），舵机松力，**不加高边开关 MOS、不真断电、不占断电 GPIO**。
- 预留：未来如需严格断电保护，可在供电路径加高边负载开关（如 TPS22965）+ 1 个 GPIO，core 状态机已把"停止"抽象为一个动作，加断电只是给该动作多挂一个 HAL 副作用，逻辑不改。

#### 3.2.5 软件设计：core 状态机 + 可插拔驱动

```cpp
// core/nav/landing_gear.h（新增，纯逻辑，PC 可单测）
namespace wp {
enum class GearState : uint8_t { Retracted, Deploying, Deployed, Retracting, Fault };

struct LandingGearConfig {
    bool     enabled = false;        // 总开关（默认关，全写好先不使能）
    uint8_t  cmd_channel = 8;        // ch9：拨杆控制收/放
    float    stall_current_a = 2.0f; // 堵转判定电流阈值（A）
    uint16_t stall_debounce_ms = 80; // 电流超阈持续多久判定到位（去抖）
    uint16_t timeout_ms = 4000;      // 行程超时无堵转 → Fault（防烧）
};

struct LandingGearInputs {
    bool  deploy_cmd;     // 飞手指令：true=放下 false=收起（由 cmd_channel 判定）
    float current_a;      // 本路电流采样（来自 INA3221）
    bool  alert;          // INA3221 硬件 ALERT（可选，作为快速路径）
    float dt;
};

struct LandingGearOutput {
    int16_t drive;        // 驱动指令：连续旋转舵机 us（1000/1500/2000）或 H 桥方向枚举
    GearState state;
};

class LandingGear {
public:
    void setConfig(const LandingGearConfig&);
    LandingGearOutput update(const LandingGearInputs&);
    GearState state() const;
private:
    // 状态机：Retracted →(放下)→ Deploying →(堵转去抖)→ Deployed
    //                   →(收起)→ Retracting →(堵转)→ Retracted
    //         Deploying/Retracting 超 timeout 无堵转 → Fault
};
}  // namespace wp
```

可插拔驱动（HAL，门控 `WP_HAS_LANDING_GEAR`）：
```cpp
// core/nav/landing_gear.h 内或同目录：驱动抽象（core 定义接口）
class ILandingGearActuator {
public:
    virtual ~ILandingGearActuator() = default;
    virtual void drive(int16_t cmd) = 0;  // B：写连续旋转舵机 us；C：翻 H 桥方向脚
    virtual void stop() = 0;              // B：写 1500us；C：H 桥置零（软件停舵）
};
```
- `hal_esp32/src/drivers/gear_servo.{h,cpp}`（类型 B，走 IServoOutput 一路 PWM）
- `hal_esp32/src/drivers/gear_hbridge.{h,cpp}`（类型 C，DRV8871/TB6612，占方向 GPIO）
- `hal_esp32/src/drivers/ina3221.{h,cpp}`（IGearCurrentSense，门控 `WP_HAS_LANDING_GEAR`）

状态持久化：`GearState`（已展开/已收起）写入 ControllerConfig 随 NVS 存（上电恢复"上次起落架状态"，避免上电误动）。⚠️ 上电默认安全态策略见附录。


## 4. 数据流

```
RC 输入(CRSF 16ch) ──┐
                     ├─→ Controller.update() ──→ mixer 8 路主舵量 ─┐
传感器(SensorBundle)─┘         │                                  │
                               │  PeripheralMap（层1 起落架直通）  │
                               │  LandingGear 状态机（层2）        │
                               ▼                                  ▼
                    ServoCommand + 扩展路指令 ──→ CompositeServoOutput
                                                    ├─→ LedcOutput（0~7，主舵面/油门，高频片）
                                                    └─→ Pca9685Output（8~N，慢速面/外设/起落架，50Hz 片）

INA3221（3 路电流）─→ poll/ALERT ─→ LandingGearInputs.current_a/alert ─→ LandingGear 状态机
```

起落架层 2 在 Controller 中的接入点：紧随 §`controller.cpp:111` 外设直通之后，作为更高优先级的输出覆盖（起落架占用的输出口由状态机指令覆盖，而非裸 RC 直通）。

## 5. 引脚与总线占用汇总

| 资源 | 现状 | 本设计新增 |
|---|---|---|
| I2C（SDA18/SCL39） | IMU/Baro/Mag/Airspeed | +PCA9685（地址可选）、+INA3221（0x40~0x43）。0 额外 GPIO |
| LEDC PWM | GPIO 1,2,8,9,10,15,16,17（8 路） | 保留为主舵面/油门高频片 |
| PCA9685 输出 | — | 16 路 OUT（芯片侧引脚，不占 ESP32 GPIO） |
| INA3221 ALERT | — | 1 个 ESP32 空闲 GPIO（候选：GPIO40 当前仅声明 kPinImuInt 未用 / 或另选自由脚，实物核对） |
| H 桥方向脚（仅类型 C） | — | 每电机 1~2 个 GPIO 或经 I2C IO 扩展（仅当选 C 时） |

board_config.h 新增常量（门控）：`kAddrPwmExpander`、`kAddrCurrentSense`、`kPinGearAlert`、（C 用）H 桥方向脚。

## 6. 门控与默认值（全写好先不使能）

| 宏 | 默认 | 含义 |
|---|---|---|
| `WP_HAS_PWM_EXPANDER` | 0（板级）/ 1（PC 测试） | 编译 PCA9685 驱动 |
| `WP_HAS_LANDING_GEAR` | 0（板级）/ 1（PC 测试） | 编译 INA3221 + 起落架驱动 |

- 运行期再加 probe：PCA9685/INA3221 I2C 探测不到 → 自动降级（PCA9685 缺 → 只用 LEDC；INA3221 缺 → 起落架层 2 禁用，层 1 直通仍可用）。
- ControllerConfig 新增 `LandingGearConfig`（默认 enabled=false）。沿用 CRC16 POD 序列化，需更新 config version 并校验 `sizeof(ControllerConfig)` 静态守卫。

## 7. 测试策略

**core（PC ctest，沿用 Unity）**
- `test_landing_gear`：状态机全路径——Retracted→Deploying→(堵转去抖)→Deployed→Retracting→Retracted；timeout→Fault；去抖边界（current 抖动不误判）；enabled=false 不动作。
- `test_composite_servo_output`：逻辑索引→后端路由正确；后端缺失时索引收缩；频率设定转发。
- INA3221/PCA9685 寄存器编解码若有纯逻辑部分（如电流寄存器值→安培换算），拆 `core/sensors/ina3221_decode.*` PC 测（沿用 ist8310/ms4525 套路）。

**HAL（pio 编译检查 + 实物）**
- `pio run -e weekendpilot_s3` 各门控组合编译通过（开/关 WP_HAS_PWM_EXPANDER、WP_HAS_LANDING_GEAR）。
- 实物：PCA9685 输出示波器核对 50Hz/脉宽；INA3221 已知负载电流标定；起落架堵转电流实测定阈值（⚠️ 阈值是机型相关经验值，须实物整定）。

## 8. 实施顺序（建议）

1. `IServoOutput` 抽象 + `LedcOutput`（从 main.cpp 抽出，行为不变，回归现有 8 路）。
2. `CompositeServoOutput` + core 单测。
3. `Pca9685Output` 驱动 + 门控 + pio 编译。
4. 起落架层 1：确认 PeripheralMap 路由到扩展口（多为验证 + 文档）。
5. `LandingGear` core 状态机 + `ILandingGearActuator` 抽象 + core 单测。
6. `ina3221_decode`（core）+ `Ina3221` 驱动 + `gear_servo`（类型 B）。
7. 类型 C（H 桥 `gear_hbridge`）—— 仅当确有裸电机需求时再做。
8. ControllerConfig 接线 + NVS version bump + 静态守卫更新。

每步沿用：设计→writing-plans→subagent 开发（Sonnet 写 + Opus 双段 review）→本地合并+建 PR。

## 9. 已知限制与待整定（⚠️）

- 堵转电流阈值 `stall_current_a` 为机型相关经验值，须实物整定，SITL 无法验证（无电流物理量）。
- INA3221 单路连续电流上限受 shunt/封装约束；大扭矩裸电机起落架需校核 shunt 功率。
- 上电默认起落架状态策略：建议上电保持"上次 NVS 记录态"且**不主动驱动**（避免上电瞬间误收放砸伤）；仅在收到明确拨杆跳变后动作。须实物验证拨杆上电初值不误触发。
- PCA9685 全局单频率；若未来需芯片内混频，需第二颗 PCA9685 或上 RP2040（本设计不含）。
- 路数 >8 参与*混控*需迁移 kNumServos 与 NVS 布局，本设计明确不含（扩展路仅作外设/慢速面路由目标）。

## 附录：与现有代码的衔接点

- `main.cpp:49-61` setupPwm/writeServoUs → 迁入 `LedcOutput`。
- `controller.cpp:108-115` mixer + PeripheralMap → 之后插入起落架状态机输出覆盖。
- `controller.h:24-47` ControllerConfig → 新增 LandingGearConfig 字段 + version bump。
- `board_config.h` → 新增扩展芯片地址/ALERT 脚常量（门控）。
- `capabilities.h` → 新增 WP_HAS_PWM_EXPANDER / WP_HAS_LANDING_GEAR + constexpr 镜像。
