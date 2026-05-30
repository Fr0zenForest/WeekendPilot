# WeekendPilot 飞控 — 总体设计文档（从 0 重构）

> **状态：** 设计稿，待你 review。本文是从 `docs/需求.md` 出发、结合 ElrsRX 已有过程文档与 git 历史，对"weekend 飞控"项目做的一次彻底重新构思。
> 
> **日期：** 2026-05-30 ｜ **作者：** 与 Claude 协作 ｜ **取代：** ElrsRX 全部历史（本项目将废弃）

---

## 0. 一句话定位

**WeekendPilot 是一台跑在 ESP32-S3 上的、自己写固件的固定翼伴侣飞控。** 它从 Betafpv SuperX Nano 接收机的串口拿遥控信道，自己跑姿态解算 + 增稳 + 混控 + 选择性自动驾驶，再输出 PWM 给舵机和电调。算法不自己抠参数，而是从 INAV / ArduPlane / Betaflight 等成熟飞控**移植成熟逻辑**；先在 JSBSim + FlightGear 仿真里调好，再上实机。

仓库英文名建议：**`WeekendPilot`**（slug: `weekend-pilot`）。固件启动横幅仍可打印"weekend 飞控"。备选：`WeekendWing` / `Skylark`。

---

## 1. 为什么推倒重来（与 ElrsRX 的本质区别）

| 维度    | 旧 ElrsRX                           | 新 WeekendPilot                    |
| ----- | ---------------------------------- | --------------------------------- |
| 本质    | ExpressLRS 接收机固件的 fork，**带增稳的接收机** | **独立伴侣飞控**，不碰 RF                  |
| RF 收发 | 自己驱动 LR1121（SPI）                   | 交给 SuperX Nano，串口拿 CRSF           |
| 代码基座  | ELRS device 框架，背着整个 ELRS 包裹        | 自己的两层架构，零 ELRS 依赖                 |
| 升级痛点  | 跟上游 merge、LR1121 bring-up          | 无（不维护 RF 栈）                       |
| 测试    | 只能上板子 / PC 单测局部                    | **JSBSim SITL 闭环仿真** + PC 单测 + 实机 |

**结论：** SuperX Nano（ESP32-C3 + 双 LR1121）已经把"接收机"这件事做完了，自研接收机方向（Core1121 开发板）正式退役。新项目专注飞控算法本身——这正是 `docs/2026-05-25-fc-feature-concepts.md` 里"伴侣飞控"的演化方向。

---

## 2. 五个已锁定的关键决策

本设计前，通过 brainstorming 与你确认了 5 个会决定"买什么硬件、克隆哪些仓库、仓库里放什么"的决策：

1. **核心策略 = 自写固件 @ ESP32-S3**，复用 ElrsRX 软件栈思路，移植 INAV/ArduPlane 成熟算法（不直接跑它们——它们是 STM32-only）。
2. **目标机型 = 传统固定翼优先**，架构预留混控/控制律切换接口，以后装涵道/矢量机再做 3D 特技。
3. **仿真 = JSBSim + FlightGear** SITL 管道（同 ArduPlane）。调参时 JSBSim 无头跑只读数据，演示时挂 FlightGear 看 3D。
4. **代码迁移 = 只参考、代码重写**。不原样搬 ElrsRX 代码，只把算法选型、PID 默认值、轴向、状态机设计当参考，重写成干净两层架构。
5. **仓库名 = WeekendPilot**（待你最终拍板）。

---

## 3. 需求重述（把 `需求.md` 理顺、细化）

你原文写得有点乱，这里按"必须有 / 应该有 / 以后做"重新归类，并给每条补上**判据**和**传感器依赖**。编号对应你原文的 1~10。

### 3.1 核心控制（必须有，第一批）

| #   | 功能                | 细化定义                                               | 传感器依赖    |
| --- | ----------------- | -------------------------------------------------- | -------- |
| 1   | **增稳 Rate mode**  | 摇杆 = 目标角速率，松杆维持当前姿态。陀螺闭环。Yaw 永远 Rate。              | 陀螺       |
| 2   | **自稳 Angle mode** | 摇杆 = 目标倾角，松杆自动回水平。AHRS 角度闭环。                       | IMU+磁力计+AHRS（9轴，见附录 C） |
| —   | **直通 Off**        | 舵机直出，等同普通接收机。任何故障的回退态。                             | 无        |
| 7   | **感度调节**          | 飞行中用 AUX 旋钮 0~100% 连续调 PID 的 P（D）增益。空中调高速/低速段不同感度。 | 无        |

### 3.2 外设与混控（必须有，第一批）

| #   | 功能       | 细化定义                                                            |
| --- | -------- | --------------------------------------------------------------- |
| 3   | **外设控制** | 起落架、襟翼、灯等。开关通道直通到指定 PWM；襟翼可与升降舵联动（flaperon/flap-elevator mix）。  |
| —   | **舵机混控** | 副翼/升降/方向/油门标准布局；支持 flaperon、V-tail、elevon 等常见混控；为涵道/矢量预留通道分配接口。 |

### 3.3 安全网（应该有，第二批）

| #   | 功能                     | 细化定义                                    | 判据                       |
| --- | ---------------------- | --------------------------------------- | ------------------------ |
| 6   | **救命键 Panic Recovery** | 一键：姿态归零目标 + 油门归中 + 高度保持。退出：摇杆动了 / 开关复位。 | 开关触发                     |
| —   | **过载限制 G-limit**       | 拉/推杆超设定 G 时软化升降舵。软限 6~8G，硬限 10G。        | IMU 法向 G                 |
| 4   | **失速自动改出**             | 临近失速：震杆（油门注方波）+ 自动加油门 + 限制后拉升降舵。        | 空速管（精确）/ 低速大仰角+法向G<1（粗估） |

### 3.4 选择性自动驾驶（应该有，第三批）

| #   | 功能                    | 细化定义                                   | 传感器依赖                         |
| --- | --------------------- | -------------------------------------- | ----------------------------- |
| 8a  | **高度保持 Alt-Hold**     | 开关瞬间锁当前气压高度为 setpoint，油门+升降舵 PID 外环维持。 | BMP390                        |
| 8b  | **航向保持 Heading-Hold** | 锁当前航向，方向舵 PID 外环维持。                    | IMU 偏航（纯陀螺短期可用，长期需 mag 或 GPS） |
| 8c  | **空速保持**              | 锁目标空速，油门外环维持（类真机自动油门）。                 | pro：空速管（精确）；plus：GPS 地速退化（无风可用） |

### 3.5 机动辅助（应该有，第三批）

| #   | 功能                 | 细化定义                                               |
| --- | ------------------ | -------------------------------------------------- |
| 9   | **协调转弯**           | 横滚率 → 偏航率前馈 `r = g·tan(φ)/V`，叠加侧滑反馈让侧向加速度趋零。可单独开关。base 用名义空速常数；plus/pro 用 GPS 地速/真空速代入 V 更准（兼作 AHRS turn-comp，见附录 C.3）。 |
| 10  | **自动配平 Auto-trim** | 平飞时把稳态舵偏学进 trim，长期 I 项固化为配平基准。                     |

### 3.6 基础设施（应该有，建议提前做）

| #   | 功能                  | 细化定义                                                                                          |
| --- | ------------------- | --------------------------------------------------------------------------------------------- |
| 5   | **黑匣子 飞行日志**        | 250Hz（默认，固定翼够）全状态记录，复用 Betaflight Blackbox 格式（能用 Blackbox Explorer 打开）。**首发写内置 PSRAM 环形缓冲、WiFi 下载**，W25Q128/SD 作可选 sink。**建议提前做——调参提速 10 倍。** 详见 §12.1 |
| 6'  | **WiFi WebUI 实时调参** | 复用 ElrsRX 的 WebUI 思路，边飞边调 PID/混控/模式/下载黑匣子。S3 平台天然优势。                                          |

### 3.7 以后做（架构预留，不进第一版）

涵道/矢量推力混控、刀飞辅助、悬停辅助(prop-hang)、过失速机动、双发差速、进气道防喘振、GPS/RTH/航点（与"重驾驶乐趣"定位冲突，仅备查）。详见 `docs/2026-05-25-fc-feature-concepts.md`。

---

## 4. 架构（全篇最重要的一节）

整个设计成立的关键，是把代码**硬切成两层**，让同一份控制代码既能跑在板子上、又能跑进 JSBSim 仿真里。

```
┌───────────────── PORTABLE CONTROL CORE （纯 C++，零硬件依赖）─────────────────┐
│                                                                              │
│   ImuSample ─┐                                                               │
│   BaroSample ├─► AHRS(Mahony) ─► Mode/ControlLaw 状态机 ─► PID(angle/rate)    │
│   Airspeed ──┤                          │                      │             │
│   channels[16]                          ▼                      ▼             │
│   dt                              Autopilot 外环 ────────►   Mixer           │
│                                  (alt/hdg/spd hold)            │             │
│                                                               ▼              │
│                                              ServoCommand{ servo[N], thr }   │
│                                                                              │
│   特点：确定性、可单元测试、不依赖 Arduino / FreeRTOS / Wire / 任何 HAL       │
└──────────────────────────────────────────────────────────────────────────────┘
        ▲                                              ▲
        │ 由 板载硬件胶（ESP32-S3）喂                    │ 由 SITL 桥（PC）喂
        │                                              │
  ┌─────┴───────────────────────┐              ┌───────┴──────────────────────┐
  │ SuperX→UART：CRSF 解码        │              │ JSBSim socket：               │
  │ ICM-42688-P / BMP390 / pitot │              │   读飞机状态 → 填同样的 struct │
  │ PWM/ESC 输出                  │              │   写 ServoCommand → JSBSim    │
  │ WiFi WebUI 调参               │              │ FlightGear：挂 socket 看 3D   │
  │ 黑匣子 PSRAM(默认)/SD/flash   │              │ 调参时无头跑，演示时开动画     │
  └──────────────────────────────┘              └───────────────────────────────┘
```

### 4.1 控制核（`core/`）— 纯算法，两端共用

输入只有几个朴素 struct（`ImuSample`、`MagSample`、`BaroSample`、`AirspeedSample`、`GnssSample`、`channels[16]`、`dt`），输出一个 `ServoCommand`。它**不知道**自己跑在 ESP32 还是 PC 上，也**不知道**背后是 1 个还是 3 个传感器（冗余表决由 §4.2.1 的 `SensorFrontend` 在喂进来之前处理掉）。这是 ArduPilot SITL、Betaflight SITL 同款思路，也是你现有 `test/test_stab_mode_controller/` 已经在 PC 上编译核心代码的自然延伸。

控制核内部模块（每个一个清晰职责、独立可测）：

| 模块           | 职责                                           | 移植参考                    |
| ------------ | -------------------------------------------- | ----------------------- |
| `ahrs/`      | Mahony AHRS：base 档 9 轴（陀螺+加速度+磁力计）+ 加速度幅值门控（转弯中陀螺扛、机动时不信加速度，见附录 C），plus/pro 叠 turn-comp，输出欧拉角 | madflight (update9DOF) |
| `filter/`    | PT1 / biquad / notch 低通                      | Betaflight / iNav       |
| `pid/`       | PID（D 项用陀螺，ki 独立原始积分器，back-calc anti-windup） | madflight               |
| `modes/`     | Off/Angle/Rate/Horizon + 平滑过渡 + 滞回 状态机       | 自研（参考 ElrsRX 已验证逻辑）     |
| `mixer/`     | 通道→舵面分配，flaperon/V-tail/elevon，预留矢量/涵道       | INAV mixer              |
| `autopilot/` | alt/hdg/spd hold 外环、协调转弯、失速改出、auto-trim      | ArduPlane (TECS/L1 简化版) |
| `safety/`    | G-limit、Panic Recovery、failsafe 降级           | 自研                      |

### 4.2 硬件胶（`hal_esp32/`）— 板载特有

| 模块                                          | 职责                                                     |
| ------------------------------------------- | ------------------------------------------------------ |
| `crsf_in`                                   | 串口读 SuperX 的 CRSF，解出 16 通道 + LinkStatistics            |
| `sensors/`（见 §4.2.1）                        | 按类型抽接口的传感器驱动（gyro/accel/mag/baro/airspeed/gnss），探测在线即解锁对应档位；带 I2cLockGuard 跨核互斥 |
| `pwm_out`                                   | LEDC/RMT 输出舵机 PWM + 电调信号                               |
| `webui`                                     | WiFi AP + GET/POST /config 热加载 + 黑匣子下载                 |
| `blackbox`                                  | SD（SPI）或 W25Q128 flash，Betaflight Blackbox 格式          |
| `main`                                      | FreeRTOS 任务编排：core 1 跑 1kHz 控制核，core 0 跑 WiFi/遥测/日志    |

### 4.2.1 模块化传感器层（换型号只动一层 + max 表决冗余）

> 目标：测试期用便宜传感器，量产期换好的，**只写一个新驱动适配，`core/` 和上层零改动**。同时为 max 档的多传感器表决留好接口。

按**传感器类型**抽接口（纯虚基类，放 `hal_esp32/sensors/`），每个具体型号实现一个驱动。`core/` 只认接口产出的标准 Sample struct，不认型号：

| 接口            | 标准输出（喂 core 的 struct）         | 已知实现（测试期→量产期可换）                 |
| ------------- | ----------------------------- | --------------------------------- |
| `IGyroAccel`  | `ImuSample{gyro[3], accel[3]}`（rad/s, g） | ICM-42688-P（已有）/ MPU6050（便宜）/ BMI270 |
| `IMagnetometer` | `MagSample{mag[3]}`（归一化或 µT）   | QMC5883L（便宜）/ IST8310 / LIS3MDL    |
| `IBarometer`  | `BaroSample{press_pa, temp_c}` | BMP390（已有）/ BMP280 / SPL06        |
| `IAirspeed`   | `AirspeedSample{ias_mps}`      | MS4525DO / SDP3x（pro 档）           |
| `IGnss`       | `GnssSample{lat,lon,vel_ned[3],fix}` | u-blox NEO-M9N（plus 档，UBX/NMEA）  |

**统一驱动契约（每个实现都给这几个方法）：**
- `bool probe()` — 探测在不在（WHO_AM_I/CHIPID），决定该传感器是否存在 → 决定当前档位能力。
- `bool init()` / `bool read(Sample&)` — 初始化与单次读取，失败返回 false（计入健康状态）。
- `applyCalibration(...)` — 零偏/标度/硬铁软铁，按型号各自实现。

**自动探测 + 能力开关：** 开机扫所有已注册驱动，`probe()` 成功的进 active 列表 → 推出"当前档位"（探到 mag=base 起步，再探到 GPS=plus，再探到空速=pro）。core 据此开/关对应功能（无 GPS 就关 GPS-turn-comp，退回纯前馈）。这把 §6.0 的档位降级落到代码。

**max 档表决（`SensorFrontend` 聚合层，在 hal 与 core 之间）：** 当某类型有多个实例（3 陀螺/2 磁/2 气压），`SensorFrontend` 把它们的读数做中值/表决，剔除离群与失效（NaN/超量程/卡死不变），输出**单个**干净 Sample 给 core。**core 永远只看到一组 Sample**，完全不知道背后是 1 个还是 3 个传感器——冗余对 core 透明。base~pro 档该层退化为直通（一个传感器原样转发）。

> 分层落点：接口与驱动在 `hal_esp32/sensors/`（板载 I/O，平台相关）；`SensorFrontend` 表决逻辑是纯算法、无 I/O，**放 `core/sensors/`** 可被 SITL 复用和单测。SITL 桥（合成磁力计、JSBSim 喂数）实现同一组接口的 PC 版，见 §4.3 与附录 C.4。

### 4.3 SITL 桥（`hal_sitl/`）— PC 仿真特有

| 模块               | 职责                                                                                     |
| ---------------- | -------------------------------------------------------------------------------------- |
| `jsbsim_bridge`  | 通过 JSBSim 的 socket/FDM 接口读飞机状态 → 填 core 的 Sample struct；把 `ServoCommand` 写回 JSBSim 作动器 |
| `flightgear_out` | 把状态以 FlightGear native-FDM 协议发出去渲染 3D（演示用，可关）                                          |
| `sitl_main`      | PC 上的主循环：固定步长喂 core，记录数据曲线                                                             |

### 4.4 Failsafe 哲学（继承 ElrsRX 的精髓）

任何故障都**退化为纯直通**——舵机跟随原始通道，与一台普通接收机完全一致：

- IMU/核心算法异常 → 停止叠加修正
- SuperX 链路丢失（CRSF 超时）→ 进 failsafe 预设位置
- 状态机：UNINIT → CALIBRATING → HEALTHY → DEGRADED → FAILED
- **增稳从不阻塞 PWM 输出**。这是安全底线。

### 4.5 四级测试金字塔

```
  实机动态飞行   ← 最后，最贵
       ▲
  实机静态/HIL   ← 接舵机+电调，地面验证修正方向、混控、failsafe
       ▲
  JSBSim SITL    ← 闭环仿真，调 PID/自动驾驶，FlightGear 看动画
       ▲
  PC 单元测试    ← 你已有，测 core 各模块（确定性、CI 可跑）
```

---

## 5. 仓库目录结构（建议）

```
WeekendPilot/
├── core/                      # 纯 C++ 控制核（两端共用，零硬件依赖）
│   ├── ahrs/  filter/  pid/  modes/  mixer/  autopilot/  safety/
│   ├── sensors/              # SensorFrontend 表决/降级（纯算法，max 档冗余，见 §4.2.1）
│   └── types.h                # ImuSample / MagSample / BaroSample / AirspeedSample / GnssSample / ServoCommand 等
├── hal_esp32/                 # 板载硬件胶（PlatformIO + Arduino）
│   ├── sensors/              # IGyroAccel/IMagnetometer/IBarometer/IAirspeed/IGnss 接口 + 各型号驱动
│   ├── crsf_in/  pwm_out/  webui/  blackbox/  main.cpp
├── hal_sitl/                  # PC 仿真胶
│   ├── jsbsim_bridge/  flightgear_out/  sitl_main.cpp
├── test/                      # PC 单元测试（GoogleTest 或现有框架）
├── sim/                       # JSBSim 飞机模型 + 脚本 + 跑测脚本
│   ├── aircraft/              # 固定翼模型（从 JSBSim 自带改）
│   └── run_sitl.sh / .py
├── tools/                     # Python 上位机：数据曲线、调参可视化
├── hardware/                  # 引脚 layout、接线图、BOM
├── docs/                      # 设计文档、移植笔记、调参记录
│   ├── specs/  plans/  porting-notes/
├── platformio.ini             # 板载构建目标
├── CMakeLists.txt             # core + SITL 的 PC 构建
└── README.md
```

**分层铁律：** `core/` 里**不许**出现 `#include <Arduino.h>` / `Wire.h` / FreeRTOS。一旦出现就是分层破了。CI 用一条"core 必须能在 PC 上纯 C++ 编译"的检查守住。

---

## 6. 硬件清单 — 你需要准备什么

### 6.0 四档版本（按传感器配置分层）

硬件按**传感器集合**分四档，每档解锁的能力不同。软件一份（同一份 `core/`），靠**传感器探测 + 能力开关**自动适配当前装了哪些传感器——少装一个就降级对应功能，不改代码。测试期可先用便宜传感器，靠 §4.2.1 的模块化驱动层换型号只动一层适配。

| 档位 | 传感器集合 | 解锁能力（在前一档基础上累加） | 定位 |
|------|-----------|------------------------------|------|
| **base** | 陀螺+加速度+**磁力计**+气压 | 9 轴 AHRS（陀螺扛转弯+加速度门控+磁力计锁航向，中短时转弯不发散，见附录 C）、Angle/Rate 自稳增稳、高度保持、航向保持、感度调节、混控、救命键、G-limit、失速粗估、黑匣子 | 完整自稳飞控的最小集，无 GPS/空速也能飞好 |
| **plus** | base + **GPS** | 地速/航迹真值 → turn-comp（用 GPS 地速做 ω×V）、地速退化的空速保持、远期 RTH/航点的位置源 | 加导航与速度真值 |
| **pro** | plus + **空速管** | 空速真值 → 精确空速保持、精确失速保护（震杆+自动油门按真空速触发）、更准的 turn-comp（空速优于地速，抗风） | 类真机自动油门 + 精确失速 |
| **max** | **3× 陀螺冗余 + 2× 磁力计 + 2× 气压 + 空速管 + GPS** | 传感器表决/故障剔除（见 §4.2.1）、单点失效容错 | 冗余高可靠版 |

> **降级哲学（与 §4.4 failsafe 一脉相承）：** 任何高档传感器缺失/失效，自动退到能支撑的最低档行为，绝不阻塞 PWM。例：GPS 丢星 → turn-comp 退回 base 的纯前馈 `r=g·tanφ/V`；空速管堵了 → 失速保护退回粗估版。
>
> **测试期策略：** 手头先用差一点的传感器（便宜 QMC5883L、廉价 GPS）跑通逻辑，型号随时换——靠 §4.2.1 模块化驱动层，换型号只写一个新的驱动适配，`core/` 与上层零改动。

### 6.1 已有（直接用）

| 硬件                                   | 用途       | 备注                                            |
| ------------------------------------ | -------- | --------------------------------------------- |
| ESP32-S3 DevKitC-1 (N8R8 / N16R8) ×2 | 飞控主控     | 注意 GPIO 35/36/37(PSRAM)、3/45/46(strapping) 禁用 |
| ICM-42688-P 开发板                      | IMU      | I2C 0x68，已在 ElrsRX 验证过驱动                      |
| BMP390L 开发板                          | 气压计      | I2C 0x76，已验证                                  |
| SuperX Nano 接收机                      | RF + 信道源 | ESP32-C3 + 双 LR1121，串口出 CRSF                  |
| TX16S (EdgeTX) + ELRS 模块             | 遥控器      | 见 §10 升级到 4.0                                 |
| WROOM-1U N16R8 核心板（未焊）               | 后期打板基础   | 量产验证用                                         |

### 6.2 需要新购（按优先级）

优先级列同时标注该硬件对应解锁哪一档（见 §6.0）。

| 优先级   | 硬件                | 型号建议                            | 用途 / 档位             | 估价     |
| ----- | ----------------- | ------------------------------- | --------------- | ------ |
| **高** | 舵机 ×4~6 + 测试电调/电机 | 任意 9g 舵机 + 小电调                  | HIL 地面验证        | $20    |
| **高** | 杜邦线/面包板/5V BEC    | —                               | 接线              | $10    |
| **高** | 磁力计               | QMC5883L / IST8310              | **base 必需**：9 轴姿态环（见附录 C）；需校准+远离动力线 | $3 |
| 中     | 黑匣子外置存储（可选）       | W25Q128 flash 或 MicroSD 模块(SPI) | 需 >9min 长记录时才要  | $1~3   |
| 中     | GPS 模块            | u-blox NEO-M9N                  | **plus**：turn-comp 地速源 / 远期 RTH | $15    |
| 中     | 空速管 + 差压传感器       | MS4525DO (~$10) 或 SDP3x         | **pro**：精确空速保持 / 失速精确版 / 抗风 turn-comp | $10~30 |
| 低     | 冗余传感器套（2 套 IMU/mag/baro） | 同上各型号再买一份/换型号                  | **max**：表决冗余（见 §4.2.1），最后做 | $10~20 |

> **黑匣子先用内置 PSRAM**（零引脚，250Hz 记 4~9 分钟，详见 §12.1），不必先买存储；需更长记录再补 W25Q128/SD。
> **采购顺序 = 档位顺序**：先凑齐 base（磁力计已在高优先级）跑通完整自稳；再上 GPS 进 plus；空速管进 pro；冗余套件留到 max。每档都能独立飞，不必一次买齐。
> **空速管不到位也能起步**：失速保护先用"低速大仰角 + 法向 G < 1"粗估，空速保持先用 GPS 地速（plus）退化或暂缓。

### 6.3 引脚分配（沿用 ElrsRX 已验证的 DevKit layout 作起点）

| 功能                     | GPIO                | 备注                       |
| ---------------------- | ------------------- | ------------------------ |
| I2C 共享（IMU+Baro+pitot） | SDA=18, SCL=39      | 三个传感器同总线，I2cLockGuard 保护 |
| PWM CH1-8              | 1,2,8,9,10,15,16,17 | 舵机/电调，固定翼舵机多，可能要扩        |
| CRSF UART（接 SuperX）    | RX=44, TX=43        | SuperX 的 TX→S3 的 RX      |
| ICM-42688-P INT        | 40                  | 数据就绪中断（可选用）              |
| RGB LED                | 48                  | 状态指示                     |
| SD/flash SPI           | 待定                  | 避开 strapping/PSRAM 脚     |

> 固定翼舵机通道可能超过 8（副翼×2+升降+方向+油门+襟翼+起落架...），需评估是否用 PCA9685 扩 PWM 或换 N16R8 多引脚版。设计时 mixer 输出通道数做成可配置。

---

## 7. 需要克隆的参考仓库（你帮我克隆到本地）

这些是**参考源，不是依赖**——我读它们的算法实现，移植/重写进 `core/`，不直接编译它们。建议都克隆到一个统一的 `references/` 目录（**不要**放进 WeekendPilot 仓库，单独放，避免污染 git）。

| 仓库             | 克隆命令                                                           | 我要看的部分                                                                                    |
| -------------- | -------------------------------------------------------------- | ----------------------------------------------------------------------------------------- |
| **INAV**       | `git clone --depth 1 https://github.com/iNavFlight/inav`       | `src/main/flight/` (pid.c, mixer.c, imu.c)、`navigation/` (althold, position)              |
| **ArduPilot**  | `git clone --depth 1 https://github.com/ArduPilot/ardupilot`   | `libraries/APM_Control/`、`ArduPlane/` (TECS, attitude, mode_*)、`libraries/AP_L1_Control/` |
| **Betaflight** | `git clone --depth 1 https://github.com/betaflight/betaflight` | `src/main/flight/` (filter.c, imu.c)、`blackbox/` (日志格式)                                   |
| **madflight**  | `git clone --depth 1 https://github.com/qqqlab/madflight`      | ESP32-S3 整套参考，AHRS/PID 直接对照                                                               |
| **JSBSim**     | `git clone --depth 1 https://github.com/JSBSim-Team/jsbsim`    | `aircraft/` (找个固定翼模型)、socket/FDM 接口文档、`scripts/`                                          |

> **深度浅克隆** `--depth 1` 够用，这些仓库很大（ArduPilot 几个 GB），只读代码不需要历史。

### 你克隆时帮我注意

1. **JSBSim 要能跑起来**：装 JSBSim 可执行（`pip install jsbsim` 或源码编译），确认 `JSBSim --version` 能跑。
2. **FlightGear**：装 FlightGear（官网下载），确认能启动、能接 native-FDM socket（ArduPilot SITL 文档有现成连接参数可抄）。
3. 告诉我每个仓库克隆到的**绝对路径**，我按路径去读。

---

## 8. 从 ElrsRX 搬过去的"有用资料"（只搬资料，代码重写）

代码决策是"只参考、重写"，但下面这些**资料/知识/工具**值得整理过去，避免重复踩坑：

### 8.1 文档（作为知识底座）

- `docs/2026-05-25-fc-feature-concepts.md` — 涵道/3D 功能构想（远期路线图）。**已搬入本仓库 `docs/`。**

以下为 ElrsRX fork 前的历史资料，**留在 ElrsRX 仓库归档**（`C:\Repository\ElrsRx\docs\`），需要时去那边查，不复制进本仓库：

- `FEATURES.md` — ElrsRX 已实现功能的详细说明（移植时的"已验证逻辑"参考）
- `2025-05-20-elrs-stabilizer.md` + `stabilizer-implementation-plan.md` — 增稳设计决策与坑
- `手册/` — BMP390L.pdf、ESP32-S3 原理图、引脚图（硬件资料）
- `需求.md` — 原始需求（归档留底）

### 8.2 知识点（写进新仓库的 porting-notes）

- **PID 默认值**（FEATURES.md §5.2）：Angle roll kp=0.011/kd=0.0056，pitch kp=0.022/kd=0.011；Rate 默认值。这是 madflight 保守值，首飞起点。
- **D 项用陀螺**（非误差微分），符号为负 `D = -Kd·gyro`，对抗运动方向。
- **ki 独立原始积分器** + back-calculation anti-windup（DEF-002 修复经验）。
- **双缓冲热加载** + packed struct 撕裂风险（DEF-007 修复经验）。
- **模式切换**：200ms 线性渐变 + ±20/50ms 滞回防抖 + 各模式各留 I 项。
- **IMU 轴向**按物理安装方向调整（每次换板子都要重验）。
- **BMP390 检测顺序** BMP390→SPL06→BMP280（与 BME280 共享 CHIPID 0x60，靠 EVENT POR 位区分）。
- **跨核 I2C** 用 FreeRTOS 互斥锁 RAII 守卫。

### 8.3 工具（重写参考）

- `tools/imu_monitor.py` / `stabilizer_visualizer.py` — Python 上位机可视化风格，新仓库的数据曲线工具照这个写。
- `test/test_stab_mode_controller/` — PC 单测框架，证明 core 能 PC 编译，新仓库延续。

### 8.4 不搬的

- 所有 ELRS device 框架代码、LR1121 驱动、CRSF 收发底层（SuperX 接管了）、ELRS 配置/WiFi/OTA 的 ELRS 专属实现（WebUI 思路保留，实现重写）。

---

## 9. 实现路线图（需求 → 阶段）

每个阶段都先在 SITL 仿真里验证，再决定要不要上实机。每条功能上线前对应的安全网/黑匣子要先就位。

### 阶段 0：地基（1 周）

- [ ] 建 WeekendPilot 仓库，搭两层目录骨架
- [ ] 定义 core 的输入/输出 struct（`types.h`）
- [ ] PC 构建（CMake）+ 单测框架跑通"core 纯 C++ 编译"CI 检查
- [ ] **SITL 桥最小闭环**：JSBSim 喂假数据 → core 直通 → 写回 JSBSim，确认管道通
- [ ] 板载骨架：CRSF 串口读 SuperX 通道 → 直通 PWM（等同普通接收机，failsafe 基线）

> **里程碑：** 仿真和板子两端都能"直通"，core 框架就位。

### 阶段 1：核心增稳（1~2 周）— 需求 #1 #2 #7

- [ ] 移植 Mahony AHRS + PT1 滤波 → core/ahrs, core/filter
- [ ] 移植 PID（陀螺 D + 独立 ki + anti-windup）→ core/pid
- [ ] Off/Angle/Rate 状态机 + 平滑过渡 → core/modes
- [ ] 感度通道（gain）实时调
- [ ] **SITL 调参**：在 JSBSim 固定翼模型上调出能稳定平飞的 Angle/Rate
- [ ] 黑匣子先上（建议提前，250Hz→PSRAM 环形缓冲，`BlackboxSink` 接口）→ hal/blackbox

> **里程碑：** 仿真里飞机能自稳/增稳/调感度。FlightGear 看动画确认控制方向对。

### 阶段 2：混控 + 外设 + G-limit（1~2 周）— 需求 #3 + G-limit（**详见附录 D**）

- [ ] Mixer：标准副翼/升降/方向/油门 + flaperon/V-tail/elevon（预设→规则表展开）→ core/mixer
- [ ] 外设通道直通（起落架/襟翼/灯）
- [ ] G-limit 过载限制（软限上方线性衰减拉杆）→ core/safety
- [ ] 实机静态/HIL：接舵机验证混控方向、failsafe
- [ ] ~~Panic Recovery 救命键~~ → **挪到阶段 3**（依赖 alt-hold 才能做"高度保持"那一项，避免返工）

> **里程碑：** 机型可配置，过载保护就位，可接舵机做 HIL 地面验证。

### 阶段 3：选择性自动驾驶（2~3 周）— 需求 #8 #9 #10 #4 #6

- [ ] Alt-Hold（BMP390）→ core/autopilot
- [ ] Panic Recovery 救命键（回正+油门锁中+**高度保持**）→ core/safety（**从阶段 2 挪入，依赖 Alt-Hold**）
- [ ] Heading-Hold（陀螺积分短期版）→ core/autopilot
- [ ] 协调转弯（`r=g·tanφ/V` 前馈）→ core/autopilot
- [ ] Auto-trim 自动配平
- [ ] 失速改出（粗估版，无空速管）→ core/safety
- [ ] 空速保持 + 失速精确版（**装空速管后**）

> **里程碑：** 选择性自动驾驶可用，SITL 全程验证，逐项上实机。

### 阶段 4：上位机 + 远期（持续）

- [ ] WiFi WebUI 实时调参页 + 黑匣子下载
- [ ] 评估 S3 抖动是否够（1kHz Mahony + cascade PID + notch 实测 latency）
- [ ] 远期：涵道/矢量混控、刀飞、prop-hang（装对应机型后，见 fc-feature-concepts.md）

---

## 10. SuperX ELRS 3.5.3 → 4.0 升级（你的 side-quest）

你的 SuperX 实测固件是 `UNIFIED_ESP32C3_LR1121_RX_3.5.3 (40555e).bin`——**官方 ExpressLRS 3.5.3**（`UNIFIED_` 前缀 + 版本号 + commit 哈希是官方 release 标准命名，BetaFPV 出厂即官方固件）。TX16S 模块为 3.x。目标升到 4.0 体验新功能 + 启用 §B.4 的 CRSF 参数菜单调参。

> ✅ **升级路径已验证通**：本地 ExpressLRS 4.0 源码（`C:\Repository\ExpressLRS`，`4.0.0-184`）的 `src/targets/esp32c3-rx.ini` 中存在 `Unified_ESP32C3_LR1121_RX_via_{UART,BetaflightPassthrough,WIFI}` 三个目标——**你的 SuperX 在 4.0 仍有对应固件,能升**。
>
> ⚠️ **网络限制：** expresslrs.org 官网被网络策略挡住，下面流程基于源码 + 既有知识 + 第三方指南整理，**实操时对照官方 quick-start 核对每个 UI 按钮名**。

### 10.1 黄金法则（务必先懂）

**TX 和 RX 必须同一大版本 + 同一频域才能绑定。** 你现在 3.5.3 → 4.0 是跨大版本，所以**两端（SuperX + TX16S 模块）必须一起升**——只升一端会失联。升级前记下当前 **Binding Phrase**，两端刷同一短语就自动绑上（刷带 phrase 的固件会重置绑定，填一样的会重新绑）。([来源](https://blog.uavmodel.com/expresslrs-flashing-and-binding-the-complete-setup-guide/))

### 10.2 工具

- **ExpressLRS Configurator**（桌面 app，推荐）或官网 **Web Flasher**。选 4.0.x 正式版 release。
- 频域：SuperX 是 LR1121（双频能力），按你实际使用频段选；2.4G 选 `ISM2400`。

### 10.3 升级 TX 模块（TX16S 内置/外置 ELRS 模块）

1. 装 ExpressLRS Configurator，选 4.0.x，选你的 TX 模块 target。
2. 填 Binding Phrase（与 RX 一致）。
3. 刷写方式：**WiFi**（模块开 AP，连上传 firmware.bin）最稳，或 UART/Betaflight-passthrough。
4. TX16S 上 ELRS LUA 脚本里确认模块版本变 4.0。

### 10.4 升级 SuperX 接收机

1. Configurator 选 SuperX 对应 target（**`Unified_ESP32C3_LR1121_RX`** ——与你当前固件同目标，4.0 里已确认存在）。
2. 同一 Binding Phrase。
3. 刷写方式：SuperX 上电进 WiFi AP 模式（即你访问 10.0.0.1 那个页面），Configurator/WebUI 走 WiFi 上传 4.0 固件。([BETAFPV 文档](https://support.betafpv.com/hc/en-us/articles/4404231679129-How-to-Flash-Firmware-of-ELRS-RX-TX))
4. 升完两端同 Binding Phrase 自动绑定，LUA 脚本确认连上。
5. 串口协议保持 **CRSF**（你已设），4.0 后即可用 §B.4 的参数菜单调参。

> 这部分独立于飞控开发，可以并行做。

---

## 附录 A：平台选型（ESP32-S3 vs STM32）与 STM32 迁移预案

### A.1 "移植算法"≠"移植固件"

要搬的是**算法，不是整套固件**。一套飞控固件可切成两类代码：

- **平台无关的"数学"**：AHRS（Mahony/EKF）、PID、混控、TECS、L1、协调转弯公式……本质是"输入一堆 float，输出一堆 float"，`r=g·tanφ/V` 在哪颗 MCU 上都一样。
- **平台相关的"I/O 和调度"**：陀螺寄存器读取、定时器/DMA 驱 PWM、RTOS 调度、DShot、Flash/DMA 配置。和芯片死死绑定。

INAV/Betaflight 之所以 "STM32-only"，卡的是第二类——它们的 I/O 层按 STM32 寄存器/DMA 写死，构建系统也围着 STM32 转，整体搬到 ESP32 等于重写。**我们的做法**：读它们的第一类代码（纯数学），重写进 `core/`；第二类自己针对 ESP32-S3 写 `hal_esp32/`。这正是 core/hal 硬切两层的根本动机——**算法可移植，I/O 各写各的**。

> 准确性补注：ArduPilot 有**实验性** `AP_HAL_ESP32` 端口（非主力、功能残缺、不建议拿去飞），INAV/Betaflight **完全没有** ESP32 端口。所以 "STM32-only" 对 INAV/BF 准确，对 ArduPilot 是 "STM32 才是一等公民"。

### A.2 两平台对固定翼的实质差异

| 维度     | STM32F4/F7           | ESP32-S3                          | 对固定翼的影响                                            |
| ------ | -------------------- | --------------------------------- | -------------------------------------------------- |
| 实时性/抖动 | 硬实时，中断~120ns，抖动<5µs  | FreeRTOS+WiFi，最差抖动~30-50µs        | **1kHz(1ms)下 50µs 仅 5%，固定翼够。** 只有 4-8kHz 高速陀螺环才被吃掉 |
| FPU    | 单精度硬件 FPU            | 单精度硬件 FPU，240MHz                  | 两边都不是瓶颈                                            |
| ESC 协议 | 定时器+DMA 完美驱 DShot 双向 | DShot 走 RMT，欠成熟                   | **固定翼标准 PWM 舵机+单电调，用不到 DShot**                     |
| 无线     | 无（要外挂）               | **内置 WiFi+BLE**                   | **杀手锏**：地面无线调参/下黑匣子                                |
| 内存     | F405:192KB/1MB       | 512KB SRAM+8MB PSRAM/8-16MB Flash | 黑匣子/WiFi/网页更宽裕                                     |
| 生态     | BF/INAV/ArduPlane 原生 | madflight 主力参考                    | 直接刷成熟固件只能 STM32；自写+无线调参 ESP32 香                    |

**结论**：STM32 的优势集中在"硬实时+高速环+DShot+直接跑成熟固件"，**恰好都是固定翼用不到、或已主动放弃的**；ESP32-S3 的优势（WiFi、大内存、已验证驱动）正中需求。**现阶段不买 STM32，ESP32-S3 够用且有余量。**

### A.3 何时才考虑 STM32

① 转去搞穿越机/激进 3D，要 4-8kHz 陀螺环+DShot 双向遥测；② 改主意想直接刷 Betaflight/INAV/ArduPlane（已否决）；③ 需要安全关键级硬实时自动驾驶。**目前都不是本项目路线。**

### A.4 STM32 迁移预案（真有需求时怎么搬）

因为 core/hal 两层硬切，迁移成本被压到最低——**`core/` 一行不动，只重写 hal 层**。具体步骤：

1. **选板**：Matek/SpeedyBee F405 或 F722 现成飞控板（带 IMU/baro/OSD/blackbox），或自己用 F405 核心板。
2. **换工具链**：PlatformIO 的 STM32 框架（Arduino-STM32 或 STM32Cube/HAL），CMake 那条 SITL 构建**完全不变**（core 是纯 C++，PC 端不受影响）。
3. **重写 `hal_stm32/`**（对照 `hal_esp32/` 逐模块）：
   - `crsf_in`：STM32 UART+DMA 收 CRSF（可参考 Betaflight 的 rx/crsf.c）
   - `imu`/`baro`：换 SPI/I2C HAL 调用，传感器 struct 接口不变
   - `pwm_out`：STM32 定时器通道（可上 DShot）
   - `blackbox`：W25Q128 SPI flash（STM32 上 DMA 更顺）
   - 调参：**WiFi 没了**，改外挂 ESP01/ESP32-C3 桥，或纯走 §B 的 ELRS 链路调参（反而更统一）
4. **RTOS**：ESP32 的 FreeRTOS 双核编排换成 STM32 单核裸机主循环或 ChibiOS（Betaflight/ArduPilot 路线）。
5. **验证**：先跑 PC SITL 确认 core 行为一致（应当 bit-level 一致，因为 core 没变），再 HIL，再实机。

> 预估工作量：~1-2 周重写 hal 层（`fc-feature-concepts.md` 阶段 4 的估算）。算法零重写是这套架构最大的保险。

---

## 附录 B：调参链路策略（飞行中 vs 地面）

> 核心结论：**飞行中靠 ELRS 的 AUX 旋钮实时调；WiFi 退化为纯地面工具，飞行时关闭。** 你"通过 ELRS 调"的直觉是对的。

### B.1 为什么 WiFi 不能用于飞行中

ElrsRX 那套 WiFi WebUI（AP 模式→浏览器 `POST /config`→热加载）技术上能不重启改 PID，但用在飞行中有三个硬伤：

1. **距离**：ESP32 WiFi AP 视距几十米，飞远即断。
2. **没有手**：打杆做特技时腾不出手戳屏幕。
3. **同频干扰**：FC 的 WiFi 是 2.4G，**ELRS 链路也是 2.4G**，飞行中开 AP 等于在接收机旁塞 2.4G 干扰源，可能压低链路质量。

所以 **WiFi 是"落地后、地面"的调参/看曲线/下黑匣子工具，不是飞行中的**。

### B.2 飞行中实时调参 = ELRS 链路（两种做法）

**做法一：AUX 旋钮（推荐，主力）。** 把 TX16S 的旋钮/滑块/段位开关映射成"参数通道"，FC 读通道值实时缩放增益。这就是"感度通道 gain_channel"的泛化（可挂 2-3 个旋钮：总增益、roll P、pitch P）。**全 RC 距离好使、零延迟、手不离杆**——RC 圈调飞控的标准做法（Betaflight/INAV 的 adjustments-via-AUX）。代价：旋钮有限，一次盲调几个参数。

**做法二：CRSF 参数菜单 / MSP-over-CRSF（已确认可行）。** 在 TX16S 的 ELRS Lua 菜单里像翻 ELRS 设置一样改 FC 的命名参数：遥控器→ELRS 模块→空中→SuperX→CRSF 串口→FC。好处是参数多、有菜单、比旋钮精确又抗距离。**已基于 ELRS 4.x 源码确认这是标准能力，不依赖 SuperX 私有功能**（机制见 §B.4），我们只需在 FC 侧实现 CRSF 参数协议（DEVICE_PING / PARAMETER_READ / PARAMETER_WRITE）注册一张参数表。**前提：SuperX 升级到 ELRS 4.0**（你本就计划），串口协议选 CRSF（你已选）。

### B.3 三层混合调参方案

| 方式                  | 场景                   | 可靠性                     |
| ------------------- | -------------------- | ----------------------- |
| **AUX 旋钮（ELRS）**    | **飞行中**实时盲调关键增益      | ★★★ 全距离、手不离杆            |
| **CRSF 参数菜单（ELRS）** | 飞行中/地面，遥控器屏幕翻菜单改命名参数 | ★★★ 全距离、有菜单、已确认可行       |
| **WiFi WebUI**      | **落地后**大改参数、看曲线、下黑匣子 | ★ 仅地面，飞行时关闭避免 2.4G 干扰   |

> 实现优先级：先做 AUX 旋钮（最简单）；CRSF 参数菜单作为飞行中调参的精确主力（比 WiFi 优雅，全距离可用）；WiFi 退到"下载黑匣子大文件"这一个不可替代的用途。

### B.4 已确认的机制（ELRS 4.x 源码佐证）

读 `C:\Repository\ExpressLRS\src` 4.x 源码（CRSFRouter 路由架构）确认：

- **设备自动发现**：`CRSFRouter::processMessage` 对任何扩展头帧（类型 ≥ `DEVICE_PING`，即参数/MSP 类）把"源地址"记进对应 connector 的设备表（`addDevice`）。FC 通过串口向上 ping 一次，SuperX 就记住"FC 在串口那头"。
- **定向转发**：`deliverMessage` 把目标地址为 FC 的参数/MSP 帧精确转发到串口 connector；广播帧发给所有 connector。
- **串口写出**：`SerialCRSF::forwardMessage` 把帧推进串口 FIFO，并把同步字节改成标准 `0xC8`——专为喂外部设备/FC。
- **RX 也响应参数帧**：`RXEndpoint::handleMessage` 明确处理 `DEVICE_PING / PARAMETER_READ / PARAMETER_WRITE` 与 `MSP_RESP/MSP_WRITE`。

这正是 Betaflight/INAV 用 ELRS Lua 在遥控器上调飞控参数的同款机制。你截图（10.0.0.1 SuperX 配置页）显示 Serial Protocol 选了 **CRSF**、设备已 **Bound**，与此完全吻合。

---

## 附录 C：AHRS 可观测性——为什么 base 档必须 9 轴（陀螺+加速度+磁力计）

> 来源：阶段 0~1 SITL 验收（Task 1.7）失败后的 systematic-debugging 根因调查（2026-05-30），用 JSBSim c172p 真值钉死。

### C.1 现象与根因

阶段 0~1 先用 **6 轴 Mahony（陀螺+加速度，无磁力计）**。SITL 闭环里，Angle 模式扰动后短时能回水平，但持续转弯会发散成螺旋。根因经 JSBSim 真值实测确认：

| 时间 | 真实滚转 phi | 加速度计推断滚转 atan2(Ny,Nz) |
|------|-----------|------------------------|
| 1s | 12° | -3° |
| 5s | 59° | -4° |
| 8s | **84°** | **-3°** |

**协调转弯里向心加速度恰好抵消重力的侧向分量**，机体比力始终指向"正下方"，加速度计在物理上分不清 banked 转弯和平飞 → 6 轴 AHRS 以为飞机一直是平的，放任进螺旋。这是固定翼 AHRS 的经典不可观测问题。短时小扰动（平飞阵风）能纠；持续转弯姿态保持不行。

### C.1b 二次调查：9 轴也救不了滚转，真正扛转弯的是陀螺（2026-05-30 复盘）

上线 9 轴后转弯仍发散，二次 systematic-debugging 把同一段录制的转弯传感器流离线灌进真实 AHRS，分四种配置对比（铁证）：

| 配置 | 转弯中滚转误差 | 结论 |
|------|-------------|------|
| **纯陀螺（kp=0，关校正）** | 一路到 120° 坡度 **<0.4°** | 陀螺单独就能精确扛转弯，单位/坐标/积分全对 |
| 6 轴（kp=1，仅加速度） | **45~66°** | 加速度把滚转拽向水平 |
| 9 轴（kp=1，加速度+磁力计） | **45°** | 磁力计救了航向，但**对滚转几乎无用** |
| 9 轴 + 加速度幅值门控 | 闭环真实 45° 转弯 **9.5°** | 见 C.3 的修法 |

**关键纠正：** 之前以为"磁力计能约束转弯滚转漂移"——**实测不成立**。地磁水平分量只约束**航向（偏航）**，对**滚转**几乎不提供信息。协调转弯里加速度计也看不见倾斜（atan2 钉在 -2~-5°）。所以转弯中**滚转的唯一短期真相是陀螺积分**；加速度/磁力计在机动时反而帮倒忙，必须门控掉。

### C.2 一个被推翻的错误归因（记录以儆效尤）

执行 subagent 报告称"加速度标度 bug（read_imu 把已是 g 的 Nx/Ny/Nz 又除 32.174）导致重力参考归零所以漂移"。**实测推翻**：Mahony 对加速度向量做归一化（recipNorm），统一标度被抵消，缩放前后姿态输出差异 `0.00e+00`。标度 bug 对姿态零影响。**但它仍是真 bug 要修**——G-limit / 失速保护会读加速度幅值（法向 G），那时标度必须对。

### C.3 决策：base = A+B（陀螺扛转弯 + 加速度门控），C/D 留给 plus/pro

转弯中要知道滚转，物理上只有四条路（一份固件，按型号开关，§6.0/§4.2.1）：

| 路线 | 机制 | 档位 | 说明 |
|------|------|------|------|
| **A 陀螺积分** | 直接积分机体角速度，不依赖重力参考 | base | 短期唯一真相，转弯里实测 <0.4°；长期有零偏漂移 |
| **B 加速度/磁力计门控校正** | 仅当 \|a\|≈1g（0.9~1.1g 窗口）才信加速度纠漂；磁力计锁航向 | base | 平飞时把 A 的漂移拉回；机动时**门控掉**避免被向心加速度污染 |
| **C 转弯补偿 turn-comp** | 从加速度减去向心加速度 `ω×V`，还原真重力方向 | plus/pro | 需速度源 V：GPS 地速(plus) / 空速管真空速(pro，抗风更准) |
| **D 完整 EKF** | IMU+GPS+mag+空速+气压全融合最优估计 | pro+ | 离不开 GPS，重量级，终极方案 |

- **base 收口 = A + B**：陀螺扛住转弯（A），加速度做幅值门控（B，移植 Betaflight/madflight 的 0.9~1.1g 窗口，`config_alen2_min/max`）。**9 轴照装**——磁力计的航向观测是真的、也是必需的（锁航向、长期偏航参考），只是它**不负责转弯滚转**。SITL 闭环真实 45° Angle 转弯实测：上门控后估计跟住真值 9.5°（门控前 45°）。
- **plus/pro → C（turn-comp）**：装了 GPS/空速后解锁 `ω×V` 补偿，长时间盘旋也能精确保持转弯姿态。**最终在同一份固件里，靠探测到速度源自动开启**（§4.2.1 能力门控）。
- **pro+ → D（EKF）**：远期，需 GPS。
- **max → 多传感器表决**（见 §4.2.1）：3 陀螺中值滤掉单只卡死/跳变，2 磁/2 气压互校，AHRS 算法不变，吃表决后的干净数据。

> **修正旧结论：** 之前写"base 靠 9 轴磁力计就稳住转弯不发散"是**高估了磁力计**。准确表述：base 靠**陀螺扛转弯 + 加速度门控**稳住中短时转弯（覆盖正常航模架次），磁力计锁航向；**长时间持续盘旋**才真的需要 C 的 turn-comp（plus/pro）。turn-comp 是精度/久航增强，不是 base 能飞的前提。

### C.4 SITL 验证方式（JSBSim 无磁场，需合成磁力计）

**JSBSim 1.3.1 不提供地磁场属性**（`query_property_catalog('mag')` 无磁场项；注意 `get_property_value` 对不存在属性静默返回 0.0，是陷阱）。SITL 测 9 轴 AHRS 的办法：从 JSBSim 真值姿态（`attitude/phi-deg`、`theta-deg`、`psi-deg`，均确认可用且随姿态变化）+ 固定地磁参考向量，旋转到机体系合成模拟磁力计读数喂 AHRS。标准传感器仿真做法。

> 这一发现把"磁力计"从原 §6.2 的"低优先级（航向保持长期参考）"提升为 **base 档姿态环的必需件**（最高优先级）。

---

## 附录 D：阶段 2 详细设计（Mixer + 外设直通 + G-limit）

> 由 brainstorming 流程产出（2026-05-30），已与你逐项确认。实现照 **Sonnet 写码 + Opus review** 模式。

### D.1 范围与失效回退

阶段 2 = **Mixer + 外设直通 + G-limit**。**救命键 Panic Recovery 挪到阶段 3**（它的"高度保持"那一项依赖 alt-hold 气压外环，现在做会返工）。

失效回退保持现状：未启用 / 链路丢失 / IMU 无效 / Off 模式 → 早返回，原样直通所有通道，**不过 mixer**。

> **已知限制（明确记一笔）**：失效态下 V尾/elevon 机型拿到的是裸 RC 通道（非混控输出），舵面方向可能不对。真正的 failsafe-降级（失效时仍走 mixer 的安全位）留到阶段 3 的 safety failsafe 处理。本阶段与现有代码行为一致。

### D.2 数据流

```
[stab corr] ──┐
[manual stick]├─► 轴需求 src[Roll/Pitch/Yaw/Throttle/Flap]
[gain]       ─┘            │
                           ▼  G-limit：法向G超软限→线性衰减"拉杆方向"的 src[Pitch]
                      Mixer.mix(src) ──► acc[8] 累加 ──► clamp ──► servo[8]
                           │
                      外设直通：用裸 RC us 覆盖指定 servo 输出口（起落架/灯/襟翼开关）
                           ▼
                      ServoCommand{ servo[8] }
```

C API **无需改动**（混控是 core 内部的，对外仍输出 servo[8]）。

### D.3 Mixer 数据模型（预设 → 展开成规则表）

照 **INAV servo mixer** 的做法（`inav/src/main/flight/servos.h:129` 的 `servoMixer_t`、`servos.c:438` 的累加循环），精简为归一化浮点版：

```cpp
enum class MixSource : uint8_t { Roll, Pitch, Yaw, Throttle, Flap, Count };
enum class Airframe  : uint8_t { Standard, Flaperon, VTail, Elevon };

struct MixRule { uint8_t out; MixSource src; float weight; };

struct MixerConfig {
    uint8_t  num_outputs = 8;
    MixRule  rules[24];            // 固定容量，嵌入式不堆分配
    uint8_t  num_rules = 0;
    bool     is_throttle[8] = {};  // 哪些输出口是油门（单边 1000~2000）
};
```

用户侧选 `Airframe`，`expand(airframe)` 填出规则表 + `is_throttle`。统一引擎：

```
acc[8] = 0
for r in rules:  acc[r.out] += r.weight * src[r.src]
for i: is_throttle[i] ? servo[i] = 1000 + 1000*clamp(acc[i], 0, 1)
                      : servo[i] = normToServoUs(clamp(acc[i], -1, 1))  // ±1 居中
```

四种预设展开（servo 索引 = 物理舵机口）：

| 布局 | 规则 |
|---|---|
| **Standard** | `0←Roll(+1)`, `1←Pitch(+1)`, `2←Throttle`, `3←Yaw(+1)` |
| **Flaperon** | Standard + `0←Flap(+1)`, `4←Roll(-1)`, `4←Flap(+1)`（左右副翼差动滚转、同向当襟翼）|
| **VTail** | `0←Roll(+1)`, `1←Pitch(+1)`,`1←Yaw(+1)`, `3←Pitch(+1)`,`3←Yaw(-1)`, `2←Throttle` |
| **Elevon** | `0←Roll(+1)`,`0←Pitch(+1)`, `1←Roll(-1)`,`1←Pitch(+1)`, `3←Yaw(+1)`, `2←Throttle` |

**Standard 展开后行为与现有 `controller.cpp` 逐位一致**（回归测试锁住）。

**饱和处理**：这版选 **逐通道 clamp 到 ±1**（最简单）。以后若要升级成"饱和时按组比例缩放保留 roll/pitch 比例"，照 Betaflight `mixer.c:556` `applyMixerAdjustmentLinear` + `motorMixRange` 的归一化因子来（备查）。

### D.4 G-limit（过载限制）

读 IMU 法向载荷 `accel_z`（单位 g，平飞 ~+1）。软限 6G、硬限 10G（可配）。**只衰减"拉杆 / nose-up"方向**的 pitch 需求，推杆（卸载）不限：

```
G = accel_z
nose_up = (src[Pitch] 的符号 == 产生抬头/正法向G 的符号)  // 符号约定在实现时按实际舵面定，单测锁住
if G > soft && nose_up:
    k = clamp(1 - (G - soft)/(hard - soft), 0, 1)
    src[Pitch] *= k        // 到硬限 k=0，拉舵贡献清零
```

放在 mixer **之前**作用于 pitch 轴需求。无状态、无 PID、好测。参考 ArduPlane `ArduPlane/Attitude.cpp` 的 `aerodynamic_load_factor`（它按空速算真·载荷限制，pro 档思路；我们 base 档用 IMU 法向G 的简化版）。

### D.5 外设直通

`ControllerConfig` 里一张小表 `{ servo_out, rc_channel, enabled }`，混控算完后**覆盖**指定输出口（起落架/襟翼/灯开关直接拷贝裸 RC us）。

### D.6 新增文件与测试

**新增**：
- `core/mixer/mixer.h` / `.cpp` — 类型、`expand(Airframe)` 预设展开、`mix(src) → servo[8]`
- `core/safety/glimit.h` / `.cpp` — GLimitConfig + 衰减函数
- `core/controller.h/.cpp` — 接线：config 加 `Airframe`/`GLimitConfig`/外设表/`flap_channel`；update 改成 demand → G-limit → mix → 外设直通
- `test/test_mixer/` — Standard 复刻现有行为、VTail/Elevon/Flaperon 展开与符号、饱和 clamp、油门单边
- `test/test_glimit/` — 软限下无变化、软硬限间线性、硬限清零、推杆不受限
- CMake：`wp_add_test(test_mixer)` `wp_add_test(test_glimit)`（core 的 .cpp 被 GLOB 自动收）

**SITL**：默认 Standard，跑现有 recover/turn 不回归即可。V尾/elevon 由单测覆盖——JSBSim c172 只认标准舵面（`fcs/aileron|elevator|rudder-cmd-norm`），喂不进非标准混控布局，故不在 SITL 验证。

### D.7 参考来源（按文件）

| 组件 | 参考 | 文件 | 协议 |
|---|---|---|---|
| Mixer 规则表+累加引擎 | INAV servo mixer | `inav/src/main/flight/servos.{h,c}`（结构 `servos.h:129`，循环 `servos.c:438`，输入装配 `:304`） | GPL-3.0 |
| flaperon / V尾 / elevon 符号 | ArduPlane 舵面分配 | `ardupilot/ArduPlane/servos.cpp` | GPL-3.0 |
| G-limit（载荷限制） | ArduPlane | `ardupilot/ArduPlane/Attitude.cpp`（`aerodynamic_load_factor`） | GPL-3.0 |
| 饱和按比例缩放（备查，本版未用） | Betaflight | `betaflight/src/main/flight/mixer.c:556` | GPL-3.0 |

---

## 11. 关键设计取舍（写下来防止以后忘）

| 取舍       | 结论                      | 理由                                                                 |
| -------- | ----------------------- | ------------------------------------------------------------------ |
| 平台       | ESP32-S3（短期）            | 复用现有栈 + WiFi 边飞边调。S3 1kHz 增稳过剩；4-8kHz gyro loop 才吃紧——固定翼用不到。详见附录 A |
| 调参链路     | 飞行中 ELRS AUX 旋钮；地面 WiFi | WiFi 同频干扰 2.4G + 飞远即断，仅地面用；飞行中靠 ELRS。详见附录 B                        |
| 分层       | core / hal 硬切           | 同一份算法跑板子和 SITL，可单测，是整个测试金字塔的地基                                     |
| RF       | 完全交给 SuperX             | 不维护 LR1121 栈，专注算法                                                  |
| 算法来源     | 移植 INAV/ArduPlane/BF    | 不自己抠参数，站在成熟飞控肩膀上                                                   |
| 混控表示     | 预设→展开成规则表             | 用户选 Airframe，内部统一规则表引擎（照 INAV servo mixer）；为矢量/涵道与 WebUI 调参留口。见附录 D |
| G-limit  | 软限上方线性衰减拉杆           | 无状态、好测；只衰减拉杆方向，推杆卸载不限。base 用 IMU 法向G，pro 档可升级为按空速算真载荷。见附录 D |
| 版本分档     | base/plus/pro/max 按传感器集 | 一份代码靠探测+能力开关自适应，少装传感器自动降级，不分叉固件。见 §6.0                          |
| 传感器层     | 按类型抽接口 + 表决聚合          | 测试期用便宜件、量产换好件只动一层驱动；max 冗余对 core 透明。见 §4.2.1                     |
| 仿真       | JSBSim + FlightGear     | ArduPlane SITL 同款，免费跨平台，调参/演示分离                                    |
| 首版机型     | 传统固定翼                   | 需求主线；涵道/3D 架构预留接口，以后做                                              |
| Failsafe | 任何故障退直通                 | 安全底线，增稳绝不阻塞 PWM                                                    |

## 12. 开放问题的决策（已与你确认）

| # | 问题 | 决策 |
|---|------|------|
| 1 | 仓库名 | **WeekendPilot**，仓库已建：[Fr0zenForest/WeekendPilot](https://github.com/Fr0zenForest/WeekendPilot) |
| 2 | 参考仓库克隆路径 | `C:\Repository`（与 WeekendPilot 同级，不嵌进仓库） |
| 3 | JSBSim 机型 | 用自带固定翼起步（Rascal110 / c172）。手头有初教-6 模型，区别不大，先用自带 |
| 4 | 黑匣子存储 | **三者都支持，抽象 `BlackboxSink` 接口；首发用内置 PSRAM 环形缓冲**（零引脚），W25Q128/SD 作可选 sink 后补。详见 §12.1 |
| 5 | PWM 通道数 | **先做 8 路**，mixer 输出通道数设计成可配置，预留 PCA9685(I2C) 外扩接口 |
| 6 | 空速管 | **后补**。失速保护先做粗估版（低速大仰角+法向G<1），空速保持暂缓 |
| 7 | SuperX 透传 | **已确认可行**（基于 ELRS 4.x 源码 + 你的 SuperX 配置页截图）。ELRS 4.x RX 标准会把 FC 地址的参数/MSP 帧路由到 CRSF 串口，不依赖 SuperX 私有功能。前提：SuperX 升 4.0 + 串口选 CRSF（已选）。机制见附录 B.4 |
| 8 | AHRS 轴数 / 版本分档 | **base 档 9 轴（陀螺+加速度+磁力计+气压）**，plus 加 GPS（turn-comp 地速源），pro 加空速管（精确空速/失速、抗风 turn-comp），max 加冗余（3 陀螺+2 磁+2 气压表决）。SITL 已证 6 轴在固定翼协调转弯不可观测（发散成螺旋）。四档传感器配置见 §6.0、模块化驱动+表决见 §4.2.1、机制 + JSBSim 合成磁力计验证见附录 C |

### 12.1 黑匣子存储与内存占用评估

**关键坑：绝不往 ESP32 程序 Flash 高速写日志。** ESP32 Flash 内存映射，写/擦要关 cache 会卡住代码执行（拖累 1kHz 控制环），且有擦写寿命。"内置存储"的正确姿势是**写 PSRAM（R8 模块有 8MB）环形缓冲，落地后 WiFi 下载**——零引脚、不磨损、不卡环。

每帧数据量：精简帧 ~40B（时间戳+陀螺+姿态+设定点+8舵机+标志），完整帧 ~90B（再加 加速度+气压/升降+8通道+PID三项）。Betaflight 格式带 delta+变长编码，实际落盘约原始 50~60%。

各介质能记多久（按完整帧 90B 原始上限算，开压缩可翻倍）：

| 介质 | 容量 | 1000Hz | 500Hz | **250Hz（推荐）** | 100Hz | 额外引脚 |
|------|------|--------|-------|---------|-------|---------|
| PSRAM 内置 | ~6MB 可用 | 66s | 2.2min | **4.4min** | 11min | **0** |
| W25Q128 外置 | 16MB | 3min | 6min | **12min** | 30min | 1~4(共享SPI) |
| SD 卡 | 8GB | ~24h | — | **基本无限** | — | 4(共享SPI) |

**结论：固定翼动态慢（带宽几 Hz~20Hz），250Hz 记录足够看清振荡、调 PID，1kHz 是浪费。** 250Hz 下 PSRAM 记 4~9 分钟（开压缩），覆盖一次典型起落/调参架次，零额外硬件。需更长记录时挂 W25Q128 或 SD（同一 `BlackboxSink` 接口切换）。

### 12.2 SuperX 透传 — 已确认（附实机自测法）

**已基于 ELRS 4.x 源码确认透传是标准能力**（机制见附录 B.4），且你的 SuperX 配置页显示 Serial Protocol=CRSF、已 Bound，完全吻合。若想在实机上再自测一遍：SuperX 串口 TX 接 USB-TTL，遥控器触发一次 ELRS Lua 参数请求，看串口有无冒出 CRSF 参数帧（0x2B/0x2C/0x2D/0x2E）或 MSP 帧（0x7A/0x7B），冒出即印证。

## 13. 术语表（统一黑话）

| 术语                     | 含义                                                   |
| ---------------------- | ---------------------------------------------------- |
| SITL                   | Software-In-The-Loop，控制核在 PC 上跑、与仿真器闭环               |
| HIL                    | Hardware-In-The-Loop，真飞控板接仿真/地面台                     |
| AHRS                   | 姿态航向参考系统，这里指 Mahony 互补滤波出欧拉角                         |
| CRSF                   | Crossfire/ELRS 串口协议，SuperX 出的信道格式                    |
| TECS                   | Total Energy Control System，ArduPlane 的能量管理（油门+升降协调） |
| L1                     | ArduPlane 航迹跟踪导引律                                    |
| Horizon                | 摇杆中位自稳、打满放权的混合模式                                     |
| flaperon/elevon/V-tail | 常见固定翼舵面混控布局                                          |

---

## 14. 你接下来要做的事（清单）

写完本文后，按这个顺序准备，准备好我们就搬去新文件夹开工：

1. ✅ 仓库名定 WeekendPilot，仓库已建
2. ☐ 克隆 5 个参考仓库到 `C:\Repository`（§7），克隆完告诉我
3. ☐ 装好 JSBSim（`pip install jsbsim`，验证 `JSBSim --version`）
4. ☐ 装好 FlightGear，确认能启动
5. ☐ 装好 PlatformIO（板载构建）+ CMake（PC 构建）
6. ☐ 新购硬件：测试舵机/电调 + 杜邦线/BEC（黑匣子先用内置 PSRAM，空速管后补，暂不下单）
7. ☐ 把 §8 列的文档/手册资料搬去新仓库
8. ☐（并行，不挡开发）研究 SuperX+TX16S 升 4.0（§10）；有空时按 §12.2 验证 SuperX 透传
9. ☐ review 本文档，确认无误后我用 writing-plans 出详细实现计划

> 准备齐 2~5 后，我就能在 WeekendPilot 仓库搭好两层骨架、跑通 SITL 直通最小闭环（阶段 0）。黑匣子首发走 PSRAM、PWM 先 8 路、空速管后补——首批开发不被任何采购阻塞。

---

*本文档由 brainstorming 流程产出，下一步：你 review → 我用 writing-plans 出 `docs/plans/` 实现计划 → 逐阶段实现。*
