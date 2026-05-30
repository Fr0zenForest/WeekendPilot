# WeekendPilot

固定翼伴侣飞控（ESP32-S3）。SuperX Nano 接收机经 CRSF 串口供给遥控信道，本固件自己跑姿态解算 + 增稳 + 混控 + 选择性自动驾驶，输出 PWM 给舵机/电调。算法不自己抠参数，而是从成熟开源飞控**移植成熟逻辑**；先在 JSBSim + FlightGear 仿真里调好，再上实机。

## 架构：两层硬切

整个设计的关键，是把代码硬切成两层，让同一份控制代码既能跑在板子上、又能跑进 JSBSim 仿真里。

```
core/        纯 C++ 控制核，零硬件依赖（无 Arduino/Wire/FreeRTOS/JSBSim）
             ├─ ahrs/    Mahony 6/9DOF AHRS（陀螺+加速度+磁力计）
             ├─ pid/     PID（陀螺 D 项 + 独立 ki + back-calc anti-windup）
             ├─ modes/   Off/Angle/Rate 状态机 + 平滑过渡
             ├─ math/    PT1 等滤波
             └─ types.h  ImuSample / MagSample / BaroSample / ServoCommand …
hal_esp32/   板载硬件胶（PlatformIO + Arduino）：CRSF 解码、传感器驱动、PWM、WiFi、黑匣子
hal_sitl/    PC 仿真胶：JSBSim 桥（ctypes 调 core DLL）、合成磁力计、FlightGear 输出
test/        PC 单元测试（Unity）
```

**分层铁律：** `core/` 里不许出现任何硬件/平台头文件。同一份算法跑板子和 SITL，可单测，是整个测试金字塔的地基。

## 传感器分档（一份固件，按型号开关能力）

| 档位 | 传感器 | 解锁能力 |
|------|--------|----------|
| **base** | 陀螺+加速度+磁力计+气压 | 9 轴 AHRS（陀螺扛转弯 + 加速度门控 + 磁力计锁航向）、Angle/Rate 自稳、高度/航向保持、混控、救命键、G-limit、黑匣子 |
| **plus** | base + GPS | turn-comp（GPS 地速 ω×V）、地速退化空速保持 |
| **pro** | plus + 空速管 | 精确空速保持/失速保护、抗风 turn-comp |
| **max** | 3×陀螺 + 2×磁 + 2×气压 + 空速 + GPS | 传感器表决/单点失效容错 |

软件靠开机探测在线传感器自动判档，少装一个就降级对应功能，不分叉固件。

## 构建与测试

PC 端（控制核 + SITL）：

```bash
cmake --preset dev          # 配置（Ninja + GCC，单配置 Debug）
cmake --build build         # 编译
ctest --test-dir build --output-on-failure   # 单元测试
```

SITL 闭环仿真（需 Python + JSBSim 1.3.1，c172p 模型）：

```bash
cd hal_sitl
python3 run_sitl.py --scenario turn --secs 12 --out ../sitl_turn.csv
python3 check_turn_hold.py ../sitl_turn.csv      # 验收：AHRS 估计跟住真值滚转
```

FlightGear 3D 动画见 `hal_sitl/fg_run.md`。板载固件（ESP32-S3）：`pio run -e weekendpilot_s3`。

## 测试金字塔

```
实机动态飞行   ← 最后，最贵
实机静态/HIL   ← 接舵机+电调地面验证混控、failsafe
JSBSim SITL    ← 闭环仿真调 PID/自动驾驶，FlightGear 看动画
PC 单元测试    ← 测 core 各模块（确定性、CI 可跑）
```

## 参考的开源项目

WeekendPilot 站在成熟开源飞控的肩膀上。下列项目是**算法参考源**——我们阅读其实现、移植/重写干净的逻辑进 `core/`，并非直接编译它们（INAV/Betaflight 是 STM32-only，ArduPilot 的 ESP32 端口为实验性）。各自版权与许可归原作者所有。

| 项目 | 许可 | 我们参考/移植了什么 |
|------|------|--------------------|
| [madflight](https://github.com/qqqlab/madflight) | MIT | Mahony AHRS（`core/ahrs/ahrs_mahony.cpp` 移植自其 `src/ahr/Mahony/Mahony.cpp` 的 `update6DOF`/`update9DOF`）、PID 与加速度幅值门控（0.9–1.1g）默认值、ESP32-S3 整套参考 |
| [PaulStoffregen/MahonyAHRS](https://github.com/PaulStoffregen/MahonyAHRS) | 开源（madflight 的 Mahony 上游） | Mahony 互补滤波四元数公式的原始来源 |
| [Betaflight](https://github.com/betaflight/betaflight) | GPL-3.0 | PT1/biquad 滤波、加速度门控姿态机动判据、Blackbox 黑匣子日志格式 |
| [INAV](https://github.com/iNavFlight/inav) | GPL-3.0 | 固定翼 PID/混控选型、高度保持、配平等导航逻辑参考 |
| [ArduPilot / ArduPlane](https://github.com/ArduPilot/ardupilot) | GPL-3.0 | TECS 能量管理、L1 航迹导引、协调转弯/turn-comp（`ω×V` 离心补偿）、SITL 架构思路 |
| [JSBSim](https://github.com/JSBSim-Team/jsbsim) | LGPL-2.1 | 飞行动力学仿真器（SITL 用 c172p 模型，经 Python API 闭环驱动）|

直接编入仓库的第三方代码：

| 组件 | 许可 | 用途 |
|------|------|------|
| [Unity](https://github.com/ThrowTheSwitch/Unity)（`third_party/unity/`） | MIT | C 单元测试框架 |

> 致谢：AHRS 在协调转弯里靠陀螺扛、加速度幅值门控避免被向心加速度污染的做法，直接借鉴自 Betaflight/madflight 的成熟工程实践（详见设计文档附录 C）。

## 许可

本项目以 **GPL-3.0** 发布（见 [LICENSE](LICENSE)），与所参考的 Betaflight/INAV/ArduPilot 生态兼容。第三方组件保留其各自许可。
