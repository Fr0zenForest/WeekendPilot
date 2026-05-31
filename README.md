# WeekendPilot

跑在 ESP32-S3 上、固件自主可控的**固定翼伴侣飞控**。从接收机（SuperX Nano，CRSF）拿遥控信道，自己做姿态解算、增稳、混控与选择性自动驾驶，输出 PWM 给舵机/电调。

算法不闭门造车——从 INAV / ArduPilot / Betaflight / madflight 等成熟开源飞控**移植经实机验证的逻辑**，重写进干净架构，先在 JSBSim 仿真里调好再上实机。

## 功能一览

- **增稳/自稳/直通** 三模式 + 飞行中感度调节
- **舵机混控**：标准布局 + flaperon / V尾 / elevon
- **过载保护** G-limit
- **9 轴姿态解算**（Mahony AHRS）
- **传感器即插即用**：陀螺/加速度、磁罗盘、气压、GPS、空速管——加新型号只写适配，核心零改动
- **开发中**：定高、航向保持、协调转弯、救命键、空速保持、黑匣子、WiFi 调参

> 完整功能与开发状态见 **[功能说明](docs/WeekendPilot-功能说明.md)**。

## 快速开始

```bash
# PC 端：编译控制核 + 跑单元测试
cmake --preset dev
cmake --build build
ctest --test-dir build --output-on-failure

# 仿真闭环（需 Python + JSBSim）
cd hal_sitl && python3 run_sitl.py --scenario turn --out turn.csv

# 板载固件（ESP32-S3）
pio run -e weekendpilot_s3
```

## 架构

代码硬切两层，同一份控制代码既跑板子又跑仿真：

```
core/        纯 C++ 控制核，零硬件依赖（AHRS / PID / 模式 / 混控 / 传感器解码）
hal_esp32/   板载硬件层（CRSF、I2C 驱动、PWM、NVS）
hal_sitl/    PC 仿真层（JSBSim 桥）
test/        单元测试（Unity）
```

**铁律**：`core/` 不出现任何硬件/平台头文件——所以同一份算法可单测、可仿真、可上板。设计细节见 [设计文档](docs/superpowers/specs/2026-05-30-weekendpilot-design.md)。

## 致谢与许可

站在成熟开源飞控的肩膀上：[madflight](https://github.com/qqqlab/madflight) (MIT, Mahony AHRS)、[Betaflight](https://github.com/betaflight/betaflight)、[INAV](https://github.com/iNavFlight/inav)、[ArduPilot](https://github.com/ArduPilot/ardupilot)（均 GPL-3.0，算法参考）、[JSBSim](https://github.com/JSBSim-Team/jsbsim) (LGPL, 仿真)。各处移植在源码中标注了来源。

本项目以 **GPL-3.0** 发布（见 [LICENSE](LICENSE)）。
