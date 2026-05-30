# WeekendPilot

固定翼伴侣飞控（ESP32-S3）。SuperX Nano 经 CRSF 串口供信道，本固件跑增稳/混控/选择性自动驾驶，输出 PWM。

- 控制核 `core/`：纯 C++，零硬件依赖，板载与 PC SITL 共用。
- `hal_esp32/`：板载（PlatformIO）。
- `hal_sitl/`：PC 仿真（CMake + JSBSim）。

设计与计划见 ElrsRX 仓库 docs/superpowers/。
