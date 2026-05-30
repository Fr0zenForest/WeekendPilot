# WeekendPilot 阶段 0~1 实现计划（骨架 + SITL 直通 + 核心增稳）

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在全新仓库 `C:\Repository\WeekendPilot` 搭好"控制核 + 硬件胶"两层骨架，打通 PC 单测、JSBSim SITL 直通闭环，并移植 Mahony AHRS + PID + Off/Angle/Rate 状态机，在 JSBSim 仿真里飞出可用的增稳。

**Architecture:** 纯 C++ 控制核（`core/`，零硬件依赖）被两个 HAL 喂数据：板载 `hal_esp32/`（PlatformIO）和 PC 仿真 `hal_sitl/`（CMake，桥接 JSBSim Python）。控制核接收 `ControlInput`（通道+IMU+baro+dt）输出 `ServoCommand`。同一份 core 同时跑在 ESP32-S3 和 PC SITL，保证仿真调好的逻辑等价上板。

**Tech Stack:** C++17 | CMake（PC core+SITL+测试）| PlatformIO（ESP32-S3 板载）| Unity（单测，沿用 ElrsRX 约定）| JSBSim 1.3.1 Python API（SITL 动力学）| FlightGear（3D 动画，data_output/flightgear.xml）| 参考源码：madflight（Mahony/PID）、INAV/ArduPlane（控制律）

**设计依据：** `docs/superpowers/specs/2026-05-30-weekendpilot-design.md`（本仓库 WeekendPilot 内）。参考仓库均在 `C:\Repository\` 下：`madflight` `inav` `ardupilot` `betaflight` `jsbsim`。

---

## 文件结构规划

阶段 0~1 结束时 `C:\Repository\WeekendPilot` 的结构（只列本计划涉及的文件）：

```
WeekendPilot/
├── core/                          # 纯 C++ 控制核（零硬件依赖）
│   ├── types.h                    # ControlInput / ServoCommand / ImuSample / BaroSample 等
│   ├── math/
│   │   └── filter_pt1.h           # PT1 一阶低通（header-only）
│   ├── ahrs/
│   │   ├── ahrs_mahony.h
│   │   └── ahrs_mahony.cpp        # Mahony 6 轴互补滤波（移植 madflight）
│   ├── pid/
│   │   ├── pid_controller.h
│   │   └── pid_controller.cpp     # PID（D 用陀螺 + 独立 ki + back-calc anti-windup）
│   ├── modes/
│   │   ├── stab_mode.h            # FlightMode 枚举 + 阈值
│   │   ├── mode_controller.h
│   │   └── mode_controller.cpp    # Off/Angle/Rate 状态机 + 平滑过渡 + 滞回
│   └── controller.h/.cpp          # 顶层：组合 AHRS+模式+PID，update(ControlInput)->ServoCommand
├── hal_sitl/
│   ├── jsbsim_bridge.py           # JSBSim 动力学 <-> core（通过 C 接口或子进程）
│   ├── sitl_main.cpp              # PC 主循环：固定步长喂 core
│   ├── core_c_api.h/.cpp          # core 的 C 包装（供 Python ctypes 调用）
│   └── run_sitl.py                # 跑仿真 + 数据曲线 + 可选 FlightGear
├── hal_esp32/
│   ├── platformio.ini
│   └── src/main.cpp               # CRSF 读通道 -> core -> PWM（阶段 0 仅直通）
├── test/
│   ├── test_pid/test_pid.cpp
│   ├── test_ahrs/test_ahrs.cpp
│   └── test_mode/test_mode.cpp
├── sim/
│   └── aircraft/                  # JSBSim 机型（用自带 c172p 起步）
├── CMakeLists.txt                 # PC：core 静态库 + 测试 + SITL
├── platformio.ini                 # 顶层指向 hal_esp32（板载）
├── .gitignore
└── README.md
```

**分层铁律：** `core/` 下任何文件**禁止** `#include <Arduino.h>` / `Wire.h` / FreeRTOS / JSBSim 头。core 只用标准 C++。CMake 的 `core` target 用 `-Wall -Wextra -Werror` 守住"纯 C++ 可编译"。

---

# 阶段 0：地基（骨架 + 双构建 + SITL 直通闭环）

> 里程碑：仿真和板子两端都能"直通"，core 框架就位，PC 单测可跑。

## Task 0.1: 克隆空仓库 + 目录骨架 + .gitignore

**Files:**
- Create: `C:\Repository\WeekendPilot\.gitignore`
- Create: `C:\Repository\WeekendPilot\README.md`

- [ ] **Step 1: 克隆空仓库**

```bash
cd /c/Repository
git clone https://github.com/Fr0zenForest/WeekendPilot
cd WeekendPilot
```
Expected: 克隆成功（仓库可能为空，git clone 会提示 empty repository 警告，正常）。

- [ ] **Step 2: 建目录骨架**

```bash
cd /c/Repository/WeekendPilot
mkdir -p core/math core/ahrs core/pid core/modes hal_sitl hal_esp32/src test/test_pid test/test_ahrs test/test_mode sim/aircraft
```

- [ ] **Step 3: 写 .gitignore**

```gitignore
# build outputs
/build/
.pio/
*.o
*.a
*.bin
*.elf
# python
__pycache__/
*.pyc
# editor
.vscode/
.idea/
# sim logs
*.csv
*.flightgear.log
```

- [ ] **Step 4: 写 README.md**

```markdown
# WeekendPilot

固定翼伴侣飞控（ESP32-S3）。SuperX Nano 经 CRSF 串口供信道，本固件跑增稳/混控/选择性自动驾驶，输出 PWM。

- 控制核 `core/`：纯 C++，零硬件依赖，板载与 PC SITL 共用。
- `hal_esp32/`：板载（PlatformIO）。
- `hal_sitl/`：PC 仿真（CMake + JSBSim）。

设计与计划见 `docs/superpowers/`。
```

- [ ] **Step 5: Commit**

```bash
cd /c/Repository/WeekendPilot
git add .gitignore README.md
git commit -m "chore: scaffold WeekendPilot repo skeleton"
```

---

## Task 0.2: CMake + Unity 单测地基（先跑通一个空测试）

**Files:**
- Create: `C:\Repository\WeekendPilot\CMakeLists.txt`
- Create: `C:\Repository\WeekendPilot\test\test_smoke\test_smoke.cpp`
- Create: `C:\Repository\WeekendPilot\test\unity\unity.h` 等（见 Step 2）

- [ ] **Step 1: 顶层 CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 3.16)
project(WeekendPilot CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 控制核：纯 C++，严格编译
file(GLOB_RECURSE CORE_SRC core/*.cpp)
add_library(wp_core STATIC ${CORE_SRC})
target_include_directories(wp_core PUBLIC core)
target_compile_options(wp_core PRIVATE -Wall -Wextra -Werror)

enable_testing()
add_subdirectory(test)
```

- [ ] **Step 2: 取 Unity 单测框架**

Unity 是单文件 C 测试框架（ElrsRX 已用）。从已有 ElrsRX 的 PlatformIO 包缓存或 GitHub 取 `unity.c` `unity.h` `unity_internals.h`：

```bash
cd /c/Repository/WeekendPilot
mkdir -p third_party/unity
# 从 PlatformIO 缓存复制（ElrsRX 编译过 native 测试，缓存里有）
find /c/Users/Weekend/.platformio -path "*Unity*/src/unity.c" 2>/dev/null | head -1
# 将找到的 unity.c / unity.h / unity_internals.h 复制到 third_party/unity/
```
若缓存找不到，从 https://github.com/ThrowTheSwitch/Unity 取 `src/unity.c` `src/unity.h` `src/unity_internals.h`。

- [ ] **Step 3: test/CMakeLists.txt**

**Files:** Create `C:\Repository\WeekendPilot\test\CMakeLists.txt`

```cmake
add_library(unity STATIC ${CMAKE_SOURCE_DIR}/third_party/unity/unity.c)
target_include_directories(unity PUBLIC ${CMAKE_SOURCE_DIR}/third_party/unity)

function(wp_add_test name)
  add_executable(${name} ${name}/${name}.cpp)
  target_link_libraries(${name} PRIVATE wp_core unity)
  add_test(NAME ${name} COMMAND ${name})
endfunction()

wp_add_test(test_smoke)
```

- [ ] **Step 4: 写冒烟测试**

**Files:** Create `C:\Repository\WeekendPilot\test\test_smoke\test_smoke.cpp`

```cpp
#include <unity.h>

void setUp() {}
void tearDown() {}

void test_sanity() { TEST_ASSERT_EQUAL_INT(4, 2 + 2); }

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_sanity);
    return UNITY_END();
}
```

需要 core 至少有一个 .cpp 才能建 wp_core 库，先放占位：

**Files:** Create `C:\Repository\WeekendPilot\core\placeholder.cpp`
```cpp
// placeholder so wp_core has a source until real modules land; remove in Task 1.1
namespace wp { int _placeholder() { return 0; } }
```

- [ ] **Step 5: 配置并跑测试**

```bash
cd /c/Repository/WeekendPilot
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```
Expected: `test_smoke` PASS（1 test）。

- [ ] **Step 6: Commit**

```bash
git add CMakeLists.txt test/ third_party/ core/placeholder.cpp
git commit -m "build: add CMake + Unity test harness with smoke test"
```

---

## Task 0.3: 控制核数据类型 types.h

**Files:**
- Create: `C:\Repository\WeekendPilot\core\types.h`
- Test: `C:\Repository\WeekendPilot\test\test_smoke\test_smoke.cpp`（追加一个 include 编译检查）

- [ ] **Step 1: 写 types.h**

```cpp
#pragma once
#include <cstdint>

namespace wp {

// 16 路 RC 通道，标准 CRSF 范围 [1000,2000]，中位 1500
constexpr int kNumChannels = 16;
constexpr int kNumServos = 8;          // 阶段 0~1 固定 8 路

struct ImuSample {
    float gyro_x, gyro_y, gyro_z;      // deg/s（机体系）
    float accel_x, accel_y, accel_z;   // g
    bool  valid = false;
};

struct BaroSample {
    float altitude_m = 0.0f;           // 相对高度（m）
    bool  valid = false;
};

struct ControlInput {
    uint16_t channels[kNumChannels];   // RC us 值 [1000,2000]
    ImuSample imu;
    BaroSample baro;
    float dt;                          // 距上次 update 的秒数
    bool  link_ok = true;              // CRSF 链路有效
};

struct ServoCommand {
    uint16_t servo[kNumServos];        // 输出 us 值 [1000,2000]
};

struct Attitude {
    float roll_deg = 0.0f;
    float pitch_deg = 0.0f;
    float yaw_deg = 0.0f;
};

}  // namespace wp
```

- [ ] **Step 2: 编译检查（追加到冒烟测试）**

在 `test/test_smoke/test_smoke.cpp` 顶部加 `#include "types.h"`，并加一个测试：

```cpp
#include "types.h"
// ... existing includes/setUp/tearDown ...

void test_types_sizes() {
    wp::ControlInput in{};
    in.channels[0] = 1500;
    in.dt = 0.001f;
    TEST_ASSERT_EQUAL_UINT16(1500, in.channels[0]);
    TEST_ASSERT_EQUAL_INT(8, wp::kNumServos);
}
```
在 `main()` 里加 `RUN_TEST(test_types_sizes);`。

- [ ] **Step 3: 跑测试**

```bash
cmake --build build && ctest --test-dir build --output-on-failure
```
Expected: 2 tests PASS。

- [ ] **Step 4: Commit**

```bash
git add core/types.h test/test_smoke/test_smoke.cpp
git commit -m "feat(core): add control I/O data types"
```

---

## Task 0.4: 顶层 controller + C API 包装（直通版）

阶段 0 的 controller 只做**直通**：把通道原样映射到舵机，不加任何修正。这样能先打通 SITL 闭环管道，阶段 1 再往里填增稳。

**Files:**
- Create: `C:\Repository\WeekendPilot\core\controller.h`
- Create: `C:\Repository\WeekendPilot\core\controller.cpp`
- Create: `C:\Repository\WeekendPilot\hal_sitl\core_c_api.h`
- Create: `C:\Repository\WeekendPilot\hal_sitl\core_c_api.cpp`
- Test: `C:\Repository\WeekendPilot\test\test_smoke\test_smoke.cpp`

- [ ] **Step 1: 写 controller.h**

```cpp
#pragma once
#include "types.h"

namespace wp {

class Controller {
public:
    // 阶段 0：纯直通。阶段 1 接入 AHRS/PID/模式。
    ServoCommand update(const ControlInput& in);
};

}  // namespace wp
```

- [ ] **Step 2: 写直通测试（先失败）**

**Files:** Create `C:\Repository\WeekendPilot\test\test_controller\test_controller.cpp`

```cpp
#include <unity.h>
#include "controller.h"

void setUp() {}
void tearDown() {}

void test_passthrough_maps_channels_to_servos() {
    wp::Controller c;
    wp::ControlInput in{};
    in.dt = 0.001f;
    in.link_ok = true;
    for (int i = 0; i < wp::kNumChannels; ++i) in.channels[i] = 1000 + i * 50;
    wp::ServoCommand out = c.update(in);
    for (int i = 0; i < wp::kNumServos; ++i) {
        TEST_ASSERT_EQUAL_UINT16(1000 + i * 50, out.servo[i]);
    }
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_passthrough_maps_channels_to_servos);
    return UNITY_END();
}
```

在 `test/CMakeLists.txt` 末尾加 `wp_add_test(test_controller)`。

- [ ] **Step 3: 跑测试确认失败**

```bash
cmake -S . -B build && cmake --build build 2>&1 | tail -20
```
Expected: 链接失败（`Controller::update` 未定义）。

- [ ] **Step 4: 写 controller.cpp（直通实现）**

```cpp
#include "controller.h"

namespace wp {

ServoCommand Controller::update(const ControlInput& in) {
    ServoCommand out{};
    for (int i = 0; i < kNumServos; ++i) {
        out.servo[i] = in.channels[i];   // 直通：通道 i -> 舵机 i
    }
    return out;
}

}  // namespace wp
```

删除占位文件：
```bash
git rm core/placeholder.cpp
```

- [ ] **Step 5: 跑测试确认通过**

```bash
cmake -S . -B build && cmake --build build && ctest --test-dir build --output-on-failure
```
Expected: 所有测试 PASS（含 test_controller）。

- [ ] **Step 6: 写 C API 包装（供 Python ctypes 调）**

**core_c_api.h:**
```cpp
#pragma once
#ifdef __cplusplus
extern "C" {
#endif

// 不透明句柄
void* wp_controller_create();
void  wp_controller_destroy(void* handle);

// 一次控制循环：
//   channels: 16 个 uint16 [1000,2000]
//   imu: [gx,gy,gz (deg/s), ax,ay,az (g)]
//   baro_alt_m, dt_s, link_ok
//   servos_out: 调用方提供长度 8 的 uint16 缓冲，函数填充
void wp_controller_update(void* handle,
                          const unsigned short* channels,
                          const float* imu6,
                          float baro_alt_m, int baro_valid,
                          float dt_s, int link_ok,
                          unsigned short* servos_out);

#ifdef __cplusplus
}
#endif
```

**core_c_api.cpp:**
```cpp
#include "core_c_api.h"
#include "controller.h"
#include <cstring>

extern "C" {

void* wp_controller_create() { return new wp::Controller(); }
void  wp_controller_destroy(void* h) { delete static_cast<wp::Controller*>(h); }

void wp_controller_update(void* h,
                          const unsigned short* channels,
                          const float* imu6,
                          float baro_alt_m, int baro_valid,
                          float dt_s, int link_ok,
                          unsigned short* servos_out) {
    auto* c = static_cast<wp::Controller*>(h);
    wp::ControlInput in{};
    for (int i = 0; i < wp::kNumChannels; ++i) in.channels[i] = channels[i];
    in.imu.gyro_x = imu6[0]; in.imu.gyro_y = imu6[1]; in.imu.gyro_z = imu6[2];
    in.imu.accel_x = imu6[3]; in.imu.accel_y = imu6[4]; in.imu.accel_z = imu6[5];
    in.imu.valid = true;
    in.baro.altitude_m = baro_alt_m; in.baro.valid = (baro_valid != 0);
    in.dt = dt_s; in.link_ok = (link_ok != 0);
    wp::ServoCommand out = c->update(in);
    std::memcpy(servos_out, out.servo, sizeof(out.servo));
}

}  // extern "C"
```

- [ ] **Step 7: 把 C API 编进共享库（CMake）**

在顶层 `CMakeLists.txt` 末尾追加：
```cmake
# SITL 共享库：供 Python ctypes 加载
add_library(wp_core_capi SHARED hal_sitl/core_c_api.cpp)
target_link_libraries(wp_core_capi PRIVATE wp_core)
target_include_directories(wp_core_capi PRIVATE core hal_sitl)
```

```bash
cmake -S . -B build && cmake --build build
ls build/*wp_core_capi* 2>/dev/null || ls build/**/*wp_core_capi* 2>/dev/null
```
Expected: 生成 `wp_core_capi.dll`（Windows）。

- [ ] **Step 8: Commit**

```bash
git add core/controller.h core/controller.cpp hal_sitl/core_c_api.h hal_sitl/core_c_api.cpp CMakeLists.txt test/
git rm core/placeholder.cpp
git commit -m "feat(core): passthrough controller + C API for SITL"
```

---

## Task 0.5: JSBSim 桥 + SITL 直通闭环

把 JSBSim 当动力学引擎：每个时间步读飞机状态 → 喂给 core（ctypes 调 `wp_core_capi.dll`）→ core 输出舵机 → 写回 JSBSim 作动器。阶段 0 core 是直通，所以这里先验证"管道通 + 飞机能被舵量驱动"。

> **单位换算（重要，已用真实 c172p 验证）：** JSBSim 陀螺 `velocities/{p,q,r}-rad_sec` 是 rad/s → 我们的 `ImuSample` 要 deg/s（×57.2958）。`accelerations/N{x,y,z}` 是 ft/s² → 要 g（÷32.174）。舵机 us[1000,2000] → JSBSim `cmd-norm`[-1,1]（油门[0,1]）。

**Files:**
- Create: `C:\Repository\WeekendPilot\hal_sitl\jsbsim_bridge.py`
- Create: `C:\Repository\WeekendPilot\hal_sitl\run_sitl.py`

- [ ] **Step 1: 写 jsbsim_bridge.py（core 的 ctypes 封装 + 单位换算）**

```python
"""WeekendPilot SITL bridge: JSBSim dynamics <-> C++ control core via ctypes."""
import ctypes, os, math

RAD2DEG = 57.29577951308232
FT_S2_TO_G = 1.0 / 32.174

class CoreController:
    def __init__(self, dll_path):
        self.lib = ctypes.CDLL(dll_path)
        self.lib.wp_controller_create.restype = ctypes.c_void_p
        self.lib.wp_controller_update.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_uint16),
            ctypes.POINTER(ctypes.c_float),
            ctypes.c_float, ctypes.c_int,
            ctypes.c_float, ctypes.c_int,
            ctypes.POINTER(ctypes.c_uint16),
        ]
        self.handle = self.lib.wp_controller_create()

    def update(self, channels16, imu6, baro_alt_m, baro_valid, dt_s, link_ok):
        ch = (ctypes.c_uint16 * 16)(*channels16)
        imu = (ctypes.c_float * 6)(*imu6)
        out = (ctypes.c_uint16 * 8)()
        self.lib.wp_controller_update(self.handle, ch, imu,
                                      ctypes.c_float(baro_alt_m), int(baro_valid),
                                      ctypes.c_float(dt_s), int(link_ok), out)
        return list(out)

    def __del__(self):
        if getattr(self, 'handle', None):
            self.lib.wp_controller_destroy(self.handle)


def read_imu(fdm):
    """Return [gx,gy,gz deg/s, ax,ay,az g] from JSBSim."""
    gx = fdm.get_property_value('velocities/p-rad_sec') * RAD2DEG
    gy = fdm.get_property_value('velocities/q-rad_sec') * RAD2DEG
    gz = fdm.get_property_value('velocities/r-rad_sec') * RAD2DEG
    ax = fdm.get_property_value('accelerations/Nx') * FT_S2_TO_G
    ay = fdm.get_property_value('accelerations/Ny') * FT_S2_TO_G
    az = fdm.get_property_value('accelerations/Nz') * FT_S2_TO_G
    return [gx, gy, gz, ax, ay, az]


def write_servos(fdm, servos8):
    """Map servo us[1000,2000] -> JSBSim cmd-norm.
       servo[0]=aileron, servo[1]=elevator, servo[2]=throttle, servo[3]=rudder."""
    def norm(us):
        return max(-1.0, min(1.0, (us - 1500) / 500.0))
    def unit(us):
        return max(0.0, min(1.0, (us - 1000) / 1000.0))
    fdm.set_property_value('fcs/aileron-cmd-norm',  norm(servos8[0]))
    fdm.set_property_value('fcs/elevator-cmd-norm', norm(servos8[1]))
    fdm.set_property_value('fcs/throttle-cmd-norm[0]', unit(servos8[2]))
    fdm.set_property_value('fcs/rudder-cmd-norm',   norm(servos8[3]))
```

- [ ] **Step 2: 写 run_sitl.py（闭环主循环）**

```python
"""Run a closed-loop SITL: scripted RC -> core -> JSBSim, log state to CSV."""
import jsbsim, os, csv, argparse
from jsbsim_bridge import CoreController, read_imu, write_servos

JSBSIM_ROOT = 'C:/Repository/jsbsim'
DLL = os.path.join(os.path.dirname(__file__), '..', 'build', 'wp_core_capi.dll')

def make_fdm(dt):
    fdm = jsbsim.FGFDMExec(None)
    fdm.set_aircraft_path(JSBSIM_ROOT + '/aircraft')
    fdm.set_engine_path(JSBSIM_ROOT + '/engine')
    fdm.set_systems_path(JSBSIM_ROOT + '/systems')
    fdm.load_model('c172p')
    fdm.set_dt(dt)
    fdm.set_property_value('ic/h-agl-ft', 1000)
    fdm.set_property_value('ic/vc-kts', 90)
    fdm.set_property_value('ic/gamma-deg', 0)
    fdm.run_ic()
    return fdm

def rc_for_time(t):
    """Scripted RC. channels us[1000,2000]. ch0 roll, ch1 pitch, ch2 throttle, ch3 yaw."""
    ch = [1500] * 16
    ch[2] = 1700
    if 2.0 < t < 4.0:
        ch[0] = 1650
    return ch

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dt', type=float, default=0.01)
    ap.add_argument('--secs', type=float, default=10.0)
    ap.add_argument('--out', default='sitl_log.csv')
    args = ap.parse_args()

    fdm = make_fdm(args.dt)
    core = CoreController(os.path.abspath(DLL))

    with open(args.out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t','roll','pitch','yaw','alt_ft','vc_kts','srv0','srv1','srv2','srv3'])
        t = 0.0
        while t < args.secs:
            imu6 = read_imu(fdm)
            alt_m = fdm.get_property_value('position/h-agl-ft') * 0.3048
            servos = core.update(rc_for_time(t), imu6, alt_m, 1, args.dt, 1)
            write_servos(fdm, servos)
            fdm.run()
            t = fdm.get_sim_time()
            w.writerow([round(t,3),
                        round(fdm.get_property_value('attitude/phi-deg'),2),
                        round(fdm.get_property_value('attitude/theta-deg'),2),
                        round(fdm.get_property_value('attitude/psi-deg'),2),
                        round(fdm.get_property_value('position/h-agl-ft'),1),
                        round(fdm.get_property_value('velocities/vc-kts'),1),
                        servos[0], servos[1], servos[2], servos[3]])
    print('wrote', args.out)

if __name__ == '__main__':
    main()
```

- [ ] **Step 3: 跑闭环（用 pyenv 3.13.3 的 python）**

```bash
cd /c/Repository/WeekendPilot
PY="C:/Users/Weekend/.pyenv/pyenv-win/versions/3.13.3/python.exe"
"$PY" hal_sitl/run_sitl.py --secs 10 --out sitl_log.csv
```
Expected: 打印 `wrote sitl_log.csv`。直通模式下 ch0 在 2~4s 打到 1650（aileron），飞机 roll 应随之偏离 0。

- [ ] **Step 4: 健全性检查（管道验证）**

```bash
"$PY" -c "
import csv
rows = list(csv.DictReader(open('sitl_log.csv')))
roll_during = max(abs(float(r['roll'])) for r in rows if 2.0 < float(r['t']) < 5.0)
print('max abs roll during aileron pulse:', round(roll_during,2), 'deg')
assert roll_during > 2.0, 'aileron pulse did not roll aircraft - pipeline broken'
print('SITL passthrough pipeline OK')
"
```
Expected: `SITL passthrough pipeline OK`。

- [ ] **Step 5: Commit**

```bash
git add hal_sitl/jsbsim_bridge.py hal_sitl/run_sitl.py
git commit -m "feat(sitl): JSBSim closed-loop bridge with passthrough verification"
```

---

## Task 0.6: ESP32-S3 板载骨架 — CRSF 读通道直通 PWM

板载阶段 0 目标：从 SuperX 串口读 CRSF 16 通道，直通到 8 路 PWM，等同普通接收机（failsafe 基线）。**不接 IMU/增稳**，先打通"信道进、PWM 出"。

> **安全说明：** 这是无线输入驱动物理舵机的链路。阶段 0 仅直通 + failsafe（链路丢失→保持中位），无增稳叠加，符合"任何故障退直通"的安全底线。

**Files:**
- Create: `C:\Repository\WeekendPilot\platformio.ini`
- Create: `C:\Repository\WeekendPilot\hal_esp32\src\main.cpp`

- [ ] **Step 1: 写 platformio.ini**

```ini
[platformio]
src_dir = hal_esp32/src

[env:weekendpilot_s3]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino
build_flags =
    -std=gnu++17
    -I core
    -DCORE_DEBUG_LEVEL=3
monitor_speed = 420000
```

- [ ] **Step 2: 写 main.cpp（CRSF 直通骨架）**

```cpp
#include <Arduino.h>
#include "types.h"
#include "controller.h"

static const int kServoPins[wp::kNumServos] = {1, 2, 8, 9, 10, 15, 16, 17};
static const int kCrsfRxPin = 44;
static const int kCrsfTxPin = 43;

static const uint8_t CRSF_ADDR = 0xC8;
static const uint8_t CRSF_FRAMETYPE_RC = 0x16;

wp::Controller g_controller;
uint8_t  g_buf[64];
uint16_t g_channels[wp::kNumChannels];
uint32_t g_lastRcMs = 0;

void setupPwm() {
    for (int i = 0; i < wp::kNumServos; ++i) {
        ledcSetup(i, 50, 16);
        ledcAttachPin(kServoPins[i], i);
        ledcWrite(i, (uint32_t)(1500.0 / 20000.0 * 65535));
    }
}

void writeServoUs(int ch, uint16_t us) {
    if (us < 1000) us = 1000;
    if (us > 2000) us = 2000;
    ledcWrite(ch, (uint32_t)((double)us / 20000.0 * 65535));
}

void parseCrsfRc(const uint8_t* p) {
    const uint8_t* d = p + 3;
    uint32_t bits = 0;
    int bitcnt = 0, ch = 0;
    for (int i = 0; i < 22 && ch < 16; ++i) {
        bits |= (uint32_t)d[i] << bitcnt;
        bitcnt += 8;
        while (bitcnt >= 11 && ch < 16) {
            uint16_t raw = bits & 0x7FF;
            bits >>= 11;
            bitcnt -= 11;
            g_channels[ch++] = (uint16_t)(raw * 0.62477f + 881);
        }
    }
    g_lastRcMs = millis();
}

void setup() {
    Serial.begin(115200);
    Serial1.begin(420000, SERIAL_8N1, kCrsfRxPin, kCrsfTxPin);
    for (int i = 0; i < wp::kNumChannels; ++i) g_channels[i] = 1500;
    setupPwm();
}

void loop() {
    static int idx = 0;
    static int len = 0;
    while (Serial1.available()) {
        uint8_t b = Serial1.read();
        if (idx == 0) {
            if (b == CRSF_ADDR) g_buf[idx++] = b;
        } else if (idx == 1) {
            len = b;
            g_buf[idx++] = b;
        } else {
            g_buf[idx++] = b;
            if (idx >= len + 2) {
                if (g_buf[2] == CRSF_FRAMETYPE_RC) parseCrsfRc(g_buf);
                idx = 0;
                len = 0;
            }
            if (idx >= (int)sizeof(g_buf)) { idx = 0; len = 0; }
        }
    }

    bool link_ok = (millis() - g_lastRcMs) < 500;

    wp::ControlInput in{};
    for (int i = 0; i < wp::kNumChannels; ++i)
        in.channels[i] = link_ok ? g_channels[i] : 1500;
    in.dt = 0.001f;
    in.link_ok = link_ok;
    in.imu.valid = false;

    wp::ServoCommand out = g_controller.update(in);
    for (int i = 0; i < wp::kNumServos; ++i) writeServoUs(i, out.servo[i]);
    delay(2);
}
```

> **循环频率说明（与 spec §4 的 1kHz 目标的关系）：** 阶段 0~1 板载用 `delay(2)`（~500Hz）、SITL 用 100Hz 步长，足够验证架构与增稳逻辑正确性。spec 要求的 **1kHz 定时器/FreeRTOS 任务调度留到阶段 2**（接入 IMU 实时采样时一起做）。core 的算法与 dt 解耦，提频不改 core。

- [ ] **Step 3: 编译板载固件**

```bash
cd /c/Repository/WeekendPilot
pio run -e weekendpilot_s3 2>&1 | tail -15
```
Expected: 编译成功，生成 `.pio/build/weekendpilot_s3/firmware.bin`。

> 阶段 0 不要求上板实测（等接好 SuperX + 舵机）。这里验证**板载固件能编译且 core 与板载共享 types.h/controller**——证明两层架构在两个工具链下都成立。

- [ ] **Step 4: Commit**

```bash
git add platformio.ini hal_esp32/src/main.cpp
git commit -m "feat(esp32): CRSF-to-PWM passthrough skeleton (failsafe baseline)"
```

> **阶段 0 里程碑：** SITL 闭环管道通（舵量驱动 JSBSim 飞机）+ 板载固件编译通过 + core 两端共享 + PC 单测绿。

---

# 阶段 1：核心增稳（Mahony AHRS + PID + Off/Angle/Rate）

> 里程碑：仿真里飞机能自稳/增稳/调感度，FlightGear 看动画确认控制方向对。
>
> 默认 PID 值来自 madflight（保守值），轴向/符号约定见各任务。所有移植代码在文件头注明来源（madflight MIT License）。

## Task 1.1: PT1 低通滤波器（header-only）

**Files:**
- Create: `C:\Repository\WeekendPilot\core\math\filter_pt1.h`
- Test: `C:\Repository\WeekendPilot\test\test_filter\test_filter.cpp`

- [ ] **Step 1: 写失败测试**

**Files:** Create `C:\Repository\WeekendPilot\test\test_filter\test_filter.cpp`

```cpp
#include <unity.h>
#include "math/filter_pt1.h"

void setUp() {}
void tearDown() {}

void test_pt1_converges_to_step() {
    wp::FilterPt1 f(10.0f);   // 10Hz cutoff
    float y = 0.0f;
    for (int i = 0; i < 1000; ++i) y = f.apply(1.0f, 0.001f);  // 1s @ 1kHz
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, y);   // settles near step value
}

void test_pt1_first_sample_attenuated() {
    wp::FilterPt1 f(10.0f);
    float y = f.apply(1.0f, 0.001f);
    TEST_ASSERT_TRUE(y > 0.0f && y < 0.2f);     // one 1ms sample: small fraction
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_pt1_converges_to_step);
    RUN_TEST(test_pt1_first_sample_attenuated);
    return UNITY_END();
}
```
在 `test/CMakeLists.txt` 加 `wp_add_test(test_filter)`。

- [ ] **Step 2: 跑测试确认失败**

```bash
cmake -S . -B build && cmake --build build 2>&1 | tail -10
```
Expected: 编译失败（`filter_pt1.h` 不存在）。

- [ ] **Step 3: 写 filter_pt1.h**

```cpp
#pragma once
#include <cmath>

namespace wp {

// 一阶低通（PT1）。参考 Betaflight/iNav pt1FilterApply。
class FilterPt1 {
public:
    explicit FilterPt1(float cutoff_hz) : cutoff_hz_(cutoff_hz), state_(0.0f) {}

    void setCutoff(float cutoff_hz) { cutoff_hz_ = cutoff_hz; }
    void reset(float v = 0.0f) { state_ = v; }

    float apply(float input, float dt) {
        // RC = 1/(2*pi*fc); alpha = dt/(RC+dt)
        const float rc = 1.0f / (2.0f * 3.14159265358979f * cutoff_hz_);
        const float alpha = dt / (rc + dt);
        state_ += alpha * (input - state_);
        return state_;
    }
    float value() const { return state_; }

private:
    float cutoff_hz_;
    float state_;
};

}  // namespace wp
```

- [ ] **Step 4: 跑测试确认通过**

```bash
cmake --build build && ctest --test-dir build --output-on-failure -R test_filter
```
Expected: 2 tests PASS。

- [ ] **Step 5: Commit**

```bash
git add core/math/filter_pt1.h test/test_filter/ test/CMakeLists.txt
git commit -m "feat(core): add PT1 low-pass filter"
```

---

## Task 1.2: PID 控制器（D 用陀螺 + 独立 ki + back-calc anti-windup）

设计依据 spec §5.2：D 项直接用陀螺角速率（非误差微分），符号为负始终对抗运动；ki 与原始积分量分离存储（改 ki 不丢配平）；限幅用反算策略。

**Files:**
- Create: `C:\Repository\WeekendPilot\core\pid\pid_controller.h`
- Create: `C:\Repository\WeekendPilot\core\pid\pid_controller.cpp`
- Test: `C:\Repository\WeekendPilot\test\test_pid\test_pid.cpp`

- [ ] **Step 1: 写 pid_controller.h**

```cpp
#pragma once

namespace wp {

struct PidGains {
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
};

// 输出限幅在 [-out_limit, out_limit]（默认 1.0，归一化舵量）。
class PidController {
public:
    explicit PidController(float out_limit = 1.0f) : out_limit_(out_limit) {}

    void setGains(const PidGains& g) { gains_ = g; }
    const PidGains& gains() const { return gains_; }

    // setpoint/measurement 同单位（角度或角速率）；gyro_rate 用于 D 项（deg/s）。
    // 返回归一化舵量 [-out_limit, out_limit]。
    float update(float setpoint, float measurement, float gyro_rate, float dt);

    void reset();
    float integralRaw() const { return integral_raw_; }

private:
    PidGains gains_;
    float out_limit_;
    float integral_raw_ = 0.0f;   // 与 ki 分离的原始积分量
};

}  // namespace wp
```

- [ ] **Step 2: 写失败测试**

**Files:** Create `C:\Repository\WeekendPilot\test\test_pid\test_pid.cpp`

```cpp
#include <unity.h>
#include "pid/pid_controller.h"

void setUp() {}
void tearDown() {}

void test_p_only_proportional_to_error() {
    wp::PidController pid;
    pid.setGains({0.01f, 0.0f, 0.0f});
    // error = 50 - 0 = 50; gyro 0; out = 0.01*50 = 0.5
    float out = pid.update(50.0f, 0.0f, 0.0f, 0.001f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.5f, out);
}

void test_d_opposes_gyro_motion() {
    wp::PidController pid;
    pid.setGains({0.0f, 0.0f, 0.01f});
    // error 0; gyro +100 dps; D = -kd*gyro = -1.0 -> clamp to -1
    float out = pid.update(0.0f, 0.0f, 100.0f, 0.001f);
    TEST_ASSERT_TRUE(out < 0.0f);
}

void test_i_accumulates_and_ki_scales_live() {
    wp::PidController pid;
    pid.setGains({0.0f, 1.0f, 0.0f});
    for (int i = 0; i < 10; ++i) pid.update(1.0f, 0.0f, 0.0f, 0.1f);  // 1s of error=1
    float raw = pid.integralRaw();
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 1.0f, raw);   // integral_raw ~ error*time = 1
    // set ki=0 -> I contribution zero but raw retained
    pid.setGains({0.0f, 0.0f, 0.0f});
    float out = pid.update(0.0f, 0.0f, 0.0f, 0.001f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, out);
    TEST_ASSERT_TRUE(pid.integralRaw() > 0.5f);   // raw still there
}

void test_back_calc_antiwindup_caps_raw() {
    wp::PidController pid;
    pid.setGains({0.0f, 1.0f, 0.0f});
    for (int i = 0; i < 1000; ++i) pid.update(10.0f, 0.0f, 0.0f, 0.1f);  // huge sustained error
    float out = pid.update(10.0f, 0.0f, 0.0f, 0.1f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.0f, out);   // output clamped at +1
    // integral_raw should not exceed what maps to the limit (ki=1 -> raw<=1)
    TEST_ASSERT_TRUE(pid.integralRaw() <= 1.0f + 1e-3f);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_p_only_proportional_to_error);
    RUN_TEST(test_d_opposes_gyro_motion);
    RUN_TEST(test_i_accumulates_and_ki_scales_live);
    RUN_TEST(test_back_calc_antiwindup_caps_raw);
    return UNITY_END();
}
```
在 `test/CMakeLists.txt` 加 `wp_add_test(test_pid)`。

- [ ] **Step 3: 跑测试确认失败**

```bash
cmake -S . -B build && cmake --build build 2>&1 | tail -10
```
Expected: 链接失败（`PidController::update` / `reset` 未定义）。

- [ ] **Step 4: 写 pid_controller.cpp**

```cpp
#include "pid/pid_controller.h"

namespace wp {

void PidController::reset() {
    integral_raw_ = 0.0f;
}

float PidController::update(float setpoint, float measurement, float gyro_rate, float dt) {
    const float error = setpoint - measurement;

    // 积分原始量累积（与 ki 分离）
    integral_raw_ += error * dt;

    const float p = gains_.kp * error;
    const float i = gains_.ki * integral_raw_;
    const float d = -gains_.kd * gyro_rate;   // D 用陀螺，符号为负，对抗运动

    float out = p + i + d;

    // back-calculation anti-windup：输出超限时把 integral_raw 回缩到
    // "刚好让输出落在限幅边界"对应的值。
    if (out > out_limit_) {
        if (gains_.ki > 0.0f) {
            integral_raw_ = (out_limit_ - p - d) / gains_.ki;
        }
        out = out_limit_;
    } else if (out < -out_limit_) {
        if (gains_.ki > 0.0f) {
            integral_raw_ = (-out_limit_ - p - d) / gains_.ki;
        }
        out = -out_limit_;
    }
    return out;
}

}  // namespace wp
```

- [ ] **Step 5: 跑测试确认通过**

```bash
cmake --build build && ctest --test-dir build --output-on-failure -R test_pid
```
Expected: 4 tests PASS。

- [ ] **Step 6: Commit**

```bash
git add core/pid/ test/test_pid/ test/CMakeLists.txt
git commit -m "feat(core): PID controller with gyro-D, separate ki, back-calc anti-windup"
```

---

## Task 1.3: Mahony AHRS（移植 madflight，6DOF）

移植 madflight `src/ahr/Mahony/Mahony.cpp`（MIT，源自 PaulStoffregen/MahonyAHRS）。我们的封装：输入 deg/s + g，内部转 rad/s，输出 `Attitude`（欧拉角 deg）。只做 6DOF（无磁力计）。

**Files:**
- Create: `C:\Repository\WeekendPilot\core\ahrs\ahrs_mahony.h`
- Create: `C:\Repository\WeekendPilot\core\ahrs\ahrs_mahony.cpp`
- Test: `C:\Repository\WeekendPilot\test\test_ahrs\test_ahrs.cpp`

- [ ] **Step 1: 写 ahrs_mahony.h**

```cpp
#pragma once
#include "types.h"

namespace wp {

// Mahony 6DOF 互补滤波。输入陀螺 deg/s、加速度 g，输出欧拉角 deg。
// 移植自 madflight (MIT) / PaulStoffregen MahonyAHRS。
class AhrsMahony {
public:
    void setKp(float two_kp) { two_kp_ = two_kp; }
    void setKi(float two_ki) { two_ki_ = two_ki; }
    void reset();

    // gyro deg/s, accel g, dt seconds
    void update(const ImuSample& imu, float dt);
    Attitude attitude() const { return att_; }

private:
    float q0_ = 1.0f, q1_ = 0.0f, q2_ = 0.0f, q3_ = 0.0f;
    float ifb_x_ = 0.0f, ifb_y_ = 0.0f, ifb_z_ = 0.0f;
    float two_kp_ = 2.0f * 0.5f;
    float two_ki_ = 2.0f * 0.0f;
    Attitude att_;
    void quaternionToEuler();
};

}  // namespace wp
```

- [ ] **Step 2: 写失败测试**

**Files:** Create `C:\Repository\WeekendPilot\test\test_ahrs\test_ahrs.cpp`

```cpp
#include <unity.h>
#include "ahrs/ahrs_mahony.h"

void setUp() {}
void tearDown() {}

static wp::ImuSample level_imu() {
    wp::ImuSample s{};
    s.gyro_x = s.gyro_y = s.gyro_z = 0.0f;
    s.accel_x = 0.0f; s.accel_y = 0.0f; s.accel_z = 1.0f;  // 1g down -> level
    s.valid = true;
    return s;
}

void test_level_converges_to_zero_roll_pitch() {
    wp::AhrsMahony ahrs;
    wp::ImuSample s = level_imu();
    for (int i = 0; i < 5000; ++i) ahrs.update(s, 0.001f);  // 5s settle
    wp::Attitude a = ahrs.attitude();
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, a.roll_deg);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, a.pitch_deg);
}

void test_roll_right_accel_gives_positive_roll() {
    wp::AhrsMahony ahrs;
    wp::ImuSample s{};
    s.valid = true;
    // banked right ~30deg: gravity vector tilts -> ay component
    s.accel_x = 0.0f; s.accel_y = 0.5f; s.accel_z = 0.866f;
    for (int i = 0; i < 5000; ++i) ahrs.update(s, 0.001f);
    wp::Attitude a = ahrs.attitude();
    TEST_ASSERT_TRUE(a.roll_deg > 20.0f && a.roll_deg < 40.0f);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_level_converges_to_zero_roll_pitch);
    RUN_TEST(test_roll_right_accel_gives_positive_roll);
    return UNITY_END();
}
```
在 `test/CMakeLists.txt` 加 `wp_add_test(test_ahrs)`。

- [ ] **Step 3: 跑测试确认失败**

```bash
cmake -S . -B build && cmake --build build 2>&1 | tail -10
```
Expected: 链接失败（AhrsMahony 方法未定义）。

- [ ] **Step 4: 写 ahrs_mahony.cpp**

```cpp
// Ported from madflight src/ahr/Mahony/Mahony.cpp (MIT License),
// originally PaulStoffregen/MahonyAHRS. Adapted: deg/s input, euler output.
#include "ahrs/ahrs_mahony.h"
#include <cmath>

namespace wp {

static constexpr float DEG2RAD = 0.017453292519943295f;
static constexpr float RAD2DEG = 57.29577951308232f;

void AhrsMahony::reset() {
    q0_ = 1.0f; q1_ = q2_ = q3_ = 0.0f;
    ifb_x_ = ifb_y_ = ifb_z_ = 0.0f;
    att_ = Attitude{};
}

void AhrsMahony::update(const ImuSample& imu, float dt) {
    float gx = imu.gyro_x * DEG2RAD;
    float gy = imu.gyro_y * DEG2RAD;
    float gz = imu.gyro_z * DEG2RAD;
    float ax = imu.accel_x, ay = imu.accel_y, az = imu.accel_z;

    const float alen2 = ax * ax + ay * ay + az * az;
    if (alen2 > 0.0f) {
        const float recipNorm = 1.0f / std::sqrt(alen2);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        const float halfvx = q1_ * q3_ - q0_ * q2_;
        const float halfvy = q0_ * q1_ + q2_ * q3_;
        const float halfvz = q0_ * q0_ - 0.5f + q3_ * q3_;

        const float halfex = (ay * halfvz - az * halfvy);
        const float halfey = (az * halfvx - ax * halfvz);
        const float halfez = (ax * halfvy - ay * halfvx);

        if (two_ki_ > 0.0f) {
            ifb_x_ += two_ki_ * halfex * dt;
            ifb_y_ += two_ki_ * halfey * dt;
            ifb_z_ += two_ki_ * halfez * dt;
            gx += ifb_x_; gy += ifb_y_; gz += ifb_z_;
        } else {
            ifb_x_ = ifb_y_ = ifb_z_ = 0.0f;
        }

        gx += two_kp_ * halfex;
        gy += two_kp_ * halfey;
        gz += two_kp_ * halfez;
    }

    gx *= 0.5f * dt; gy *= 0.5f * dt; gz *= 0.5f * dt;
    const float qa = q0_, qb = q1_, qc = q2_;
    q0_ += (-qb * gx - qc * gy - q3_ * gz);
    q1_ += ( qa * gx + qc * gz - q3_ * gy);
    q2_ += ( qa * gy - qb * gz + q3_ * gx);
    q3_ += ( qa * gz + qb * gy - qc * gx);

    const float recipNorm = 1.0f / std::sqrt(q0_*q0_ + q1_*q1_ + q2_*q2_ + q3_*q3_);
    q0_ *= recipNorm; q1_ *= recipNorm; q2_ *= recipNorm; q3_ *= recipNorm;

    quaternionToEuler();
}

void AhrsMahony::quaternionToEuler() {
    // standard aerospace ZYX
    const float sinr = 2.0f * (q0_ * q1_ + q2_ * q3_);
    const float cosr = 1.0f - 2.0f * (q1_ * q1_ + q2_ * q2_);
    att_.roll_deg = std::atan2(sinr, cosr) * RAD2DEG;

    float sinp = 2.0f * (q0_ * q2_ - q3_ * q1_);
    if (sinp > 1.0f) sinp = 1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    att_.pitch_deg = std::asin(sinp) * RAD2DEG;

    const float siny = 2.0f * (q0_ * q3_ + q1_ * q2_);
    const float cosy = 1.0f - 2.0f * (q2_ * q2_ + q3_ * q3_);
    att_.yaw_deg = std::atan2(siny, cosy) * RAD2DEG;
}

}  // namespace wp
```

- [ ] **Step 5: 跑测试确认通过**

```bash
cmake --build build && ctest --test-dir build --output-on-failure -R test_ahrs
```
Expected: 2 tests PASS。

- [ ] **Step 6: Commit**

```bash
git add core/ahrs/ test/test_ahrs/ test/CMakeLists.txt
git commit -m "feat(core): Mahony 6DOF AHRS (ported from madflight, MIT)"
```

---

## Task 1.4: 飞行模式定义 + 通道映射工具

先定义模式枚举、阈值、摇杆映射的纯函数（易测），下一任务再组装状态机。

**Files:**
- Create: `C:\Repository\WeekendPilot\core\modes\stab_mode.h`
- Test: `C:\Repository\WeekendPilot\test\test_mode\test_mode.cpp`

- [ ] **Step 1: 写 stab_mode.h**

```cpp
#pragma once
#include <cstdint>

namespace wp {

enum class FlightMode : uint8_t { Off = 0, Angle = 1, Rate = 2 };

// 三段开关阈值（CRSF us）。<1300 Off，1300~1700 Angle，>1700 Rate。
constexpr uint16_t kModeAngleThresh = 1300;
constexpr uint16_t kModeRateThresh  = 1700;

inline FlightMode modeFromChannel(uint16_t us) {
    if (us > kModeRateThresh) return FlightMode::Rate;
    if (us > kModeAngleThresh) return FlightMode::Angle;
    return FlightMode::Off;
}

// 通道 us[1000,2000] 中位 1500 -> 归一化 [-1,1]
inline float channelToNorm(uint16_t us) {
    float n = (static_cast<float>(us) - 1500.0f) / 500.0f;
    if (n > 1.0f) n = 1.0f;
    if (n < -1.0f) n = -1.0f;
    return n;
}

// 归一化舵量 [-1,1] -> us[1000,2000]
inline uint16_t normToServoUs(float n) {
    if (n > 1.0f) n = 1.0f;
    if (n < -1.0f) n = -1.0f;
    return static_cast<uint16_t>(1500.0f + n * 500.0f);
}

// 增益通道 us[1000,2000] -> [0,1]
inline float gainFromChannel(uint16_t us) {
    float g = (static_cast<float>(us) - 1000.0f) / 1000.0f;
    if (g > 1.0f) g = 1.0f;
    if (g < 0.0f) g = 0.0f;
    return g;
}

}  // namespace wp
```

- [ ] **Step 2: 写测试**

**Files:** Create `C:\Repository\WeekendPilot\test\test_mode\test_mode.cpp`

```cpp
#include <unity.h>
#include "modes/stab_mode.h"

void setUp() {}
void tearDown() {}

void test_mode_thresholds() {
    TEST_ASSERT_EQUAL_INT((int)wp::FlightMode::Off,   (int)wp::modeFromChannel(1000));
    TEST_ASSERT_EQUAL_INT((int)wp::FlightMode::Angle, (int)wp::modeFromChannel(1500));
    TEST_ASSERT_EQUAL_INT((int)wp::FlightMode::Rate,  (int)wp::modeFromChannel(2000));
}

void test_channel_norm_roundtrip() {
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, wp::channelToNorm(1500));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.0f, wp::channelToNorm(2000));
    TEST_ASSERT_EQUAL_UINT16(1500, wp::normToServoUs(0.0f));
    TEST_ASSERT_EQUAL_UINT16(2000, wp::normToServoUs(1.0f));
}

void test_gain_channel() {
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, wp::gainFromChannel(1000));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.0f, wp::gainFromChannel(2000));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.5f, wp::gainFromChannel(1500));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_mode_thresholds);
    RUN_TEST(test_channel_norm_roundtrip);
    RUN_TEST(test_gain_channel);
    return UNITY_END();
}
```
在 `test/CMakeLists.txt` 加 `wp_add_test(test_mode)`。

- [ ] **Step 3: 跑测试确认通过（header-only，应直接过）**

```bash
cmake -S . -B build && cmake --build build && ctest --test-dir build --output-on-failure -R test_mode
```
Expected: 3 tests PASS。

- [ ] **Step 4: Commit**

```bash
git add core/modes/stab_mode.h test/test_mode/ test/CMakeLists.txt
git commit -m "feat(core): flight mode enum + channel mapping helpers"
```

---

## Task 1.5: 模式状态机（Off/Angle/Rate + 平滑过渡）

组装 AHRS + 双 PID（roll/pitch/yaw 各一）成模式控制器。Angle 摇杆=目标角度，Rate 摇杆=目标角速率，Off=直通。模式切换时输出在 `blend_ms` 内线性渐变。Yaw 永远 Rate。

> **轴向/符号约定（固定翼，与设计文档一致）：** roll/pitch/yaw PID 输出归一化舵量 [-1,1]，正=对应舵面正向。Angle 默认 max roll 45°/pitch 30°；Rate 默认 roll 180/pitch 120/yaw 90 dps。默认 PID 见 spec §5.2。

**Files:**
- Create: `C:\Repository\WeekendPilot\core\modes\mode_controller.h`
- Create: `C:\Repository\WeekendPilot\core\modes\mode_controller.cpp`
- Test: `C:\Repository\WeekendPilot\test\test_mode\test_mode_controller.cpp`

- [ ] **Step 1: 写 mode_controller.h**

```cpp
#pragma once
#include "types.h"
#include "modes/stab_mode.h"
#include "pid/pid_controller.h"
#include "ahrs/ahrs_mahony.h"

namespace wp {

struct StabConfig {
    PidGains angle_roll  {0.011f, 0.0f, 0.0056f};
    PidGains angle_pitch {0.022f, 0.0f, 0.011f};
    PidGains rate_roll   {0.0025f, 0.0f, 0.00003f};
    PidGains rate_pitch  {0.0025f, 0.0f, 0.00003f};
    PidGains rate_yaw    {0.004f, 0.0f, 0.0f};
    float max_angle_roll_deg  = 45.0f;
    float max_angle_pitch_deg = 30.0f;
    float max_rate_roll_dps   = 180.0f;
    float max_rate_pitch_dps  = 120.0f;
    float max_rate_yaw_dps    = 90.0f;
    float pitch_offset_deg    = 3.0f;   // 平飞迎角补偿
    float blend_ms            = 200.0f; // 模式切换过渡
};

// 三轴归一化修正量（叠加到手动舵量之前）
struct StabCorrection {
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
};

class ModeController {
public:
    ModeController();
    void setConfig(const StabConfig& cfg) { cfg_ = cfg; applyGains(); }

    // roll/pitch/yaw_cmd 为摇杆归一化 [-1,1]；att 当前姿态；imu 当前陀螺；dt 秒。
    StabCorrection update(FlightMode mode,
                          float roll_cmd, float pitch_cmd, float yaw_cmd,
                          const Attitude& att, const ImuSample& imu,
                          float dt, bool throttle_low);

    FlightMode activeMode() const { return active_mode_; }

private:
    StabConfig cfg_;
    PidController pid_angle_roll_, pid_angle_pitch_;
    PidController pid_rate_roll_, pid_rate_pitch_, pid_rate_yaw_;
    FlightMode active_mode_ = FlightMode::Off;
    float blend_ = 1.0f;          // 0..1 当前模式占比（过渡用）
    StabCorrection last_corr_;    // 过渡起点
    void applyGains();
    StabCorrection computeFor(FlightMode mode,
                              float r_cmd, float p_cmd, float y_cmd,
                              const Attitude& att, const ImuSample& imu,
                              float dt, bool throttle_low);
};

}  // namespace wp
```

- [ ] **Step 2: 写失败测试**

**Files:** Create `C:\Repository\WeekendPilot\test\test_mode\test_mode_controller.cpp`

```cpp
#include <unity.h>
#include "modes/mode_controller.h"

void setUp() {}
void tearDown() {}

static wp::ImuSample zero_imu() { wp::ImuSample s{}; s.valid = true; s.accel_z = 1.0f; return s; }

void test_off_mode_zero_correction() {
    wp::ModeController mc;
    wp::Attitude att{}; att.roll_deg = 20.0f;
    auto c = mc.update(wp::FlightMode::Off, 0,0,0, att, zero_imu(), 0.01f, false);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, c.roll);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, c.pitch);
}

void test_angle_mode_corrects_toward_level() {
    wp::ModeController mc;
    wp::Attitude att{}; att.roll_deg = 30.0f;   // banked right, stick centered
    wp::ImuSample imu = zero_imu();
    wp::StabCorrection c;
    for (int i = 0; i < 100; ++i)   // let blend settle
        c = mc.update(wp::FlightMode::Angle, 0,0,0, att, imu, 0.01f, false);
    // error = target(0) - 30 = -30 -> negative roll correction (toward level)
    TEST_ASSERT_TRUE(c.roll < 0.0f);
}

void test_rate_mode_opposes_rotation() {
    wp::ModeController mc;
    wp::Attitude att{};
    wp::ImuSample imu = zero_imu();
    imu.gyro_x = 50.0f;   // rolling right, stick centered -> target rate 0
    wp::StabCorrection c;
    for (int i = 0; i < 100; ++i)
        c = mc.update(wp::FlightMode::Rate, 0,0,0, att, imu, 0.01f, false);
    TEST_ASSERT_TRUE(c.roll < 0.0f);   // opposes the +50dps roll
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_off_mode_zero_correction);
    RUN_TEST(test_angle_mode_corrects_toward_level);
    RUN_TEST(test_rate_mode_opposes_rotation);
    return UNITY_END();
}
```
在 `test/CMakeLists.txt` 加 `wp_add_test(test_mode_controller)`。

- [ ] **Step 3: 跑测试确认失败**

```bash
cmake -S . -B build && cmake --build build 2>&1 | tail -10
```
Expected: 链接失败（ModeController 方法未定义）。

- [ ] **Step 4: 写 mode_controller.cpp**

```cpp
#include "modes/mode_controller.h"

namespace wp {

ModeController::ModeController() { applyGains(); }

void ModeController::applyGains() {
    pid_angle_roll_.setGains(cfg_.angle_roll);
    pid_angle_pitch_.setGains(cfg_.angle_pitch);
    pid_rate_roll_.setGains(cfg_.rate_roll);
    pid_rate_pitch_.setGains(cfg_.rate_pitch);
    pid_rate_yaw_.setGains(cfg_.rate_yaw);
}

StabCorrection ModeController::computeFor(FlightMode mode,
                                          float r_cmd, float p_cmd, float y_cmd,
                                          const Attitude& att, const ImuSample& imu,
                                          float dt, bool throttle_low) {
    StabCorrection c;
    if (mode == FlightMode::Off) return c;

    // Yaw 永远 Rate
    {
        float target = y_cmd * cfg_.max_rate_yaw_dps;
        c.yaw = pid_rate_yaw_.update(target, imu.gyro_z, imu.gyro_z, dt);
    }

    if (mode == FlightMode::Angle) {
        float tgt_roll  = r_cmd * cfg_.max_angle_roll_deg;
        float tgt_pitch = p_cmd * cfg_.max_angle_pitch_deg + cfg_.pitch_offset_deg;
        c.roll  = pid_angle_roll_.update(tgt_roll,  att.roll_deg,  imu.gyro_x, dt);
        c.pitch = pid_angle_pitch_.update(tgt_pitch, att.pitch_deg, imu.gyro_y, dt);
    } else { // Rate
        // Rate 地面冻结：油门低时冻结 I（防地面 I 跑飞）
        float tgt_roll  = r_cmd * cfg_.max_rate_roll_dps;
        float tgt_pitch = p_cmd * cfg_.max_rate_pitch_dps;
        if (throttle_low) {
            pid_rate_roll_.reset();
            pid_rate_pitch_.reset();
        }
        c.roll  = pid_rate_roll_.update(tgt_roll,  imu.gyro_x, imu.gyro_x, dt);
        c.pitch = pid_rate_pitch_.update(tgt_pitch, imu.gyro_y, imu.gyro_y, dt);
    }
    return c;
}

StabCorrection ModeController::update(FlightMode mode,
                                      float roll_cmd, float pitch_cmd, float yaw_cmd,
                                      const Attitude& att, const ImuSample& imu,
                                      float dt, bool throttle_low) {
    // 模式切换检测 -> 启动过渡
    if (mode != active_mode_) {
        active_mode_ = mode;
        blend_ = 0.0f;   // 从上次修正渐变到新模式
    }

    StabCorrection target = computeFor(mode, roll_cmd, pitch_cmd, yaw_cmd,
                                       att, imu, dt, throttle_low);

    // 线性过渡 blend 0..1
    if (blend_ < 1.0f && cfg_.blend_ms > 0.0f) {
        blend_ += dt * 1000.0f / cfg_.blend_ms;
        if (blend_ > 1.0f) blend_ = 1.0f;
    }
    StabCorrection out;
    out.roll  = last_corr_.roll  + (target.roll  - last_corr_.roll)  * blend_;
    out.pitch = last_corr_.pitch + (target.pitch - last_corr_.pitch) * blend_;
    out.yaw   = last_corr_.yaw   + (target.yaw   - last_corr_.yaw)   * blend_;

    if (blend_ >= 1.0f) last_corr_ = target;
    return out;
}

}  // namespace wp
```

- [ ] **Step 5: 跑测试确认通过**

```bash
cmake --build build && ctest --test-dir build --output-on-failure -R test_mode_controller
```
Expected: 3 tests PASS。

- [ ] **Step 6: Commit**

```bash
git add core/modes/mode_controller.h core/modes/mode_controller.cpp test/test_mode/test_mode_controller.cpp test/CMakeLists.txt
git commit -m "feat(core): Off/Angle/Rate mode controller with smooth blending"
```

---

## Task 1.6: 顶层 Controller 接入增稳（替换直通）

把 Task 0.4 的直通 Controller 升级：读模式/增益通道，跑 AHRS + ModeController，按 `舵量 = gain*PID + (1-gain)*手动` 混合，叠加到副翼/升降/方向通道。Off 或链路丢失时纯直通。

> **通道约定（阶段 1 固定，阶段 2 做可配置 mixer）：** ch0=roll(副翼) ch1=pitch(升降) ch2=throttle ch3=yaw(方向) ch4=模式 ch5=增益。servo0..3 对应 aileron/elevator/throttle/rudder（与 SITL bridge write_servos 一致）。

**Files:**
- Modify: `C:\Repository\WeekendPilot\core\controller.h`
- Modify: `C:\Repository\WeekendPilot\core\controller.cpp`
- Test: `C:\Repository\WeekendPilot\test\test_controller\test_controller.cpp`

- [ ] **Step 1: 升级 controller.h**

```cpp
#pragma once
#include "types.h"
#include "modes/mode_controller.h"
#include "modes/stab_mode.h"

namespace wp {

struct ControllerConfig {
    bool enabled = true;
    uint8_t mode_channel = 4;      // ch5
    uint8_t gain_channel = 5;      // ch6
    uint8_t throttle_channel = 2;  // ch3
    uint8_t roll_channel = 0;
    uint8_t pitch_channel = 1;
    uint8_t yaw_channel = 3;
    float throttle_low_us = 1100.0f;
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
};

}  // namespace wp
```

- [ ] **Step 2: 升级 test_controller.cpp（保留直通测试 + 加增稳测试）**

```cpp
#include <unity.h>
#include "controller.h"

void setUp() {}
void tearDown() {}

static void fill_centered(wp::ControlInput& in) {
    for (int i = 0; i < wp::kNumChannels; ++i) in.channels[i] = 1500;
    in.channels[2] = 1700;          // throttle up (not ground-frozen)
    in.dt = 0.01f; in.link_ok = true;
    in.imu.valid = true; in.imu.accel_z = 1.0f;
}

void test_off_mode_is_passthrough() {
    wp::Controller c;
    wp::ControlInput in{}; fill_centered(in);
    in.channels[4] = 1000;          // mode Off
    in.channels[0] = 1650;          // roll stick
    wp::ServoCommand out = c.update(in);
    TEST_ASSERT_EQUAL_UINT16(1650, out.servo[0]);   // aileron passthrough
}

void test_link_lost_forces_passthrough() {
    wp::Controller c;
    wp::ControlInput in{}; fill_centered(in);
    in.channels[4] = 1500;          // Angle
    in.link_ok = false;
    in.channels[0] = 1650;
    wp::ServoCommand out = c.update(in);
    TEST_ASSERT_EQUAL_UINT16(1650, out.servo[0]);
}

void test_angle_mode_banked_adds_correction() {
    wp::Controller c;
    wp::ControlInput in{}; fill_centered(in);
    in.channels[4] = 1500;          // Angle
    in.channels[5] = 2000;          // gain 100%
    in.imu.accel_y = 0.5f; in.imu.accel_z = 0.866f;  // banked right
    wp::ServoCommand out{};
    for (int i = 0; i < 300; ++i) out = c.update(in);   // settle AHRS + blend
    // banked right, stick centered -> aileron should move away from 1500
    TEST_ASSERT_TRUE(out.servo[0] != 1500);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_off_mode_is_passthrough);
    RUN_TEST(test_link_lost_forces_passthrough);
    RUN_TEST(test_angle_mode_banked_adds_correction);
    return UNITY_END();
}
```

- [ ] **Step 3: 跑测试确认失败**

```bash
cmake -S . -B build && cmake --build build 2>&1 | tail -10
```
Expected: 失败（新接口未实现）。

- [ ] **Step 4: 写 controller.cpp**

```cpp
#include "controller.h"

namespace wp {

Controller::Controller() {
    modes_.setConfig(cfg_.stab);
}

void Controller::setConfig(const ControllerConfig& cfg) {
    cfg_ = cfg;
    modes_.setConfig(cfg_.stab);
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

    ahrs_.update(in.imu, in.dt);

    float roll_cmd  = channelToNorm(in.channels[cfg_.roll_channel]);
    float pitch_cmd = channelToNorm(in.channels[cfg_.pitch_channel]);
    float yaw_cmd   = channelToNorm(in.channels[cfg_.yaw_channel]);
    float gain      = gainFromChannel(in.channels[cfg_.gain_channel]);
    bool throttle_low = in.channels[cfg_.throttle_channel] < cfg_.throttle_low_us;

    StabCorrection corr = modes_.update(mode, roll_cmd, pitch_cmd, yaw_cmd,
                                        ahrs_.attitude(), in.imu, in.dt, throttle_low);

    // 舵量 = gain*PID修正 + (1-gain)*手动；PID 修正已是归一化 [-1,1]
    // 手动归一化 = channelToNorm；最终 = 手动 + gain*修正（修正叠加在手动之上）
    auto mix = [&](uint8_t servo_idx, uint8_t in_ch, float c) {
        float manual = channelToNorm(in.channels[in_ch]);
        float blended = manual + gain * c;
        out.servo[servo_idx] = normToServoUs(blended);
    };
    mix(0, cfg_.roll_channel,  corr.roll);    // aileron
    mix(1, cfg_.pitch_channel, corr.pitch);   // elevator
    mix(3, cfg_.yaw_channel,   corr.yaw);     // rudder
    // servo[2] throttle 直通（已在上面 passthrough）
    return out;
}

}  // namespace wp
```

- [ ] **Step 5: 跑全部测试确认通过**

```bash
cmake -S . -B build && cmake --build build && ctest --test-dir build --output-on-failure
```
Expected: 所有测试 PASS（smoke/types/controller/filter/pid/ahrs/mode/mode_controller）。

- [ ] **Step 6: 重建 SITL 共享库并验证板载仍编译**

```bash
cmake --build build              # 重建 wp_core_capi.dll（含新 controller）
pio run -e weekendpilot_s3 2>&1 | tail -5
```
Expected: DLL 重建成功；板载固件仍编译通过（core 升级未破坏板载）。

- [ ] **Step 7: Commit**

```bash
git add core/controller.h core/controller.cpp test/test_controller/test_controller.cpp
git commit -m "feat(core): wire AHRS+modes into top-level controller with gain mixing"
```

---

## Task 1.7: SITL 闭环调参 + FlightGear 动画

在 JSBSim 里验证 Angle 模式能把扰动后的飞机拉回水平，并接 FlightGear 看 3D。这是阶段 1 的验收。

**Files:**
- Modify: `C:\Repository\WeekendPilot\hal_sitl\run_sitl.py`（加扰动注入 + Angle 模式通道）
- Create: `C:\Repository\WeekendPilot\hal_sitl\check_angle_hold.py`
- Create: `C:\Repository\WeekendPilot\hal_sitl\fg_run.md`（FlightGear 启动说明）

- [ ] **Step 1: run_sitl.py 加 Angle 模式 + 扰动**

把 `rc_for_time` 改为开启 Angle 模式（ch4=1500）、增益满（ch5=2000），并在某时刻通过 JSBSim 属性注入一个滚转扰动。在 `main()` 的循环里，t=3.0s 时执行：

```python
def rc_for_time(t):
    ch = [1500] * 16
    ch[2] = 1700       # throttle
    ch[4] = 1500       # mode = Angle
    ch[5] = 2000       # gain 100%
    return ch
```

在 `main()` 循环 `fdm.run()` 之前加扰动注入：

```python
            if abs(t - 3.0) < args.dt:   # one-shot roll disturbance
                fdm.set_property_value('attitude/phi-rad',
                    fdm.get_property_value('attitude/phi-rad') + 0.35)  # ~20deg kick
```

- [ ] **Step 2: 写 check_angle_hold.py（验收判据）**

```python
"""Verify Angle mode recovers roll toward level after a disturbance."""
import csv, sys
rows = list(csv.DictReader(open(sys.argv[1] if len(sys.argv) > 1 else 'sitl_log.csv')))
def roll_at(t):
    best = min(rows, key=lambda r: abs(float(r['t']) - t))
    return float(best['roll'])
peak = max(abs(roll_at(t/10.0)) for t in range(30, 40))   # 3.0-4.0s peak after kick
settled = abs(roll_at(8.0))                                # near end
print('peak roll after disturbance:', round(peak,1), 'deg')
print('roll at t=8s:', round(settled,1), 'deg')
assert settled < peak * 0.5, 'Angle mode did not recover toward level'
print('ANGLE-HOLD RECOVERY OK')
```

- [ ] **Step 3: 跑调参验证**

```bash
cd /c/Repository/WeekendPilot
PY="C:/Users/Weekend/.pyenv/pyenv-win/versions/3.13.3/python.exe"
"$PY" hal_sitl/run_sitl.py --secs 10 --out sitl_log.csv
"$PY" hal_sitl/check_angle_hold.py sitl_log.csv
```
Expected: `ANGLE-HOLD RECOVERY OK`（扰动后 roll 收敛回水平）。若不收敛，调 `StabConfig.angle_roll` 的 kp/kd（这是 PID 调参循环的起点，记录到 docs）。

- [ ] **Step 4: 写 FlightGear 启动说明 fg_run.md**

```markdown
# FlightGear 3D 动画（演示用）

1. 启动 FlightGear，等待 socket 输入（native-FDM，端口 5550）：
   fgfs --fdm=null --native-fdm=socket,in,30,,5550,udp --aircraft=c172p --disable-ai-traffic

2. 在 run_sitl.py 里启用 FlightGear 输出：JSBSim 加载
   C:/Repository/jsbsim/data_output/flightgear.xml 作为 output，
   设 IP=127.0.0.1 port=5550 rate=30，调用 fdm.set_output_directive 或在脚本里
   fdm.enable_output()。详见 JSBSim FGOutputFG 文档。

3. 调参时不启动 FlightGear（无头跑只读 CSV），演示时再开。
```

- [ ] **Step 5: Commit**

```bash
git add hal_sitl/run_sitl.py hal_sitl/check_angle_hold.py hal_sitl/fg_run.md
git commit -m "feat(sitl): Angle-mode recovery test + FlightGear animation notes"
```

> **阶段 1 里程碑：** 全部 core 单测绿；SITL 里 Angle 模式扰动后能自动回水平；FlightGear 可看 3D；板载固件持续可编译。增稳核心成立。

---

## 阶段 0~1 完成后的状态

- ✅ 两层架构落地：`core/`（纯 C++）+ `hal_esp32/` + `hal_sitl/`，同一 core 两端共用
- ✅ PC 单测覆盖 PT1/PID/AHRS/模式/控制器；JSBSim SITL 闭环可跑
- ✅ Off/Angle/Rate 三模式 + 平滑过渡 + 增益混合 + failsafe 直通
- ✅ FlightGear 3D 动画管道就绪

**下一阶段（阶段 2，另出计划）：** Mixer（flaperon/V-tail/elevon）+ 外设通道 + Panic Recovery + G-limit + 黑匣子（PSRAM）。
