# 配置化 + 模块化传感器骨架 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 建立"换传感器只写一个适配、core 与上层零改动"的模块化传感器层 + 三层配置化骨架，全部在 SITL/单测可验，板载侧默认门控关闭，硬件到货改一个 `-D` 即激活。

**Architecture:** 三层配置（板级 profile 编译期 / 能力宏 `WP_*` 编译期 / `ControllerConfig` 运行时+NVS）。传感器走纯虚接口（`IGyroAccel/IMagnetometer/IBarometer/IGnss/IAirspeed`，放 `core/sensors/`）→ 具体实现（板载真驱动 / SITL Mock / 测试 Mock）→ `SensorFrontend` 聚合层（纯算法，base~pro 档直通、max 档表决，放 `core/sensors/`）→ 输出单组干净 Sample 给 `core`。`probe()` 自动探测推断当前档位。真驱动（ICM-42688-P/BMP390L）只在 PlatformIO 编译，PC 侧用 Mock 实现同一接口喂合成数据。

**Tech Stack:** C++17 纯算法核（`-Wall -Wextra -Werror`）、Unity 单测、CMake preset `dev`(Ninja+GCC)、PlatformIO(ESP32-S3 Arduino)、NVS(`Preferences`)。

**Scope（本次只搭骨架）：** 配置层 + 5 接口 + SensorFrontend 直通 + Mock + NVS 序列化骨架。**不含**：ICM/BMP 真驱动实现、阶段 3 算法（Alt-Hold 等）。这些插进本次建好的插座即可，不返工。

**前置事实（已核实 2026-05-31）：**
- `ControllerConfig`/`StabConfig`/`PidGains`/`GLimitConfig`/`PeripheralMap` 全为 POD，可平凡拷贝 → NVS 用 `memcpy+版本头+CRC`。
- `core/types.h` 已有 `ImuSample`(gyro deg/s, accel g)/`MagSample`/`BaroSample`(altitude_m)。本计划新增 `GnssSample`/`AirspeedSample` 并补 `SensorTier` 枚举。
- 引脚：I2C SDA=18 SCL=39；ICM-42688-P @0x68 INT=40；BMP390L @0x76。ElrsRX 有已验证驱动 `lib/Stabilizer/imu_icm42688.*` 和 `lib/Baro/baro_bmp390.*`（后续真驱动任务移植参考，本计划不实现）。
- 编译：`cmake --preset dev` 后 `cmake --build build`；测试 `ctest --test-dir build --output-on-failure`；新测试在 `test/CMakeLists.txt` 加 `wp_add_test(name)`（GLOB core/*.cpp 自动纳入）。

---

## File Structure

新增文件及职责：

| 文件 | 职责 |
|------|------|
| `core/sensors/sensor_types.h` | 新增 `GnssSample`/`AirspeedSample` struct + `SensorTier` 枚举（Base/Plus/Pro/Max/None）。`ImuSample` 等仍在 `types.h`。 |
| `core/sensors/sensor_interfaces.h` | 5 个纯虚接口 `IGyroAccel/IMagnetometer/IBarometer/IGnss/IAirspeed`，统一契约 `probe()/init()/read(Sample&)`。零 I/O。 |
| `core/sensors/sensor_frontend.h/.cpp` | `SensorFrontend` 聚合层：持有各类型接口指针，`poll()` 读所有→填 `SensorBundle`；`detectTier()` 据 probe 结果推断档位。base~pro 直通，max 表决（本次只做直通 + 单实例）。 |
| `core/sensors/mock_sensors.h` | `MockGyroAccel/MockBaro/MockMag/MockGnss/MockAirspeed`：实现接口、由测试/SITL 注入合成数据。header-only。 |
| `core/config/board_config.h` | 板级 profile：引脚常量 + 默认能力宏。`#if defined(BOARD_DEVKIT_S3)` 选具体板。 |
| `core/config/capabilities.h` | 能力宏框架：`WP_HAS_IMU/WP_HAS_BARO/WP_HAS_MAG/WP_HAS_GPS/WP_HAS_AIRSPEED` + `WP_ENABLE_*`，提供 `constexpr bool` 镜像供 core 运行时分支。 |
| `core/config/config_store.h/.cpp` | `ControllerConfig` 的 NVS 序列化骨架：`serialize(buf)`/`deserialize(buf)` 纯函数（版本头+CRC16），平台 I/O（NVS 读写）留虚接口 `IConfigBackend`。PC 用内存 backend 单测。 |
| `test/test_sensor_frontend/` | Frontend 直通 + 档位探测单测。 |
| `test/test_config_store/` | 序列化往返 + 版本/CRC 校验单测。 |
| 修改 `core/controller.h/.cpp` | `update()` 增加从 `SensorBundle` 取数的重载/路径（保持旧 `ControlInput` 兼容，渐进迁移）。 |
| 修改 `hal_sitl/core_c_api.cpp` | 经 Mock 注入路径喂数（验证 SITL 走新抽象后数值不变）。 |

---

## Task 1: 传感器类型与档位枚举

**Files:**
- Create: `core/sensors/sensor_types.h`
- Test: `test/test_sensor_frontend/test_sensor_frontend.cpp`（本任务先建文件 + 编译占位测试）
- Modify: `test/CMakeLists.txt`

- [ ] **Step 1: 写失败测试**

`test/test_sensor_frontend/test_sensor_frontend.cpp`：
```cpp
#include "unity.h"
#include "sensors/sensor_types.h"
using namespace wp;

void setUp() {} void tearDown() {}

void test_tier_enum_ordered() {
    // 档位有序：None < Base < Plus < Pro < Max，便于"≥某档"比较
    TEST_ASSERT_TRUE(static_cast<int>(SensorTier::None) < static_cast<int>(SensorTier::Base));
    TEST_ASSERT_TRUE(static_cast<int>(SensorTier::Base) < static_cast<int>(SensorTier::Plus));
    TEST_ASSERT_TRUE(static_cast<int>(SensorTier::Plus) < static_cast<int>(SensorTier::Pro));
}

void test_gnss_sample_defaults_invalid() {
    GnssSample g{};
    TEST_ASSERT_FALSE(g.valid);
}

void test_airspeed_sample_defaults_invalid() {
    AirspeedSample a{};
    TEST_ASSERT_FALSE(a.valid);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_tier_enum_ordered);
    RUN_TEST(test_gnss_sample_defaults_invalid);
    RUN_TEST(test_airspeed_sample_defaults_invalid);
    return UNITY_END();
}
```

- [ ] **Step 2: 在 test/CMakeLists.txt 注册**

在文件末尾 `wp_add_test(test_glimit)` 后加一行：
```cmake
wp_add_test(test_sensor_frontend)
```

- [ ] **Step 3: 运行测试确认失败（编译错误：找不到头）**

Run: `cmake --build build && ctest --test-dir build -R test_sensor_frontend --output-on-failure`
Expected: 配置/编译失败，找不到 `sensors/sensor_types.h`

- [ ] **Step 4: 写最小实现**

`core/sensors/sensor_types.h`：
```cpp
#pragma once
#include <cstdint>

namespace wp {

// 传感器档位（有序：缺高档传感器自动降级，见设计 §6.0）。
enum class SensorTier : uint8_t { None = 0, Base = 1, Plus = 2, Pro = 3, Max = 4 };

// GPS 标准样本（plus 档）。vel_ned: m/s，北/东/地。
struct GnssSample {
    double lat = 0.0, lon = 0.0;
    float  vel_ned[3] = {0.0f, 0.0f, 0.0f};
    float  ground_speed_mps = 0.0f;
    uint8_t fix = 0;        // 0=no fix
    uint8_t sats = 0;
    bool   valid = false;
};

// 空速标准样本（pro 档）。
struct AirspeedSample {
    float ias_mps = 0.0f;   // 指示空速
    bool  valid = false;
};

}  // namespace wp
```

- [ ] **Step 5: 运行测试确认通过**

Run: `cmake --build build && ctest --test-dir build -R test_sensor_frontend --output-on-failure`
Expected: PASS (3/3)

- [ ] **Step 6: 提交**

```bash
git add core/sensors/sensor_types.h test/test_sensor_frontend/ test/CMakeLists.txt
git commit -m "feat(sensors): sensor tier enum + GNSS/airspeed sample types"
```

---

## Task 2: 传感器接口层（5 个纯虚接口）

**Files:**
- Create: `core/sensors/sensor_interfaces.h`
- Create: `core/sensors/mock_sensors.h`
- Test: `test/test_sensor_frontend/test_sensor_frontend.cpp`（追加）

- [ ] **Step 1: 追加失败测试**

在 `test_sensor_frontend.cpp` 顶部加 include，并在 main 前加测试：
```cpp
#include "sensors/sensor_interfaces.h"
#include "sensors/mock_sensors.h"

void test_mock_gyroaccel_probe_and_read() {
    MockGyroAccel imu;
    imu.present = true;
    imu.sample.gyro_x = 1.5f; imu.sample.accel_z = 1.0f; imu.sample.valid = true;
    TEST_ASSERT_TRUE(imu.probe());
    TEST_ASSERT_TRUE(imu.init());
    ImuSample s{};
    TEST_ASSERT_TRUE(imu.read(s));
    TEST_ASSERT_EQUAL_FLOAT(1.5f, s.gyro_x);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, s.accel_z);
}

void test_mock_absent_probe_fails() {
    MockBaro baro;
    baro.present = false;
    TEST_ASSERT_FALSE(baro.probe());
}
```
并在 main 的 UNITY_BEGIN/END 之间加：
```cpp
    RUN_TEST(test_mock_gyroaccel_probe_and_read);
    RUN_TEST(test_mock_absent_probe_fails);
```

- [ ] **Step 2: 运行确认失败**

Run: `cmake --build build && ctest --test-dir build -R test_sensor_frontend --output-on-failure`
Expected: 编译失败，找不到 `sensor_interfaces.h`

- [ ] **Step 3: 写接口实现**

`core/sensors/sensor_interfaces.h`：
```cpp
#pragma once
#include "types.h"
#include "sensors/sensor_types.h"

namespace wp {

// 统一驱动契约（设计 §4.2.1）：每个实现给 probe/init/read。
// 接口零 I/O —— core 只认这些接口，不认型号；真驱动在 hal 实现，PC 用 Mock。
struct IGyroAccel {
    virtual ~IGyroAccel() = default;
    virtual bool probe() = 0;              // WHO_AM_I 在不在
    virtual bool init()  = 0;
    virtual bool read(ImuSample& out) = 0; // gyro deg/s, accel g
};
struct IMagnetometer {
    virtual ~IMagnetometer() = default;
    virtual bool probe() = 0;
    virtual bool init()  = 0;
    virtual bool read(MagSample& out) = 0;
};
struct IBarometer {
    virtual ~IBarometer() = default;
    virtual bool probe() = 0;
    virtual bool init()  = 0;
    virtual bool read(BaroSample& out) = 0;
};
struct IGnss {
    virtual ~IGnss() = default;
    virtual bool probe() = 0;
    virtual bool init()  = 0;
    virtual bool read(GnssSample& out) = 0;
};
struct IAirspeed {
    virtual ~IAirspeed() = default;
    virtual bool probe() = 0;
    virtual bool init()  = 0;
    virtual bool read(AirspeedSample& out) = 0;
};

}  // namespace wp
```

- [ ] **Step 4: 写 Mock 实现**

`core/sensors/mock_sensors.h`：
```cpp
#pragma once
#include "sensors/sensor_interfaces.h"

namespace wp {

// 测试/SITL 用：注入合成数据。present 控制 probe 返回，valid 控制 read 成败。
struct MockGyroAccel : IGyroAccel {
    bool present = true;
    ImuSample sample{};
    bool probe() override { return present; }
    bool init()  override { return present; }
    bool read(ImuSample& out) override { if (!present) return false; out = sample; return true; }
};
struct MockMag : IMagnetometer {
    bool present = true;
    MagSample sample{};
    bool probe() override { return present; }
    bool init()  override { return present; }
    bool read(MagSample& out) override { if (!present) return false; out = sample; return true; }
};
struct MockBaro : IBarometer {
    bool present = true;
    BaroSample sample{};
    bool probe() override { return present; }
    bool init()  override { return present; }
    bool read(BaroSample& out) override { if (!present) return false; out = sample; return true; }
};
struct MockGnss : IGnss {
    bool present = true;
    GnssSample sample{};
    bool probe() override { return present; }
    bool init()  override { return present; }
    bool read(GnssSample& out) override { if (!present) return false; out = sample; return true; }
};
struct MockAirspeed : IAirspeed {
    bool present = true;
    AirspeedSample sample{};
    bool probe() override { return present; }
    bool init()  override { return present; }
    bool read(AirspeedSample& out) override { if (!present) return false; out = sample; return true; }
};

}  // namespace wp
```

- [ ] **Step 5: 运行确认通过**

Run: `cmake --build build && ctest --test-dir build -R test_sensor_frontend --output-on-failure`
Expected: PASS (5/5)

- [ ] **Step 6: 提交**

```bash
git add core/sensors/sensor_interfaces.h core/sensors/mock_sensors.h test/test_sensor_frontend/
git commit -m "feat(sensors): 5 pure-virtual sensor interfaces + mock implementations"
```

---

## Task 3: SensorFrontend 聚合层（直通 + 档位探测）

**Files:**
- Create: `core/sensors/sensor_frontend.h`
- Create: `core/sensors/sensor_frontend.cpp`
- Test: `test/test_sensor_frontend/test_sensor_frontend.cpp`（追加）

- [ ] **Step 1: 追加失败测试**

在 `test_sensor_frontend.cpp` 加 include 与测试：
```cpp
#include "sensors/sensor_frontend.h"

void test_frontend_passthrough_fills_bundle() {
    MockGyroAccel imu; imu.present = true;
    imu.sample.gyro_x = 2.0f; imu.sample.valid = true;
    MockBaro baro; baro.present = true;
    baro.sample.altitude_m = 123.0f; baro.sample.valid = true;

    SensorFrontend fe;
    fe.setGyroAccel(&imu);
    fe.setBarometer(&baro);
    fe.begin();                 // 调 probe+init

    SensorBundle b{};
    fe.poll(b);
    TEST_ASSERT_TRUE(b.imu.valid);
    TEST_ASSERT_EQUAL_FLOAT(2.0f, b.imu.gyro_x);
    TEST_ASSERT_TRUE(b.baro.valid);
    TEST_ASSERT_EQUAL_FLOAT(123.0f, b.baro.altitude_m);
}

void test_tier_base_requires_imu_mag_baro() {
    MockGyroAccel imu; MockMag mag; MockBaro baro;
    imu.present = mag.present = baro.present = true;
    SensorFrontend fe;
    fe.setGyroAccel(&imu); fe.setMagnetometer(&mag); fe.setBarometer(&baro);
    fe.begin();
    TEST_ASSERT_EQUAL_INT((int)SensorTier::Base, (int)fe.tier());
}

void test_tier_none_without_mag() {
    // 缺磁力计：达不到 base（设计 §6.0/附录 C：base 必须 9 轴）
    MockGyroAccel imu; MockBaro baro;
    imu.present = baro.present = true;
    SensorFrontend fe;
    fe.setGyroAccel(&imu); fe.setBarometer(&baro);
    fe.begin();
    TEST_ASSERT_EQUAL_INT((int)SensorTier::None, (int)fe.tier());
}

void test_tier_plus_with_gps() {
    MockGyroAccel imu; MockMag mag; MockBaro baro; MockGnss gps;
    imu.present = mag.present = baro.present = gps.present = true;
    SensorFrontend fe;
    fe.setGyroAccel(&imu); fe.setMagnetometer(&mag);
    fe.setBarometer(&baro); fe.setGnss(&gps);
    fe.begin();
    TEST_ASSERT_EQUAL_INT((int)SensorTier::Plus, (int)fe.tier());
}

void test_absent_sensor_leaves_bundle_invalid() {
    MockGyroAccel imu; imu.present = false;
    SensorFrontend fe;
    fe.setGyroAccel(&imu);
    fe.begin();
    SensorBundle b{};
    fe.poll(b);
    TEST_ASSERT_FALSE(b.imu.valid);
}
```
在 main 注册全部 5 个：
```cpp
    RUN_TEST(test_frontend_passthrough_fills_bundle);
    RUN_TEST(test_tier_base_requires_imu_mag_baro);
    RUN_TEST(test_tier_none_without_mag);
    RUN_TEST(test_tier_plus_with_gps);
    RUN_TEST(test_absent_sensor_leaves_bundle_invalid);
```

- [ ] **Step 2: 运行确认失败**

Run: `cmake --build build && ctest --test-dir build -R test_sensor_frontend --output-on-failure`
Expected: 编译失败，找不到 `sensor_frontend.h`

- [ ] **Step 3: 写 header**

`core/sensors/sensor_frontend.h`：
```cpp
#pragma once
#include "sensors/sensor_interfaces.h"

namespace wp {

// 喂给 core 的单组干净样本（core 永远只看到这一组，不知背后几个传感器）。
struct SensorBundle {
    ImuSample      imu;
    MagSample      mag;
    BaroSample     baro;
    GnssSample     gnss;
    AirspeedSample airspeed;
};

// 聚合层：持有各类型接口指针，begin() 探测+初始化并推断档位，poll() 读一轮填 bundle。
// 本次为单实例直通；max 档多实例表决留待后续（接口已为此预留）。
class SensorFrontend {
public:
    void setGyroAccel(IGyroAccel* p)   { gyro_ = p; }
    void setMagnetometer(IMagnetometer* p) { mag_ = p; }
    void setBarometer(IBarometer* p)   { baro_ = p; }
    void setGnss(IGnss* p)             { gnss_ = p; }
    void setAirspeed(IAirspeed* p)     { air_ = p; }

    void begin();              // probe + init 全部已注册传感器，推断 tier_
    void poll(SensorBundle& out); // 读一轮；缺失/失败的样本 valid=false
    SensorTier tier() const { return tier_; }

    bool has_imu  = false;
    bool has_mag  = false;
    bool has_baro = false;
    bool has_gnss = false;
    bool has_air  = false;

private:
    IGyroAccel*    gyro_ = nullptr;
    IMagnetometer* mag_  = nullptr;
    IBarometer*    baro_ = nullptr;
    IGnss*         gnss_ = nullptr;
    IAirspeed*     air_  = nullptr;
    SensorTier     tier_ = SensorTier::None;

    void detectTier();
};

}  // namespace wp
```

- [ ] **Step 4: 写 .cpp**

`core/sensors/sensor_frontend.cpp`：
```cpp
#include "sensors/sensor_frontend.h"

namespace wp {

static bool probeInit(IGyroAccel* p)    { return p && p->probe() && p->init(); }
static bool probeInit(IMagnetometer* p) { return p && p->probe() && p->init(); }
static bool probeInit(IBarometer* p)    { return p && p->probe() && p->init(); }
static bool probeInit(IGnss* p)         { return p && p->probe() && p->init(); }
static bool probeInit(IAirspeed* p)     { return p && p->probe() && p->init(); }

void SensorFrontend::begin() {
    has_imu  = probeInit(gyro_);
    has_mag  = probeInit(mag_);
    has_baro = probeInit(baro_);
    has_gnss = probeInit(gnss_);
    has_air  = probeInit(air_);
    detectTier();
}

void SensorFrontend::detectTier() {
    // base = 陀螺+加速度+磁力计+气压（§6.0/附录 C：少磁力计达不到 base）
    if (!(has_imu && has_mag && has_baro)) { tier_ = SensorTier::None; return; }
    tier_ = SensorTier::Base;
    if (has_gnss) tier_ = SensorTier::Plus;        // plus = base + GPS
    if (has_gnss && has_air) tier_ = SensorTier::Pro; // pro = plus + 空速
    // max（冗余表决）留待后续多实例实现
}

void SensorFrontend::poll(SensorBundle& out) {
    if (!(has_imu  && gyro_ && gyro_->read(out.imu)))   out.imu.valid  = false;
    if (!(has_mag  && mag_  && mag_->read(out.mag)))    out.mag.valid  = false;
    if (!(has_baro && baro_ && baro_->read(out.baro)))  out.baro.valid = false;
    if (!(has_gnss && gnss_ && gnss_->read(out.gnss)))  out.gnss.valid = false;
    if (!(has_air  && air_  && air_->read(out.airspeed))) out.airspeed.valid = false;
}

}  // namespace wp
```

- [ ] **Step 5: 运行确认通过**

Run: `cmake --build build && ctest --test-dir build -R test_sensor_frontend --output-on-failure`
Expected: PASS (10/10 累计)

- [ ] **Step 6: 提交**

```bash
git add core/sensors/sensor_frontend.h core/sensors/sensor_frontend.cpp test/test_sensor_frontend/
git commit -m "feat(sensors): SensorFrontend passthrough aggregation + tier auto-detect"
```

---

## Task 4: 板级 profile + 能力宏框架

**Files:**
- Create: `core/config/board_config.h`
- Create: `core/config/capabilities.h`
- Test: `test/test_config_store/test_config_store.cpp`（本任务建文件 + 能力宏测试）
- Modify: `test/CMakeLists.txt`、`platformio.ini`

- [ ] **Step 1: 写失败测试**

`test/test_config_store/test_config_store.cpp`：
```cpp
#include "unity.h"
#include "config/capabilities.h"
#include "config/board_config.h"
using namespace wp;

void setUp() {} void tearDown() {}

void test_pc_test_build_enables_all_capabilities() {
    // PC/测试构建默认全开（算法都要在 SITL 验）
    TEST_ASSERT_TRUE(kHasImu);
    TEST_ASSERT_TRUE(kHasBaro);
    TEST_ASSERT_TRUE(kHasMag);
}

void test_board_pins_defined() {
    // 板级 profile 暴露引脚常量（值见设计 §6.3）
    TEST_ASSERT_EQUAL_INT(18, kPinI2cSda);
    TEST_ASSERT_EQUAL_INT(39, kPinI2cScl);
    TEST_ASSERT_EQUAL_INT(0x68, kAddrImu);
    TEST_ASSERT_EQUAL_INT(0x76, kAddrBaro);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_pc_test_build_enables_all_capabilities);
    RUN_TEST(test_board_pins_defined);
    return UNITY_END();
}
```

- [ ] **Step 2: 在 test/CMakeLists.txt 注册**

在 `wp_add_test(test_sensor_frontend)` 后加：
```cmake
wp_add_test(test_config_store)
```

- [ ] **Step 3: 运行确认失败**

Run: `cmake --build build && ctest --test-dir build -R test_config_store --output-on-failure`
Expected: 编译失败，找不到 `config/capabilities.h`

- [ ] **Step 4: 写 capabilities.h**

`core/config/capabilities.h`：
```cpp
#pragma once

// 能力宏框架（编译期门控）。三种来源覆盖默认值：
//   1) 板级 profile（board_config.h）按 BOARD_* 设默认
//   2) PlatformIO build_flags -D 覆盖（板载按手头硬件开关）
//   3) 未定义任何 BOARD_* 时（PC/CMake 测试构建）默认全开 —— 算法都要在 SITL 验
//
// "全写好先不使能"：功能代码恒编译进 core（PC 全开可验）；板载侧用这些宏 +
// 运行时 probe() 双重门控决定是否激活。硬件到货改一个 -D 即 enable。

// PC/测试构建（无 BOARD_* 宏）默认全开
#if !defined(BOARD_DEVKIT_S3)
  #ifndef WP_HAS_IMU
    #define WP_HAS_IMU 1
  #endif
  #ifndef WP_HAS_BARO
    #define WP_HAS_BARO 1
  #endif
  #ifndef WP_HAS_MAG
    #define WP_HAS_MAG 1
  #endif
  #ifndef WP_HAS_GPS
    #define WP_HAS_GPS 1
  #endif
  #ifndef WP_HAS_AIRSPEED
    #define WP_HAS_AIRSPEED 1
  #endif
#endif

// 兜底：任何未被上面或 build_flags 定义的，默认 0（关闭）
#ifndef WP_HAS_IMU
  #define WP_HAS_IMU 0
#endif
#ifndef WP_HAS_BARO
  #define WP_HAS_BARO 0
#endif
#ifndef WP_HAS_MAG
  #define WP_HAS_MAG 0
#endif
#ifndef WP_HAS_GPS
  #define WP_HAS_GPS 0
#endif
#ifndef WP_HAS_AIRSPEED
  #define WP_HAS_AIRSPEED 0
#endif

namespace wp {
// constexpr 镜像：让 core 用 `if constexpr (kHasGps)` 做分支，比裸宏更安全可读。
constexpr bool kHasImu      = WP_HAS_IMU;
constexpr bool kHasBaro     = WP_HAS_BARO;
constexpr bool kHasMag      = WP_HAS_MAG;
constexpr bool kHasGps      = WP_HAS_GPS;
constexpr bool kHasAirspeed = WP_HAS_AIRSPEED;
}  // namespace wp
```

- [ ] **Step 5: 写 board_config.h**

`core/config/board_config.h`：
```cpp
#pragma once
#include <cstdint>

// 板级 profile：引脚 + 总线地址（编译期）。新板加一个 #elif 块。
// 引脚值见设计 §6.3（沿用 ElrsRX 已验证 DevKit layout）。

#if defined(BOARD_DEVKIT_S3)
  // ESP32-S3 DevKitC-1：默认按"当前手头硬件"开能力（IMU+Baro 有，其余在路上）
  #ifndef WP_HAS_IMU
    #define WP_HAS_IMU 1
  #endif
  #ifndef WP_HAS_BARO
    #define WP_HAS_BARO 1
  #endif
  // WP_HAS_MAG / WP_HAS_GPS / WP_HAS_AIRSPEED 默认不开，到货由 build_flags -D 打开
#endif

namespace wp {

// I2C 共享总线 + 传感器地址（§6.3）
constexpr int kPinI2cSda = 18;
constexpr int kPinI2cScl = 39;
constexpr int kAddrImu   = 0x68;  // ICM-42688-P
constexpr int kAddrBaro  = 0x76;  // BMP390L
constexpr int kPinImuInt = 40;    // ICM 数据就绪中断（可选）

// PWM 8 路（§6.3）
constexpr int kPwmPins[8] = {1, 2, 8, 9, 10, 15, 16, 17};

// CRSF UART（接 SuperX）
constexpr int kPinCrsfRx = 44;
constexpr int kPinCrsfTx = 43;

}  // namespace wp
```

注意：`capabilities.h` 必须在判断 `BOARD_DEVKIT_S3` 后才定稳。为避免包含顺序坑，约定 **`board_config.h` 先 include，再 include `capabilities.h`**；或在 `capabilities.h` 顶部 `#include "config/board_config.h"`。本计划采用后者——在 Step 4 的 `capabilities.h` 顶部加 `#include "config/board_config.h"`，确保 BOARD_* 分支先生效。实现时务必加这一行。

- [ ] **Step 6: platformio.ini 增加板级宏**

`platformio.ini` 的 `build_flags` 末尾加（在 `-DCORE_DEBUG_LEVEL=3` 后）：
```ini
    -DBOARD_DEVKIT_S3
    -DWP_HAS_IMU=1
    -DWP_HAS_BARO=1
; mag/gps/airspeed 到货后在此打开，例如：
;   -DWP_HAS_MAG=1
```

- [ ] **Step 7: 运行确认通过（PC 构建，全开）**

Run: `cmake --build build && ctest --test-dir build -R test_config_store --output-on-failure`
Expected: PASS (2/2)

- [ ] **Step 8: 提交**

```bash
git add core/config/board_config.h core/config/capabilities.h test/test_config_store/ test/CMakeLists.txt platformio.ini
git commit -m "feat(config): board profile + capability macro framework (compile-time gating)"
```

---

## Task 5: ControllerConfig NVS 序列化骨架

**Files:**
- Create: `core/config/config_store.h`
- Create: `core/config/config_store.cpp`
- Test: `test/test_config_store/test_config_store.cpp`（追加）

**说明：** 序列化是纯函数（`memcpy` POD + 版本头 + CRC16），PC 可单测。平台读写（NVS）抽成 `IConfigBackend` 虚接口，PC 用内存 backend；板载真 NVS backend 留待硬件任务。

- [ ] **Step 1: 追加失败测试**

在 `test_config_store.cpp` 加 include 与测试：
```cpp
#include "config/config_store.h"
#include "controller.h"

void test_serialize_roundtrip_preserves_config() {
    ControllerConfig cfg{};
    cfg.airframe = Airframe::Flaperon;
    cfg.glimit.soft_g = 5.5f;
    cfg.stab.angle_roll.kp = 0.099f;

    uint8_t buf[kConfigBlobSize];
    uint16_t n = serializeConfig(cfg, buf, sizeof(buf));
    TEST_ASSERT_TRUE(n > 0);

    ControllerConfig out{};
    bool ok = deserializeConfig(buf, n, out);
    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_INT((int)Airframe::Flaperon, (int)out.airframe);
    TEST_ASSERT_EQUAL_FLOAT(5.5f, out.glimit.soft_g);
    TEST_ASSERT_EQUAL_FLOAT(0.099f, out.stab.angle_roll.kp);
}

void test_deserialize_rejects_bad_crc() {
    ControllerConfig cfg{};
    uint8_t buf[kConfigBlobSize];
    uint16_t n = serializeConfig(cfg, buf, sizeof(buf));
    buf[n - 1] ^= 0xFF;  // 破坏 CRC
    ControllerConfig out{};
    TEST_ASSERT_FALSE(deserializeConfig(buf, n, out));
}

void test_deserialize_rejects_wrong_version() {
    ControllerConfig cfg{};
    uint8_t buf[kConfigBlobSize];
    uint16_t n = serializeConfig(cfg, buf, sizeof(buf));
    buf[0] = 0xEE;  // 破坏 magic/version
    ControllerConfig out{};
    TEST_ASSERT_FALSE(deserializeConfig(buf, n, out));
}

void test_backend_save_load_via_memory() {
    MemoryConfigBackend backend;
    ConfigStore store(&backend);
    ControllerConfig cfg{};
    cfg.glimit.hard_g = 8.0f;
    TEST_ASSERT_TRUE(store.save(cfg));
    ControllerConfig out{};
    TEST_ASSERT_TRUE(store.load(out));
    TEST_ASSERT_EQUAL_FLOAT(8.0f, out.glimit.hard_g);
}

void test_load_returns_false_when_empty() {
    MemoryConfigBackend backend;  // 空
    ConfigStore store(&backend);
    ControllerConfig out{};
    TEST_ASSERT_FALSE(store.load(out));
}
```
在 main 注册：
```cpp
    RUN_TEST(test_serialize_roundtrip_preserves_config);
    RUN_TEST(test_deserialize_rejects_bad_crc);
    RUN_TEST(test_deserialize_rejects_wrong_version);
    RUN_TEST(test_backend_save_load_via_memory);
    RUN_TEST(test_load_returns_false_when_empty);
```

- [ ] **Step 2: 运行确认失败**

Run: `cmake --build build && ctest --test-dir build -R test_config_store --output-on-failure`
Expected: 编译失败，找不到 `config/config_store.h`

- [ ] **Step 3: 写 header**

`core/config/config_store.h`：
```cpp
#pragma once
#include <cstdint>
#include <cstddef>

namespace wp {

struct ControllerConfig;  // 前置声明，避免头循环依赖

// blob 布局：[magic(2) | version(1) | size(1) | payload(memcpy ControllerConfig) | crc16(2)]
constexpr uint16_t kConfigMagic   = 0x5750;  // 字母 W,P
constexpr uint8_t  kConfigVersion = 1;
constexpr int      kConfigHeaderBytes = 4;
constexpr int      kConfigCrcBytes    = 2;

// blob 上限：头 + payload(sizeof ControllerConfig) + crc。给足余量。
constexpr int kConfigBlobSize = 512;

uint16_t crc16_ccitt(const uint8_t* data, size_t len);

// 序列化/反序列化纯函数。serialize 返回写入字节数（0=缓冲不足）；
// deserialize 校验 magic/version/size/crc，全过才写 out 并返回 true。
uint16_t serializeConfig(const ControllerConfig& cfg, uint8_t* buf, size_t cap);
bool     deserializeConfig(const uint8_t* buf, uint16_t len, ControllerConfig& out);

// 平台持久化后端（NVS/文件）抽象。core 不碰具体 I/O。
struct IConfigBackend {
    virtual ~IConfigBackend() = default;
    virtual bool write(const uint8_t* buf, uint16_t len) = 0;
    virtual uint16_t read(uint8_t* buf, uint16_t cap) = 0;  // 返回读到字节数，0=空
};

// PC/测试用内存后端
class MemoryConfigBackend : public IConfigBackend {
public:
    bool write(const uint8_t* buf, uint16_t len) override;
    uint16_t read(uint8_t* buf, uint16_t cap) override;
private:
    uint8_t store_[kConfigBlobSize];
    uint16_t len_ = 0;
};

// 串起序列化 + 后端
class ConfigStore {
public:
    explicit ConfigStore(IConfigBackend* backend) : backend_(backend) {}
    bool save(const ControllerConfig& cfg);
    bool load(ControllerConfig& out);
private:
    IConfigBackend* backend_;
};

}  // namespace wp
```

- [ ] **Step 4: 写 .cpp**

`core/config/config_store.cpp`：
```cpp
#include "config/config_store.h"
#include "controller.h"   // 完整 ControllerConfig 定义
#include <cstring>

namespace wp {

uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; ++b)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}

uint16_t serializeConfig(const ControllerConfig& cfg, uint8_t* buf, size_t cap) {
    const uint16_t payload = static_cast<uint16_t>(sizeof(ControllerConfig));
    const uint16_t total = kConfigHeaderBytes + payload + kConfigCrcBytes;
    if (cap < total) return 0;
    buf[0] = static_cast<uint8_t>(kConfigMagic & 0xFF);
    buf[1] = static_cast<uint8_t>(kConfigMagic >> 8);
    buf[2] = kConfigVersion;
    buf[3] = static_cast<uint8_t>(payload);  // 假定 payload <= 255；超则需扩 size 字段
    std::memcpy(buf + kConfigHeaderBytes, &cfg, payload);
    uint16_t crc = crc16_ccitt(buf, kConfigHeaderBytes + payload);
    buf[kConfigHeaderBytes + payload]     = static_cast<uint8_t>(crc & 0xFF);
    buf[kConfigHeaderBytes + payload + 1] = static_cast<uint8_t>(crc >> 8);
    return total;
}

bool deserializeConfig(const uint8_t* buf, uint16_t len, ControllerConfig& out) {
    const uint16_t payload = static_cast<uint16_t>(sizeof(ControllerConfig));
    const uint16_t total = kConfigHeaderBytes + payload + kConfigCrcBytes;
    if (len < total) return false;
    uint16_t magic = static_cast<uint16_t>(buf[0]) | (static_cast<uint16_t>(buf[1]) << 8);
    if (magic != kConfigMagic) return false;
    if (buf[2] != kConfigVersion) return false;
    if (buf[3] != static_cast<uint8_t>(payload)) return false;
    uint16_t want = crc16_ccitt(buf, kConfigHeaderBytes + payload);
    uint16_t got = static_cast<uint16_t>(buf[kConfigHeaderBytes + payload]) |
                   (static_cast<uint16_t>(buf[kConfigHeaderBytes + payload + 1]) << 8);
    if (want != got) return false;
    std::memcpy(&out, buf + kConfigHeaderBytes, payload);
    return true;
}

bool MemoryConfigBackend::write(const uint8_t* buf, uint16_t len) {
    if (len > kConfigBlobSize) return false;
    std::memcpy(store_, buf, len);
    len_ = len;
    return true;
}

uint16_t MemoryConfigBackend::read(uint8_t* buf, uint16_t cap) {
    if (len_ == 0 || cap < len_) return 0;
    std::memcpy(buf, store_, len_);
    return len_;
}

bool ConfigStore::save(const ControllerConfig& cfg) {
    uint8_t buf[kConfigBlobSize];
    uint16_t n = serializeConfig(cfg, buf, sizeof(buf));
    if (n == 0) return false;
    return backend_->write(buf, n);
}

bool ConfigStore::load(ControllerConfig& out) {
    uint8_t buf[kConfigBlobSize];
    uint16_t n = backend_->read(buf, sizeof(buf));
    if (n == 0) return false;
    return deserializeConfig(buf, n, out);
}

}  // namespace wp
```

- [ ] **Step 5: 运行确认通过**

Run: `cmake --build build && ctest --test-dir build -R test_config_store --output-on-failure`
Expected: PASS (7/7 累计)

- [ ] **Step 6: 提交**

```bash
git add core/config/config_store.h core/config/config_store.cpp test/test_config_store/
git commit -m "feat(config): NVS-ready config serialization (POD memcpy + version + CRC16)"
```

---

## Task 6: Controller 接入 SensorBundle（保留旧路径兼容）

**Files:**
- Modify: `core/controller.h`
- Modify: `core/controller.cpp`
- Test: `test/test_controller/test_controller.cpp`（追加）

**说明：** 给 `Controller` 加一个吃 `SensorBundle` 的 `updateFromBundle()` 路径，内部把 bundle 拆进现有 `ControlInput` 字段后复用原 `update()`。旧 `update(ControlInput)` 完全不动（mixer/glimit/SITL 数值不变），新路径只是上游适配。这样 SensorFrontend → Controller 链路打通，且零回归。

- [ ] **Step 1: 追加失败测试**

在 `test_controller.cpp` 顶部加 include：
```cpp
#include "sensors/sensor_frontend.h"
```
追加测试：
```cpp
void test_update_from_bundle_matches_control_input() {
    // 同样的传感器数 + 通道，updateFromBundle 与手填 ControlInput 应得到相同 servo
    Controller a, b;
    SensorBundle bundle{};
    bundle.imu.gyro_x = 5.0f; bundle.imu.accel_z = 1.0f; bundle.imu.valid = true;

    uint16_t ch[kNumChannels];
    for (int i = 0; i < kNumChannels; ++i) ch[i] = 1500;
    ch[4] = 1500;  // Angle 模式
    float dt = 0.002f;

    ServoCommand viaBundle = a.updateFromBundle(ch, bundle, dt, true);

    ControlInput in{};
    for (int i = 0; i < kNumChannels; ++i) in.channels[i] = ch[i];
    in.imu = bundle.imu; in.mag = bundle.mag; in.baro = bundle.baro;
    in.dt = dt; in.link_ok = true;
    ServoCommand viaInput = b.update(in);

    for (int i = 0; i < kNumServos; ++i)
        TEST_ASSERT_EQUAL_UINT16(viaInput.servo[i], viaBundle.servo[i]);
}
```
在 main 注册：
```cpp
    RUN_TEST(test_update_from_bundle_matches_control_input);
```

- [ ] **Step 2: 运行确认失败**

Run: `cmake --build build && ctest --test-dir build -R test_controller --output-on-failure`
Expected: 编译失败，`Controller` 无 `updateFromBundle` 成员

- [ ] **Step 3: 在 controller.h 加声明**

在 `core/controller.h` 的 `ServoCommand update(const ControlInput& in);` 行后加：
```cpp
    // 从 SensorFrontend 的 SensorBundle 直接喂控制器（拆进 ControlInput 后复用 update）。
    ServoCommand updateFromBundle(const uint16_t channels[kNumChannels],
                                  const struct SensorBundle& bundle,
                                  float dt, bool link_ok);
```
并在文件顶部 include 区加：
```cpp
#include "sensors/sensor_frontend.h"
```

- [ ] **Step 4: 在 controller.cpp 实现**

在 `core/controller.cpp` 的 `update()` 函数定义之后追加：
```cpp
ServoCommand Controller::updateFromBundle(const uint16_t channels[kNumChannels],
                                          const SensorBundle& bundle,
                                          float dt, bool link_ok) {
    ControlInput in{};
    for (int i = 0; i < kNumChannels; ++i) in.channels[i] = channels[i];
    in.imu  = bundle.imu;
    in.mag  = bundle.mag;
    in.baro = bundle.baro;
    in.dt = dt;
    in.link_ok = link_ok;
    return update(in);
}
```

- [ ] **Step 5: 运行确认通过（含全量回归）**

Run: `cmake --build build && ctest --test-dir build --output-on-failure`
Expected: 全部 PASS（含既有 mixer/glimit/controller 测试，零回归）

- [ ] **Step 6: 提交**

```bash
git add core/controller.h core/controller.cpp test/test_controller/
git commit -m "feat(core): Controller.updateFromBundle bridges SensorFrontend output"
```

---

## Task 7: SITL 桥经新抽象喂数 + 全链路回归

**Files:**
- Modify: `hal_sitl/core_c_api.cpp`
- Verify: `hal_sitl/check_angle_hold.py`、`hal_sitl/check_turn_hold.py`

**说明：** 把 SITL C API 内部改为构造 `SensorBundle` + 调 `updateFromBundle`，证明 SITL 走新抽象后数值与基线完全一致（angle-hold recovery、turn-hold tracking 不变）。C API 签名不变，Python 侧零改动。

- [ ] **Step 1: 改 core_c_api.cpp 走 bundle**

将 `wp_controller_update` 内部填 `ControlInput` 的部分改为填 `SensorBundle` + `updateFromBundle`。原逻辑（`core_c_api.cpp:20-25` 一带）：
```cpp
    // 旧：in.imu.gyro_x = imu6[0]; ... ctrl->update(in);
```
改为：
```cpp
    wp::SensorBundle bundle{};
    bundle.imu.gyro_x = imu6[0]; bundle.imu.gyro_y = imu6[1]; bundle.imu.gyro_z = imu6[2];
    bundle.imu.accel_x = imu6[3]; bundle.imu.accel_y = imu6[4]; bundle.imu.accel_z = imu6[5];
    bundle.imu.valid = true;
    bundle.mag.mag_x = mag3[0]; bundle.mag.mag_y = mag3[1]; bundle.mag.mag_z = mag3[2];
    bundle.mag.valid = (mag_valid != 0);
    bundle.baro.altitude_m = baro_alt_m; bundle.baro.valid = (baro_valid != 0);

    wp::ServoCommand out = ctrl->updateFromBundle(channels, bundle, dt_s, link_ok != 0);
```
（`#include "sensors/sensor_frontend.h"` 已通过 controller.h 间接引入；若未，显式加。变量名 `ctrl`/`channels` 以现有文件实际为准。）

- [ ] **Step 2: 重建 DLL**

Run: `cmake --build build`
Expected: 成功生成 `build/libwp_core_capi.dll`

- [ ] **Step 3: 跑 angle-hold 验收**

Run: `python3 hal_sitl/check_angle_hold.py`
Expected: ANGLE-HOLD RECOVERY 数值与基线一致（扰动 ~7.0° → 收敛 ~1.3°），PASS

- [ ] **Step 4: 跑 turn-hold 验收**

Run: `python3 hal_sitl/check_turn_hold.py`
Expected: TURN-HOLD TRACKING 与基线一致（truth ~47.9° / est-error ~9.5°），PASS

- [ ] **Step 5: 全量单测回归**

Run: `ctest --test-dir build --output-on-failure`
Expected: 全部 PASS

- [ ] **Step 6: 提交**

```bash
git add hal_sitl/core_c_api.cpp
git commit -m "refactor(sitl): feed controller via SensorBundle, baseline-identical numbers"
```

---

## Self-Review 记录

- **Spec 覆盖：** §4.2.1 模块化传感器层 → Task 1-3（类型/接口/Frontend）；§6.0 档位 → Task 3 detectTier；§6.3 引脚 → Task 4 board_config；"全写好先不使能"双层门控 → Task 4 能力宏 + Task 3 probe；运行时配置+NVS → Task 5；core 接入 + SITL 零回归 → Task 6-7。**本次不覆盖**（明确排除）：ICM/BMP 真驱动实现、阶段 3 算法、max 档表决、WiFi WebUI——均插入本次插座，列为后续。
- **类型一致性：** `SensorBundle`/`SensorTier`/5 接口名/`serializeConfig`/`deserializeConfig`/`ConfigStore`/`updateFromBundle` 跨任务一致。
- **占位符：** 无 TBD/TODO；每个代码步含完整代码。
- **已知后续（非本计划缺口）：** 板载真 NVS backend（`PreferencesConfigBackend`）、ICM-42688-P/BMP390L 驱动（移植 ElrsRX `imu_icm42688.*`/`baro_bmp390.*` 适配到 `IGyroAccel`/`IBarometer`）、IST8310/GPS 驱动（到货后）、串口遥测打印（HAL 层 `#if WP_DEBUG_SERIAL`）。
