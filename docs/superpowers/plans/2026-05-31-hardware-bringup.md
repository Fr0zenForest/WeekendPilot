# 硬件 Bring-up：IMU + 气压计真驱动 + Frontend 接线 + NVS backend 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 移植 ICM-42688-P / BMP390L 真驱动（core 纯解码 PC 全测 + hal 门控 I/O），把 SensorFrontend 接进 SITL 和板载入口（替换手填 bundle / 纯遥控直通），并加 ESP32 NVS 配置后端——让传感器骨架真正"通电"，base 档在 SITL 用合成数据闭环、在真硬件按需 enable。

**Architecture:** 延续两层 + 双层门控。core 新增 `icm42688_decode`（raw[14]→ImuSample）、`bmp390_compensate`（raw+trim→Pa，datasheet float 公式）、`baro_altitude`（Pa→相对高度 m，**新代码，国际标准大气公式**）三个纯函数，PC 全测。hal_esp32 新增 `Icm42688:IGyroAccel`、`Bmp390:IBarometer` 两个门控驱动 + `NvsConfigBackend:IConfigBackend`。SITL 的 `core_c_api` 改用 `SensorFrontend`+Mock 喂数据（让 `if constexpr(kHas*)` 在 PC 真正合拢）；板载 `main.cpp` 实例化 Frontend 接真驱动，替换 `imu.valid=false` 直通。

**Tech Stack:** C++17（core `-Wall -Wextra -Werror`）、Unity 单测、CMake/Ninja（PC）、PlatformIO/Arduino-ESP32（板载 `Wire`/`Preferences`）。常量/公式移植自 ElrsRX（其本身移植自 datasheet/madflight）。

---

## ⚠️ 数据真实性声明（务必通读）

| 部分 | 数据来源 | 可信边界 |
|------|---------|---------|
| **ICM raw→ImuSample 解码** | scales 1/16.4(gyro)、1/4096(accel)、字节序，移植自 ElrsRX `imu_icm42688.cpp` | ✅ 字节装配+标度数学 PC 可测；**轴向极性（X前/Y右/Z下 NED）须装机后实物核对**，零偏由 AHRS 校准吸收 |
| **BMP390 补偿公式** | datasheet §8.4/8.6 float trim，移植自 ElrsRX `baro_bmp390.cpp` | ✅ 给定 trim+raw 的补偿数学 PC 可测（用 datasheet 示例 trim）；**真实气压噪声/温漂/地面效应未测** |
| **气压→高度换算** | **新代码**：国际标准大气（ISA）正压公式 | ⚠️ 公式数学 PC 可测；**海平面基准气压须实物起飞前置零**，否则只有相对高度有意义 |
| **SITL Frontend 通路** | Mock 喂合成数据（同既有 synth_mag/理想气压） | ⚠️ 验证的是"门控+Frontend+解算"通路逻辑，**非真实传感器信号** |
| **hal 驱动 I/O** | Icm42688/Bmp390/NvsBackend | ⚠️ 真 I2C/NVS，PC 不编不测，**端到端须上板** |

**核心可信结论：** 三个 core 解码/换算函数在 PC 上确定性可测；Frontend 接线在 SITL 用合成数据验证通路；真硬件读数、轴向、气压基准全部待实物。

---

## 文件结构

| 文件 | 职责 | 层 | 可测性 |
|------|------|-----|--------|
| `core/sensors/icm42688_decode.h/.cpp` | raw[14] 突发 → ImuSample（gyro deg/s, accel g）；寄存器常量 | core 纯逻辑 | ✅ PC 全测 |
| `core/sensors/bmp390_compensate.h/.cpp` | NVM trim 解析 + 温压补偿（datasheet float）→ Pa；寄存器常量 | core 纯逻辑 | ✅ PC 全测 |
| `core/sensors/baro_altitude.h/.cpp` | 气压(Pa) + 基准 → 相对高度(m)，ISA 公式 | core 纯逻辑 | ✅ PC 全测 |
| `hal_esp32/src/drivers/icm42688.h/.cpp` | `Icm42688 : IGyroAccel`，I2C 读寄存器喂 decode | hal 真 I/O | ❌ 无板不可测 |
| `hal_esp32/src/drivers/bmp390.h/.cpp` | `Bmp390 : IBarometer`，I2C + compensate + altitude | hal 真 I/O | ❌ 无板不可测 |
| `hal_esp32/src/config/nvs_config_backend.h/.cpp` | `NvsConfigBackend : IConfigBackend` over Preferences | hal 真 I/O | ❌ 无板不可测 |
| `hal_sitl/core_c_api.cpp` | 改用 SensorFrontend+Mock 喂 bundle（替换手填） | SITL | ✅ SITL 回归 |
| `hal_esp32/src/main.cpp` | 实例化 Frontend + 真驱动，替换 imu.valid=false | 板载 | ❌ 无板不可测 |
| `test/test_icm42688_decode/` `test_bmp390_compensate/` `test_baro_altitude/` | 三个解码/换算单测 | 测试 | ✅ |

**门控：** PC/测试构建 `kHasImu=kHasBaro=1`（全开）；DevKit 板 `WP_HAS_IMU/BARO` 已在 board_config 默认开（IMU/Baro 你已有硬件）。hal 驱动 `.cpp` 用 `#if WP_HAS_*` 包裹。

---

## Task 1: ICM-42688-P 解码纯函数（raw[14] → ImuSample）

**Files:**
- Create: `core/sensors/icm42688_decode.h`
- Create: `core/sensors/icm42688_decode.cpp`
- Test: `test/test_icm42688_decode/test_icm42688_decode.cpp`
- Modify: `test/CMakeLists.txt`

> ⚠️ scales(1/16.4 gyro, 1/4096 accel)+字节序移植自 ElrsRX `imu_icm42688.cpp`（其配置 ±2000dps/±8g）。字节装配数学可测；**轴向极性须装机后核对**。

- [ ] **Step 1: 头文件**

`core/sensors/icm42688_decode.h`：

```cpp
#pragma once
#include "types.h"
#include <cstdint>

namespace wp {

// ICM-42688-P 寄存器（移植自 ElrsRX imu_icm42688.cpp，⚠️ 配置 ±2000dps/±8g/1kHz）。
namespace icm42688 {
    constexpr uint8_t kI2cAddr   = 0x68;
    constexpr uint8_t kRegWhoAmI = 0x75;
    constexpr uint8_t kWhoAmIVal = 0x47;
    constexpr uint8_t kRegDeviceConfig = 0x11;  // 0x01=软复位
    constexpr uint8_t kRegPwrMgmt0     = 0x4E;  // 0x0F=陀螺+加速度低噪声
    constexpr uint8_t kRegGyroConfig0  = 0x4F;  // 0x06=±2000dps,1kHz
    constexpr uint8_t kRegAccelConfig0 = 0x50;  // (0x1<<5)|0x06=±8g,1kHz
    constexpr uint8_t kRegGyroAccelConfig0 = 0x52;  // (0x4<<4)|0x4=BW≈ODR/4
    constexpr uint8_t kRegTempData1    = 0x1D;  // 突发起点: TEMP(2)+ACCEL(6)+GYRO(6)=14B

    constexpr float kGyroScale  = 1.0f / 16.4f;    // ±2000dps -> 16.4 LSB/(°/s)
    constexpr float kAccelScale = 1.0f / 4096.0f;  // ±8g -> 4096 LSB/g
}

// 把从 TEMP_DATA1 突发读的 14 字节解码为 ImuSample（gyro deg/s, accel g, 机体系）。
// 布局：[0..1]=temp(跳过) [2..7]=accel XYZ(hi,lo) [8..13]=gyro XYZ(hi,lo)。
// out.valid=true。⚠️ 轴向极性须装机后核对（X前/Y右/Z下 NED），见 plan。
ImuSample icm42688Decode(const uint8_t raw[14]);

}  // namespace wp
```

- [ ] **Step 2: 失败测试**

`test/test_icm42688_decode/test_icm42688_decode.cpp`：

```cpp
#include "unity.h"
#include "sensors/icm42688_decode.h"
using namespace wp;

void setUp() {} void tearDown() {}

// ⚠️ scales 移植自 ElrsRX，仅验字节装配+标度数学，非实物。
void test_decode_accel_gyro_scaling() {
    // accel X = +4096 LSB -> +1.0 g ; gyro X = +16 LSB*... 用 16400 -> 1000 dps
    // 布局: [0..1]temp [2..3]ax [4..5]ay [6..7]az [8..9]gx [10..11]gy [12..13]gz, hi 在前
    uint8_t raw[14] = {0};
    // ax = +4096 = 0x1000
    raw[2] = 0x10; raw[3] = 0x00;
    // az = +4096 (静止平放 1g) = 0x1000
    raw[6] = 0x10; raw[7] = 0x00;
    // gx = +16400 = 0x4010 -> 16400/16.4 = 1000 dps
    raw[8] = 0x40; raw[9] = 0x10;
    ImuSample s = icm42688Decode(raw);
    TEST_ASSERT_TRUE(s.valid);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 1.0f, s.accel_x);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 1.0f, s.accel_z);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 1000.0f, s.gyro_x);
}

void test_decode_negative_signed() {
    // ay = -4096 = 0xF000 (two's complement int16) -> -1.0 g
    uint8_t raw[14] = {0};
    raw[4] = 0xF0; raw[5] = 0x00;
    ImuSample s = icm42688Decode(raw);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, -1.0f, s.accel_y);
}

void test_whoami_constants() {
    TEST_ASSERT_EQUAL_UINT8(0x47, icm42688::kWhoAmIVal);
    TEST_ASSERT_EQUAL_UINT8(0x68, icm42688::kI2cAddr);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_decode_accel_gyro_scaling);
    RUN_TEST(test_decode_negative_signed);
    RUN_TEST(test_whoami_constants);
    return UNITY_END();
}
```

`test/CMakeLists.txt` 末尾加：`wp_add_test(test_icm42688_decode)`

- [ ] **Step 3: 运行确认失败**

Run: `cmake --preset dev && cmake --build build`
Expected: 链接失败（icm42688Decode 未定义）

- [ ] **Step 4: 实现**

`core/sensors/icm42688_decode.cpp`：

```cpp
#include "sensors/icm42688_decode.h"

namespace wp {

ImuSample icm42688Decode(const uint8_t raw[14]) {
    auto be16 = [](uint8_t hi, uint8_t lo) -> int16_t {
        return static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) |
                                    static_cast<uint16_t>(lo));
    };
    ImuSample s{};
    // [2..7] accel, [8..13] gyro，各轴 hi 在前
    s.accel_x = be16(raw[2],  raw[3])  * icm42688::kAccelScale;
    s.accel_y = be16(raw[4],  raw[5])  * icm42688::kAccelScale;
    s.accel_z = be16(raw[6],  raw[7])  * icm42688::kAccelScale;
    s.gyro_x  = be16(raw[8],  raw[9])  * icm42688::kGyroScale;
    s.gyro_y  = be16(raw[10], raw[11]) * icm42688::kGyroScale;
    s.gyro_z  = be16(raw[12], raw[13]) * icm42688::kGyroScale;
    s.valid = true;
    return s;
}

}  // namespace wp
```

- [ ] **Step 5: 运行确认通过**

Run: `cmake --preset dev && cmake --build build && ctest --test-dir build -R test_icm42688_decode --output-on-failure && ctest --test-dir build --output-on-failure`
Expected: test_icm42688_decode 3/3 PASS，全部 suites 绿。

- [ ] **Step 6: 提交**

```bash
git add core/sensors/icm42688_decode.h core/sensors/icm42688_decode.cpp test/test_icm42688_decode/ test/CMakeLists.txt
git commit -m "feat(sensors): ICM-42688-P raw[14] -> ImuSample decode (ported from ElrsRX, untested on HW)"
```

---

## Task 2: BMP390 温压补偿纯函数（trim + raw → Pa）

**Files:**
- Create: `core/sensors/bmp390_compensate.h`
- Create: `core/sensors/bmp390_compensate.cpp`
- Test: `test/test_bmp390_compensate/test_bmp390_compensate.cpp`
- Modify: `test/CMakeLists.txt`

> ⚠️ NVM trim 解析 + 补偿公式移植自 ElrsRX `baro_bmp390.cpp`（datasheet §8.4/8.6 float 版）。给定 trim+raw 的补偿数学可测；**真实气压噪声/温漂未测**。

- [ ] **Step 1: 头文件**

`core/sensors/bmp390_compensate.h`：

```cpp
#pragma once
#include <cstdint>

namespace wp {

namespace bmp390 {
    constexpr uint8_t kI2cAddr    = 0x76;
    constexpr uint8_t kI2cAddrAlt = 0x77;
    constexpr uint8_t kChipId     = 0x60;
    constexpr uint8_t kRegChipId  = 0x00;
    constexpr uint8_t kRegErr     = 0x02;
    constexpr uint8_t kRegData0   = 0x04;   // press(3) + temp(3) = 6 字节
    constexpr uint8_t kRegEvent   = 0x10;
    constexpr uint8_t kRegPwrCtrl = 0x1B;
    constexpr uint8_t kRegOsr     = 0x1C;
    constexpr uint8_t kRegOdr     = 0x1D;
    constexpr uint8_t kRegConfig  = 0x1F;
    constexpr uint8_t kRegNvmPar  = 0x31;   // 21 字节 trim
    constexpr uint8_t kRegCmd     = 0x7E;
    constexpr uint8_t kCmdSoftReset = 0xB6;
    constexpr int     kLenNvm     = 21;
    constexpr int     kLenData    = 6;
}

// datasheet §8.4 float trim（从 21 字节 NVM 解出）。
struct Bmp390Calib {
    float par_t1, par_t2, par_t3;
    float par_p1, par_p2, par_p3, par_p4, par_p5, par_p6;
    float par_p7, par_p8, par_p9, par_p10, par_p11;
    float t_lin;   // compensateTemperature 写，compensatePressure 读
};

// 21 字节 NVM → float trim（datasheet §8.4）。
Bmp390Calib bmp390ParseCalib(const uint8_t nvm[21]);
// datasheet §8.6 温度补偿，返回 °C，并更新 calib.t_lin（顺序依赖：先调它）。
float bmp390CompensateTemperature(uint32_t uncomp_temp, Bmp390Calib& calib);
// datasheet §8.6 压力补偿，返回 Pa（依赖上一步的 t_lin）。
float bmp390CompensatePressure(uint32_t uncomp_press, const Bmp390Calib& calib);
// 便捷：6 字节 DATA → (压力 Pa, 温度 °C)。raw_press/raw_temp 为 24-bit 小端。
struct Bmp390Reading { float pressure_pa; float temperature_c; bool valid; };
Bmp390Reading bmp390Decode(const uint8_t data[6], Bmp390Calib& calib);

}  // namespace wp
```

- [ ] **Step 2: 失败测试**

`test/test_bmp390_compensate/test_bmp390_compensate.cpp`。⚠️ 没有 Bosch 官方"原始码→Pa"金标向量，所以这里用**已知 trim + 已知 raw 跑全管线，断言落在物理合理区间**（温度室温档、压力近 1 atm），并验证 trim 解析的字节装配确定性。

```cpp
#include "unity.h"
#include "sensors/bmp390_compensate.h"
#include <cstring>
using namespace wp;

void setUp() {} void tearDown() {}

// ⚠️ 这是一组构造的典型 trim（量级取自 datasheet §8.4 范围），非某颗实物 NVM。
// 仅用于验证补偿管线数学自洽 + 输出物理合理，非端到端精度校验。
static void make_nvm(uint8_t nvm[21]) {
    std::memset(nvm, 0, 21);
    // par_t1 = u16(0)*256 ; 取 27500 -> par_t1=7.04e6
    nvm[0] = 0x6C; nvm[1] = 0x6B;          // 0x6B6C = 27500
    // par_t2 = u16(2)/2^30 ; 取 18000
    nvm[2] = 0x50; nvm[3] = 0x46;          // 0x4650 = 18000
    // par_t3 = i8(4)/2^48 ; 取 -3
    nvm[4] = 0xFD;                          // -3
    // par_p1 = (i16(5)-16384)/2^20 ; 取 i16=16000 -> 略负
    nvm[5] = 0x80; nvm[6] = 0x3E;          // 0x3E80 = 16000
    // par_p2 = (i16(7)-16384)/2^29 ; 取 16500
    nvm[7] = 0x74; nvm[8] = 0x40;          // 0x4074 = 16500
    // 其余 p3..p11 留 0（对补偿是高阶小项，置 0 不影响"物理合理"判定）
}

void test_temperature_then_pressure_pipeline() {
    uint8_t nvm[21]; make_nvm(nvm);
    Bmp390Calib c = bmp390ParseCalib(nvm);
    // 典型未补偿原始码（24-bit）：温度档对应室温附近、压力近 1atm。
    uint32_t raw_temp  = 8000000u;   // 经验量级
    uint32_t raw_press = 6500000u;
    float tC = bmp390CompensateTemperature(raw_temp, c);   // 先温度（更新 t_lin）
    float pPa = bmp390CompensatePressure(raw_press, c);
    // 物理合理区间（构造 trim 下宽松判定）：温度 -40..85°C，压力 30k..110k Pa
    TEST_ASSERT_TRUE(tC > -40.0f && tC < 85.0f);
    TEST_ASSERT_TRUE(pPa > 30000.0f && pPa < 110000.0f);
}

void test_calib_parse_is_deterministic() {
    uint8_t nvm[21]; make_nvm(nvm);
    Bmp390Calib a = bmp390ParseCalib(nvm);
    Bmp390Calib b = bmp390ParseCalib(nvm);
    TEST_ASSERT_EQUAL_FLOAT(a.par_t1, b.par_t1);
    TEST_ASSERT_EQUAL_FLOAT(a.par_p1, b.par_p1);
    // par_t1 = 27500*256 = 7.04e6
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 7040000.0f, a.par_t1);
}

void test_decode_zero_raw_invalid() {
    uint8_t nvm[21]; make_nvm(nvm);
    Bmp390Calib c = bmp390ParseCalib(nvm);
    uint8_t data[6] = {0,0,0,0,0,0};       // raw=0 -> 芯片未出数
    Bmp390Reading r = bmp390Decode(data, c);
    TEST_ASSERT_FALSE(r.valid);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_temperature_then_pressure_pipeline);
    RUN_TEST(test_calib_parse_is_deterministic);
    RUN_TEST(test_decode_zero_raw_invalid);
    return UNITY_END();
}
```

`test/CMakeLists.txt` 末尾加：`wp_add_test(test_bmp390_compensate)`

> 实现者注意：若 `test_temperature_then_pressure_pipeline` 的 raw 经验值落在区间外，**不要改公式**——调整测试里的 `raw_temp/raw_press` 经验码使输出落在物理区间（trim 是构造的，原始码无金标，目标是验管线自洽不是绝对精度）。把最终用的 raw 值写进注释。

- [ ] **Step 3: 运行确认失败**

Run: `cmake --preset dev && cmake --build build`
Expected: 链接失败。

- [ ] **Step 4: 实现（trim 解析）**

`core/sensors/bmp390_compensate.cpp`（先写 include + parseCalib，下一步续写补偿）：

```cpp
#include "sensors/bmp390_compensate.h"

namespace wp {

Bmp390Calib bmp390ParseCalib(const uint8_t buf[21]) {
    auto u16 = [&](int o) -> uint16_t {
        return (uint16_t)buf[o] | ((uint16_t)buf[o + 1] << 8);
    };
    auto i16 = [&](int o) -> int16_t {
        return (int16_t)((uint16_t)buf[o] | ((uint16_t)buf[o + 1] << 8));
    };
    auto i8 = [&](int o) -> int8_t { return (int8_t)buf[o]; };

    Bmp390Calib c{};
    // datasheet §8.4（除数为 2 的幂；移植自 ElrsRX baro_bmp390.cpp）
    c.par_t1  = (float)u16(0)  * 256.0f;
    c.par_t2  = (float)u16(2)  / 1073741824.0f;            // 2^30
    c.par_t3  = (float)i8(4)   / 281474976710656.0f;       // 2^48
    c.par_p1  = ((float)i16(5) - 16384.0f) / 1048576.0f;   // 2^20
    c.par_p2  = ((float)i16(7) - 16384.0f) / 536870912.0f; // 2^29
    c.par_p3  = (float)i8(9)   / 4294967296.0f;            // 2^32
    c.par_p4  = (float)i8(10)  / 137438953472.0f;          // 2^37
    c.par_p5  = (float)u16(11) * 8.0f;
    c.par_p6  = (float)u16(13) / 64.0f;
    c.par_p7  = (float)i8(15)  / 256.0f;
    c.par_p8  = (float)i8(16)  / 32768.0f;
    c.par_p9  = (float)i16(17) / 281474976710656.0f;       // 2^48
    c.par_p10 = (float)i8(19)  / 281474976710656.0f;       // 2^48
    c.par_p11 = (float)i8(20)  / 36893488147419103232.0f;  // 2^65
    c.t_lin = 0.0f;
    return c;
}

// WP_BMP390_COMP_PLACEHOLDER
}  // namespace wp
```

- [ ] **Step 4b: 实现（补偿 + decode），替换上面 `// WP_BMP390_COMP_PLACEHOLDER` 这一行**

```cpp
float bmp390CompensateTemperature(uint32_t uncomp_temp, Bmp390Calib& c) {
    // datasheet §8.6（移植自 ElrsRX）
    float pd1 = (float)((float)uncomp_temp - c.par_t1);
    float pd2 = pd1 * c.par_t2;
    c.t_lin = pd2 + (pd1 * pd1) * c.par_t3;
    return c.t_lin;
}

float bmp390CompensatePressure(uint32_t uncomp_press, const Bmp390Calib& c) {
    float pd1, pd2, pd3, pd4, po1, po2;
    pd1 = c.par_p6 * c.t_lin;
    pd2 = c.par_p7 * (c.t_lin * c.t_lin);
    pd3 = c.par_p8 * (c.t_lin * c.t_lin * c.t_lin);
    po1 = c.par_p5 + pd1 + pd2 + pd3;

    pd1 = c.par_p2 * c.t_lin;
    pd2 = c.par_p3 * (c.t_lin * c.t_lin);
    pd3 = c.par_p4 * (c.t_lin * c.t_lin * c.t_lin);
    po2 = (float)uncomp_press * (c.par_p1 + pd1 + pd2 + pd3);

    pd1 = (float)uncomp_press * (float)uncomp_press;
    pd2 = c.par_p9 + c.par_p10 * c.t_lin;
    pd3 = pd1 * pd2;
    pd4 = pd3 + ((float)uncomp_press * (float)uncomp_press * (float)uncomp_press) * c.par_p11;

    return po1 + po2 + pd4;   // Pa
}

Bmp390Reading bmp390Decode(const uint8_t data[6], Bmp390Calib& c) {
    Bmp390Reading r{};
    uint32_t raw_press = (uint32_t)data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16);
    uint32_t raw_temp  = (uint32_t)data[3] | ((uint32_t)data[4] << 8) | ((uint32_t)data[5] << 16);
    if (raw_press == 0 || raw_temp == 0) return r;   // 芯片未出数
    r.temperature_c = bmp390CompensateTemperature(raw_temp, c);  // 先温度（更新 t_lin）
    r.pressure_pa   = bmp390CompensatePressure(raw_press, c);
    r.valid = true;
    return r;
}
```

- [ ] **Step 5: 运行确认通过**

Run: `cmake --preset dev && cmake --build build && ctest --test-dir build -R test_bmp390_compensate --output-on-failure && ctest --test-dir build --output-on-failure`
Expected: test_bmp390_compensate 3/3 PASS，全 suites 绿。

- [ ] **Step 6: 提交**

```bash
git add core/sensors/bmp390_compensate.h core/sensors/bmp390_compensate.cpp test/test_bmp390_compensate/ test/CMakeLists.txt
git commit -m "feat(sensors): BMP390 trim parse + temp/press compensation (ported from ElrsRX, untested on HW)"
```

---

## Task 3: 气压→相对高度换算（ISA 公式，新代码）

**Files:**
- Create: `core/sensors/baro_altitude.h`
- Create: `core/sensors/baro_altitude.cpp`
- Test: `test/test_baro_altitude/test_baro_altitude.cpp`
- Modify: `test/CMakeLists.txt`

> ⚠️ **这是新代码（非移植）**，用国际标准大气（ISA）正压公式。`BaroSample.altitude_m` 是相对高度——须实物起飞前用 `setReference()` 锁当前气压为 0 基准；否则按标准海平面 101325 Pa 算绝对高度（误差受当日天气影响）。

ISA 公式（对流层 <11km）：`h = 44330 * (1 - (p/p0)^0.1903)`，p0 为基准气压(Pa)。相对高度 = h(p) - h(p_ref)，等价 `44330 * ((1-(p_ref/p0)^0.1903) ... )`；这里直接用 `h = 44330*(1-(p/p_ref)^0.1903)` 以 p_ref 为零点更直接。

- [ ] **Step 1: 头文件**

`core/sensors/baro_altitude.h`：

```cpp
#pragma once

namespace wp {

// 国际标准大气海平面气压 (Pa)。
constexpr float kSeaLevelPa = 101325.0f;

// 气压(Pa) → 相对参考点的高度(m)。ISA 对流层公式：
//   h = 44330 * (1 - (p / p_ref)^0.1903)
// p_ref 为零点基准气压；传 kSeaLevelPa 得相对标准海平面的绝对高度（受天气误差）。
// p<=0 或 p_ref<=0 返回 0。
float baroPressureToAltitude(float pressure_pa, float p_ref_pa);

// 有状态封装：起飞前 setReference() 锁基准，之后 altitude() 出相对高度。
class BaroAltitude {
public:
    void setReference(float p_ref_pa) { p_ref_ = (p_ref_pa > 0.0f) ? p_ref_pa : kSeaLevelPa; }
    float reference() const { return p_ref_; }
    float altitude(float pressure_pa) const { return baroPressureToAltitude(pressure_pa, p_ref_); }
private:
    float p_ref_ = kSeaLevelPa;   // 默认未置零 -> 绝对(标准海平面)高度
};

}  // namespace wp
```

- [ ] **Step 2: 失败测试**

`test/test_baro_altitude/test_baro_altitude.cpp`：

```cpp
#include "unity.h"
#include "sensors/baro_altitude.h"
using namespace wp;

void setUp() {} void tearDown() {}

// ⚠️ 新代码，ISA 公式数学可测；实物须起飞前置零基准。
void test_at_reference_is_zero() {
    // p == p_ref -> 0 m
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 0.0f, baroPressureToAltitude(101325.0f, 101325.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 0.0f, baroPressureToAltitude(95000.0f, 95000.0f));
}

void test_lower_pressure_is_higher_altitude() {
    // 气压降 -> 高度升（正）。标准大气下约 8.4 m/hPa 近地面。
    float h = baroPressureToAltitude(101325.0f - 100.0f, 101325.0f); // -1 hPa
    TEST_ASSERT_TRUE(h > 7.0f && h < 10.0f);   // ~8.4 m
}

void test_known_altitude_ratio() {
    // 海平面标准下 p=89876 Pa 对应约 1000 m（ISA 表值）。
    float h = baroPressureToAltitude(89876.0f, 101325.0f);
    TEST_ASSERT_FLOAT_WITHIN(15.0f, 1000.0f, h);
}

void test_reference_class_relative() {
    BaroAltitude ba;
    ba.setReference(98000.0f);             // 起飞点气压
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 0.0f, ba.altitude(98000.0f)); // 起飞点 = 0
    TEST_ASSERT_TRUE(ba.altitude(97000.0f) > 0.0f);              // 爬升
}

void test_invalid_pressure_zero() {
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, baroPressureToAltitude(0.0f, 101325.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, baroPressureToAltitude(95000.0f, 0.0f));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_at_reference_is_zero);
    RUN_TEST(test_lower_pressure_is_higher_altitude);
    RUN_TEST(test_known_altitude_ratio);
    RUN_TEST(test_reference_class_relative);
    RUN_TEST(test_invalid_pressure_zero);
    return UNITY_END();
}
```

`test/CMakeLists.txt` 末尾加：`wp_add_test(test_baro_altitude)`

- [ ] **Step 3: 运行确认失败**

Run: `cmake --preset dev && cmake --build build`
Expected: 链接失败。

- [ ] **Step 4: 实现**

`core/sensors/baro_altitude.cpp`：

```cpp
#include "sensors/baro_altitude.h"
#include <cmath>

namespace wp {

float baroPressureToAltitude(float pressure_pa, float p_ref_pa) {
    if (pressure_pa <= 0.0f || p_ref_pa <= 0.0f) return 0.0f;
    // ISA 对流层：h = 44330 * (1 - (p/p_ref)^(1/5.255))
    return 44330.0f * (1.0f - std::pow(pressure_pa / p_ref_pa, 0.1902949f));
}

}  // namespace wp
```

- [ ] **Step 5: 运行确认通过**

Run: `cmake --preset dev && cmake --build build && ctest --test-dir build -R test_baro_altitude --output-on-failure && ctest --test-dir build --output-on-failure`
Expected: test_baro_altitude 5/5 PASS，全 suites 绿。

- [ ] **Step 6: 提交**

```bash
git add core/sensors/baro_altitude.h core/sensors/baro_altitude.cpp test/test_baro_altitude/ test/CMakeLists.txt
git commit -m "feat(sensors): ISA pressure->relative-altitude conversion + reference zeroing"
```

---

## Task 4: hal_esp32 Icm42688 适配（IGyroAccel，I2C，门控）

**Files:**
- Create: `hal_esp32/src/drivers/icm42688.h`
- Create: `hal_esp32/src/drivers/icm42688.cpp`

> ⚠️ 真 I2C I/O，PC 不编不测。init 序列移植自 ElrsRX（datasheet §14.36：滤波/FSR/ODR 必须在 PWR_MGMT0 上电前配）。`#if WP_HAS_IMU` 门控。read 遵守 no-pollute 契约。

- [ ] **Step 1: 头文件 — `hal_esp32/src/drivers/icm42688.h`**

```cpp
#pragma once
#include "config/capabilities.h"
#if WP_HAS_IMU
#include "sensors/sensor_interfaces.h"
#include "sensors/icm42688_decode.h"
#include <Wire.h>

namespace wp {

// IGyroAccel over I2C（共享主总线）。1kHz ODR 连续模式，read() 突发 14 字节。
class Icm42688 : public IGyroAccel {
public:
    explicit Icm42688(TwoWire& bus, uint8_t addr = icm42688::kI2cAddr)
        : bus_(bus), addr_(addr) {}
    bool probe() override;   // 读 WHO_AM_I == 0x47
    bool init()  override;   // 软复位 + 配置 + 上电（顺序敏感）
    bool read(ImuSample& out) override;
private:
    TwoWire& bus_;
    uint8_t  addr_;
    bool writeReg(uint8_t reg, uint8_t val);
    bool readRegs(uint8_t reg, uint8_t* buf, uint8_t n);
};

}  // namespace wp
#endif  // WP_HAS_IMU
```

- [ ] **Step 2: 实现 — `hal_esp32/src/drivers/icm42688.cpp`**

```cpp
#include "drivers/icm42688.h"
#if WP_HAS_IMU
#include <Arduino.h>

namespace wp {

bool Icm42688::writeReg(uint8_t reg, uint8_t val) {
    bus_.beginTransmission(addr_);
    bus_.write(reg); bus_.write(val);
    return bus_.endTransmission() == 0;
}

bool Icm42688::readRegs(uint8_t reg, uint8_t* buf, uint8_t n) {
    bus_.beginTransmission(addr_);
    bus_.write(reg);
    if (bus_.endTransmission(false) != 0) return false;
    if (bus_.requestFrom((int)addr_, (int)n) != n) return false;
    for (uint8_t i = 0; i < n; ++i) buf[i] = bus_.read();
    return true;
}

bool Icm42688::probe() {
    uint8_t who = 0;
    if (!readRegs(icm42688::kRegWhoAmI, &who, 1)) return false;
    return who == icm42688::kWhoAmIVal;
}

bool Icm42688::init() {
    if (!writeReg(icm42688::kRegDeviceConfig, 0x01)) return false;  // 软复位
    delay(10);
    uint8_t who = 0;
    if (!readRegs(icm42688::kRegWhoAmI, &who, 1) || who != icm42688::kWhoAmIVal) return false;
    // datasheet §14.36：滤波/FSR/ODR 必须在 PWR_MGMT0 上电前配（ElrsRX 同款顺序）
    if (!writeReg(icm42688::kRegGyroConfig0, 0x06)) return false;        // ±2000dps,1kHz
    if (!writeReg(icm42688::kRegAccelConfig0, (0x1 << 5) | 0x06)) return false;  // ±8g,1kHz
    if (!writeReg(icm42688::kRegGyroAccelConfig0, (0x4 << 4) | 0x4)) return false; // BW≈ODR/4
    if (!writeReg(icm42688::kRegPwrMgmt0, 0x0F)) return false;           // 陀螺+加速度低噪声
    delay(50);   // 陀螺上电后需 ≥30ms 稳定（datasheet §3.1）
    return true;
}

bool Icm42688::read(ImuSample& out) {
    uint8_t raw[14];
    if (!readRegs(icm42688::kRegTempData1, raw, 14)) return false;  // 失败前不碰 out
    out = icm42688Decode(raw);   // ⚠️ 轴向极性须装机后核对
    return out.valid;
}

}  // namespace wp
#endif  // WP_HAS_IMU
```

> read() 契约：唯一的 false 路径（readRegs 失败）在写 out 之前返回；decode 恒 valid=true。契约满足。

- [ ] **Step 3: 验证 PC 构建不受影响**

Run: `cmake --build build && ctest --test-dir build --output-on-failure`
Expected: 全 suites PASS（core 未动）。

- [ ] **Step 4: 提交**

```bash
git add hal_esp32/src/drivers/icm42688.h hal_esp32/src/drivers/icm42688.cpp
git commit -m "feat(hal): ICM-42688-P IGyroAccel over I2C (gated WP_HAS_IMU, untested on HW)"
```

---

## Task 5: hal_esp32 Bmp390 适配（IBarometer，I2C，门控）

**Files:**
- Create: `hal_esp32/src/drivers/bmp390.h`
- Create: `hal_esp32/src/drivers/bmp390.cpp`

> ⚠️ 真 I2C I/O，PC 不编不测。init 序列（OSR/ODR/IIR/normal 模式）移植自 ElrsRX。`#if WP_HAS_BARO` 门控。首次 read 成功时锁基准气压（起飞前置零），之后出相对高度。

- [ ] **Step 1: 头文件 — `hal_esp32/src/drivers/bmp390.h`**

```cpp
#pragma once
#include "config/capabilities.h"
#if WP_HAS_BARO
#include "sensors/sensor_interfaces.h"
#include "sensors/bmp390_compensate.h"
#include "sensors/baro_altitude.h"
#include <Wire.h>

namespace wp {

// IBarometer over I2C（共享主总线）。init 读 trim + 配置 normal 模式；
// read() 突发 6 字节 -> 补偿 -> 相对高度。首个有效读数锁基准气压（起飞前置零）。
class Bmp390 : public IBarometer {
public:
    explicit Bmp390(TwoWire& bus, uint8_t addr = bmp390::kI2cAddr)
        : bus_(bus), addr_(addr) {}
    bool probe() override;   // CHIPID == 0x60
    bool init()  override;   // 软复位 + 读 trim + 配置
    bool read(BaroSample& out) override;
private:
    TwoWire& bus_;
    uint8_t  addr_;
    Bmp390Calib calib_{};
    BaroAltitude alt_;
    bool ref_set_ = false;
    bool writeReg(uint8_t reg, uint8_t val);
    bool readRegs(uint8_t reg, uint8_t* buf, uint8_t n);
};

}  // namespace wp
#endif  // WP_HAS_BARO
```

- [ ] **Step 2: 实现 — `hal_esp32/src/drivers/bmp390.cpp`**

```cpp
#include "drivers/bmp390.h"
#if WP_HAS_BARO
#include <Arduino.h>

namespace wp {

bool Bmp390::writeReg(uint8_t reg, uint8_t val) {
    bus_.beginTransmission(addr_);
    bus_.write(reg); bus_.write(val);
    return bus_.endTransmission() == 0;
}

bool Bmp390::readRegs(uint8_t reg, uint8_t* buf, uint8_t n) {
    bus_.beginTransmission(addr_);
    bus_.write(reg);
    if (bus_.endTransmission(false) != 0) return false;
    if (bus_.requestFrom((int)addr_, (int)n) != n) return false;
    for (uint8_t i = 0; i < n; ++i) buf[i] = bus_.read();
    return true;
}

bool Bmp390::probe() {
    uint8_t id = 0;
    if (!readRegs(bmp390::kRegChipId, &id, 1)) return false;
    return id == bmp390::kChipId;
}

bool Bmp390::init() {
    if (!writeReg(bmp390::kRegCmd, bmp390::kCmdSoftReset)) return false;
    delay(5);
    uint8_t nvm[bmp390::kLenNvm];
    if (!readRegs(bmp390::kRegNvmPar, nvm, bmp390::kLenNvm)) return false;
    calib_ = bmp390ParseCalib(nvm);
    // 无人机预设（ElrsRX/datasheet §3.4.5）：OSR t×1 p×8，ODR 50Hz，IIR coeff3
    if (!writeReg(bmp390::kRegOsr, (0x0 << 3) | 0x3)) return false;  // x1 / x8
    if (!writeReg(bmp390::kRegOdr, 0x02)) return false;             // 50 Hz
    if (!writeReg(bmp390::kRegConfig, (0x2 << 1))) return false;    // IIR coeff 3
    // press_en|temp_en|normal(0x3<<4)
    if (!writeReg(bmp390::kRegPwrCtrl, (1u<<0)|(1u<<1)|(0x3<<4))) return false;
    delay(10);
    return true;
}

bool Bmp390::read(BaroSample& out) {
    uint8_t data[bmp390::kLenData];
    if (!readRegs(bmp390::kRegData0, data, bmp390::kLenData)) return false;  // 失败前不碰 out
    Bmp390Reading r = bmp390Decode(data, calib_);
    if (!r.valid) return false;            // 契约：无效不污染 out
    if (!ref_set_) { alt_.setReference(r.pressure_pa); ref_set_ = true; }  // 首读锁基准
    BaroSample s{};
    s.altitude_m = alt_.altitude(r.pressure_pa);
    s.valid = true;
    out = s;
    return true;
}

}  // namespace wp
#endif  // WP_HAS_BARO
```

- [ ] **Step 3: 验证 PC 构建不受影响**

Run: `cmake --build build && ctest --test-dir build --output-on-failure`
Expected: 全 suites PASS。

- [ ] **Step 4: 提交**

```bash
git add hal_esp32/src/drivers/bmp390.h hal_esp32/src/drivers/bmp390.cpp
git commit -m "feat(hal): BMP390 IBarometer over I2C + first-read reference zeroing (gated WP_HAS_BARO, untested on HW)"
```

---

## Task 6: ESP32 NVS 配置后端（IConfigBackend over Preferences）

**Files:**
- Create: `hal_esp32/src/config/nvs_config_backend.h`
- Create: `hal_esp32/src/config/nvs_config_backend.cpp`

> ⚠️ 真 NVS I/O，PC 不编不测。序列化逻辑已由 core `config_store` PC 验过；这里只做 Preferences 存取。无门控宏（配置存储恒可用），但用 Arduino 头故仅板载编译。

`IConfigBackend` 契约（来自 `core/config/config_store.h`）：`bool write(const uint8_t* buf, uint16_t len)` + `uint16_t read(uint8_t* buf, uint16_t cap)`（返回读到字节数，0=空）。

- [ ] **Step 1: 头文件 — `hal_esp32/src/config/nvs_config_backend.h`**

```cpp
#pragma once
#include "config/config_store.h"
#include <Preferences.h>

namespace wp {

// IConfigBackend over ESP32 NVS（Preferences）。namespace "wpcfg"，key "blob"。
class NvsConfigBackend : public IConfigBackend {
public:
    bool write(const uint8_t* buf, uint16_t len) override;
    uint16_t read(uint8_t* buf, uint16_t cap) override;
private:
    Preferences prefs_;
    static constexpr const char* kNs  = "wpcfg";
    static constexpr const char* kKey = "blob";
};

}  // namespace wp
```

- [ ] **Step 2: 实现 — `hal_esp32/src/config/nvs_config_backend.cpp`**

```cpp
#include "config/nvs_config_backend.h"

namespace wp {

bool NvsConfigBackend::write(const uint8_t* buf, uint16_t len) {
    if (!prefs_.begin(kNs, /*readOnly=*/false)) return false;
    size_t n = prefs_.putBytes(kKey, buf, len);
    prefs_.end();
    return n == len;
}

uint16_t NvsConfigBackend::read(uint8_t* buf, uint16_t cap) {
    if (!prefs_.begin(kNs, /*readOnly=*/true)) return 0;
    size_t stored = prefs_.getBytesLength(kKey);
    if (stored == 0 || stored > cap) { prefs_.end(); return 0; }
    size_t n = prefs_.getBytes(kKey, buf, stored);
    prefs_.end();
    return (uint16_t)n;
}

}  // namespace wp
```

- [ ] **Step 3: 验证 PC 构建不受影响**

Run: `cmake --build build && ctest --test-dir build --output-on-failure`
Expected: 全 suites PASS（此文件不进 PC 构建）。

- [ ] **Step 4: 提交**

```bash
git add hal_esp32/src/config/nvs_config_backend.h hal_esp32/src/config/nvs_config_backend.cpp
git commit -m "feat(hal): NVS config backend over Preferences (untested on HW)"
```

---

## Task 7: SITL 接 SensorFrontend（替换手填 bundle）

**Files:**
- Modify: `hal_sitl/core_c_api.cpp`

> ✅ 这是本计划唯一 SITL 可回归验证的接线任务。用 Mock 把 SITL 喂的 imu6/mag3/baro 经 SensorFrontend 走一遍，证明"Frontend+Mock+档位探测"通路逻辑，闭环数值必须与基线逐位一致。⚠️ Mock 喂的是合成数据（同既有 synth_mag/理想气压），非真实传感器。

当前 `core_c_api.cpp` 的 `wp_controller_update` 手填 `wp::SensorBundle bundle{}` 再调 `updateFromBundle`。改为：持有一个 `SensorFrontend` + 三个 Mock（imu/mag/baro），每次 update 把入参写进 Mock.sample，poll 出 bundle，再调 `updateFromBundle`。GPS/airspeed SITL 暂不喂（保持 bundle 默认 invalid，与现状一致）。

- [ ] **Step 1: 改 `wp_controller_create` 持有 Frontend+Mock**

把句柄从裸 `Controller*` 换成一个聚合 struct。替换文件顶部到 `wp_controller_destroy`：

```cpp
#include "core_c_api.h"
#include "controller.h"
#include "sensors/sensor_frontend.h"
#include "sensors/mock_sensors.h"
#include <cstring>

namespace {
// SITL 句柄：Controller + Frontend + 三个 Mock（喂 SITL 合成数据）。
struct SitlCore {
    wp::Controller controller;
    wp::SensorFrontend frontend;
    wp::MockGyroAccel imu;
    wp::MockMag mag;
    wp::MockBaro baro;
    SitlCore() {
        frontend.setGyroAccel(&imu);
        frontend.setMagnetometer(&mag);
        frontend.setBarometer(&baro);
        frontend.begin();   // probe+init 三个 Mock（present 默认 true）-> 探测档位
    }
};
}  // namespace

extern "C" {

void* wp_controller_create() { return new SitlCore(); }
void  wp_controller_destroy(void* h) { delete static_cast<SitlCore*>(h); }
```

- [ ] **Step 2: 改 `wp_controller_update` 走 Frontend**

替换 `wp_controller_update` 整个函数体：

```cpp
void wp_controller_update(void* h,
                          const unsigned short* channels,
                          const float* imu6,
                          const float* mag3, int mag_valid,
                          float baro_alt_m, int baro_valid,
                          float dt_s, int link_ok,
                          unsigned short* servos_out) {
    auto* c = static_cast<SitlCore*>(h);

    uint16_t ch[wp::kNumChannels];
    for (int i = 0; i < wp::kNumChannels; ++i)
        ch[i] = static_cast<uint16_t>(channels[i]);

    // 把 SITL 入参写进 Mock.sample（⚠️ 合成数据，非真实传感器信号）
    c->imu.sample.gyro_x = imu6[0]; c->imu.sample.gyro_y = imu6[1]; c->imu.sample.gyro_z = imu6[2];
    c->imu.sample.accel_x = imu6[3]; c->imu.sample.accel_y = imu6[4]; c->imu.sample.accel_z = imu6[5];
    c->imu.sample.valid = true;
    c->mag.sample.mag_x = mag3[0]; c->mag.sample.mag_y = mag3[1]; c->mag.sample.mag_z = mag3[2];
    c->mag.sample.valid = (mag_valid != 0);
    c->mag.present = (mag_valid != 0);   // mag_valid=0 时 Mock 装作不在
    c->baro.sample.altitude_m = baro_alt_m; c->baro.sample.valid = (baro_valid != 0);
    c->baro.present = (baro_valid != 0);

    wp::SensorBundle bundle{};
    c->frontend.poll(bundle);

    wp::ServoCommand out = c->controller.updateFromBundle(ch, bundle, dt_s, (link_ok != 0));
    std::memcpy(servos_out, out.servo, sizeof(out.servo));
}
```

> 注意：`mag.present`/`baro.present` 在 update 里改了，但 `begin()` 只在构造时跑过一次、档位已定。这里改 present 只影响 `poll()` 的 read 成败（Mock.read 在 !present 时返回 false → poll 置 invalid），与原手填 `bundle.mag.valid=(mag_valid!=0)` 行为等价。档位探测不需要每帧重算。

- [ ] **Step 3: `wp_controller_attitude` 改用新句柄**

替换该函数体内的 cast：

```cpp
void wp_controller_attitude(void* h, float* rpy_out) {
    auto* c = static_cast<SitlCore*>(h);
    wp::Attitude a = c->controller.attitude();
    rpy_out[0] = a.roll_deg; rpy_out[1] = a.pitch_deg; rpy_out[2] = a.yaw_deg;
}
```

- [ ] **Step 4: 重建 DLL + SITL 回归**

Run:
```bash
cmake --build build --target wp_core_capi
python3 hal_sitl/run_sitl.py --scenario recover --out /tmp/wp_r.csv && python3 hal_sitl/check_angle_hold.py /tmp/wp_r.csv
python3 hal_sitl/run_sitl.py --scenario turn --out /tmp/wp_t.csv && python3 hal_sitl/check_turn_hold.py /tmp/wp_t.csv
```
Expected: angle-hold ≈ 7.0°→1.3°，turn-hold ≈ 47.9°/9.5°（与基线逐位一致，证明 Frontend 通路零行为改变）。

- [ ] **Step 5: 提交**

```bash
git add hal_sitl/core_c_api.cpp
git commit -m "feat(sitl): route SITL data through SensorFrontend+Mock (baseline-identical)"
```

---

## Task 8: 板载 main.cpp 接 Frontend + 真驱动（替换 imu.valid=false 直通）

**Files:**
- Modify: `hal_esp32/src/main.cpp`

> ⚠️ 真硬件入口，PC 不编不测。无 GPS/罗盘/空速时它们 probe 失败、Frontend 不计入（自动降级），base 档需 IMU+Baro+Mag 全在；当前手头只有 IMU+Baro，故 Frontend 探测出的 tier 会是 None（缺 mag）——但 imu/baro 样本仍喂给 controller，Angle 模式 6DOF 退化可用（见 AHRS 的 6/9DOF 自动派发）。端到端须上板验证。

当前 `main.cpp` 的 `loop()` 手填 `wp::ControlInput in{}` 且 `in.imu.valid = false`（纯遥控直通）。改为：全局持有 SensorFrontend + 真驱动实例（门控），`setup()` 里 `Wire.begin()` + `frontend.begin()`，`loop()` 里 `frontend.poll(bundle)` 后调 `updateFromBundle`。

- [ ] **Step 1: 顶部加 include + 全局驱动实例（门控）**

在 `main.cpp` 现有 `#include "controller.h"` 之后插入：

```cpp
#include "sensors/sensor_frontend.h"
#include "sensors/sensor_bundle.h"
#include "config/board_config.h"
#include "config/capabilities.h"
#include <Wire.h>
#if WP_HAS_IMU
#include "drivers/icm42688.h"
#endif
#if WP_HAS_BARO
#include "drivers/bmp390.h"
#endif
#if WP_HAS_MAG
#include "drivers/ist8310.h"
#endif

static wp::SensorFrontend g_frontend;
#if WP_HAS_IMU
static wp::Icm42688 g_imu(Wire, wp::kAddrImu);
#endif
#if WP_HAS_BARO
static wp::Bmp390 g_baro(Wire, wp::kAddrBaro);
#endif
#if WP_HAS_MAG
static wp::Ist8310 g_mag(Wire, wp::kAddrMag);
#endif
```

- [ ] **Step 2: `setup()` 初始化 I2C 总线 + Frontend**

在 `setup()` 的 `setupPwm();` 之前插入：

```cpp
    Wire.begin(wp::kPinI2cSda, wp::kPinI2cScl);
    Wire.setClock(400000);
#if WP_HAS_IMU
    g_frontend.setGyroAccel(&g_imu);
#endif
#if WP_HAS_BARO
    g_frontend.setBarometer(&g_baro);
#endif
#if WP_HAS_MAG
    g_frontend.setMagnetometer(&g_mag);
#endif
    g_frontend.begin();   // probe+init 已注册驱动，缺的自动降级
    Serial.printf("[wp] sensor tier=%d imu=%d baro=%d mag=%d\n",
                  (int)g_frontend.tier(), g_frontend.has_imu,
                  g_frontend.has_baro, g_frontend.has_mag);
```

- [ ] **Step 3: `loop()` 走 Frontend → updateFromBundle**

替换 `loop()` 里从 `wp::ControlInput in{};` 到 `wp::ServoCommand out = g_controller.update(in);` 这一整段（含 imu.valid=false 那行）为：

```cpp
    uint16_t ch[wp::kNumChannels];
    for (int i = 0; i < wp::kNumChannels; ++i)
        ch[i] = link_ok ? g_channels[i] : 1500;

    wp::SensorBundle bundle{};
    g_frontend.poll(bundle);   // 缺失/读失败的样本 valid=false

    wp::ServoCommand out = g_controller.updateFromBundle(ch, bundle, 0.001f, link_ok);
```

（其余 `writeServoUs` 循环和 `delay(2)` 不变。）

- [ ] **Step 4: 验证 PC 构建不受影响 + 板载编译检查（若 pio 可用）**

PC 侧 main.cpp 不参与 CMake，确认核心不受影响：
```bash
cmake --build build && ctest --test-dir build --output-on-failure
```
Expected: 全 suites PASS。

若 PlatformIO 可用，做板载编译检查（不烧录）：`pio run -e weekendpilot_s3`。若 `pio` 未安装，记录"板载编译延后（无硬件/工具链）"——驱动均已门控，DevKit profile 下 IMU/Baro 开、Mag/GPS/Airspeed 关，编译应通过。

- [ ] **Step 5: 提交**

```bash
git add hal_esp32/src/main.cpp
git commit -m "feat(hal): wire SensorFrontend + real drivers into board main (replaces RC passthrough)"
```

---

## 收尾验证（全部任务完成后）

- [ ] **全量单测**

Run: `cmake --preset dev && cmake --build build && ctest --test-dir build --output-on-failure`
Expected: 17 套全 PASS（原 14 + test_icm42688_decode + test_bmp390_compensate + test_baro_altitude），core 全程 `-Wall -Wextra -Werror` 零警告。

- [ ] **SITL 回归**

Run:
```bash
python3 hal_sitl/run_sitl.py --scenario recover --out /tmp/wp_r.csv && python3 hal_sitl/check_angle_hold.py /tmp/wp_r.csv
python3 hal_sitl/run_sitl.py --scenario turn --out /tmp/wp_t.csv && python3 hal_sitl/check_turn_hold.py /tmp/wp_t.csv
```
Expected: angle-hold ≈ 7.0°→1.3°，turn-hold ≈ 47.9°/9.5°（与基线一致）。

## 本计划交付边界（诚实总结）

**✅ 可信交付（PC 确定性可测）：**
- ICM-42688-P raw[14]→ImuSample 解码（scales+字节序）
- BMP390 trim 解析 + 温压补偿（datasheet float 公式）
- 气压→相对高度 ISA 换算 + 基准置零
- SITL 经 SensorFrontend+Mock 走通（闭环基线一致）

**⚠️ 已写好但未经硬件实测（标注在代码+测试+本文档三处）：**
- Icm42688/Bmp390 I2C 驱动（解码靠 core 单测背书，端到端读数待实物）
- ICM 轴向极性（X前/Y右/Z下 NED）须装机后核对
- BMP390 真实气压噪声/温漂、气压基准（须起飞前置零）
- NvsConfigBackend（序列化 PC 验过，真实 NVS 持久化待板）
- 板载 main.cpp Frontend 接线（无硬件不能跑；当前缺 mag，tier 探测会是 None，6DOF 退化）

**🔌 本计划不含（后续任务）：**
- IMU 陀螺零偏标定流程（开机静止采集，AHRS 已能吸收部分）
- 磁力计硬磁/软磁标定（须实物转圈）
- ConfigStore 在 main.cpp 的实际 load/save 调用接线（backend 已就位，调用留待运行时调参需求）
- 把真 IST8310 接进 SITL 替代 synth_mag（SITL 仍用合成磁场）
- 阶段 3 算法（Alt-Hold 等，气压高度链路本计划已就位，可起步）

## Self-Review 结果

- **Spec 覆盖：** IMU 真驱动 ✅ Task 1+4；气压计真驱动 ✅ Task 2+3+5；NVS backend ✅ Task 6；SITL 接 Frontend ✅ Task 7；板载接线 ✅ Task 8。五块全覆盖。
- **Placeholder 扫描：** Task 2 的 `// WP_BMP390_COMP_PLACEHOLDER` 是 Step 4→4b 的有意续写锚点，Step 4b 明确要求替换，非缺陷。BMP390 测试 raw 经验值允许实现者微调（已注明：调测试不调公式）。
- **类型一致性：** `ImuSample`(gyro_x/y/z,accel_x/y/z,valid)、`BaroSample`(altitude_m,valid)、`MagSample` 与 `types.h` 一致；`IGyroAccel/IBarometer`(probe/init/read)、`IConfigBackend`(write/read)、`SensorFrontend`(setGyroAccel/setBarometer/setMagnetometer/begin/poll/tier/has_*)、`Controller::updateFromBundle((uint16_t(&)[kNumChannels]),bundle,dt,link_ok)` 全部对现有定义核对一致。`icm42688Decode`/`bmp390ParseCalib`/`bmp390CompensateTemperature`/`bmp390CompensatePressure`/`bmp390Decode`/`baroPressureToAltitude`/`BaroAltitude` 命名跨任务一致。
- **常量溯源：** ICM（0x68/0x47/scales/init 序列）、BMP390（0x76/0x60/trim 公式/init 序列）均对 ElrsRX 源码核对；ISA 高度公式为标准大气模型。

## ⚠️ 上实机前的遗留项（最终 Opus 评审，Minor，不阻塞合并）

> 这两项不影响合并（已门控、已标注、SITL 用合成数据），但**真硬件起飞前应处理**：

1. **BMP390 首读锁基准时 IIR 滤波（coeff 3）仍在沉降**（`bmp390.cpp` read 的 `ref_set_` 首读 latch）。init 后第一帧可能处于滤波过渡态，使零点偏几米。**建议：** 锁基准前丢弃/平均前几帧。属起飞前置零流程的一部分。
2. **main.cpp 传 `dt=0.001f` 硬编码**，但 `loop()` 实际 ~500Hz（delay(2)+计算）。真传感器接入后 dt 误差直接影响 AHRS/PID 积分时序。**建议：** 用 `micros()` 差测实际 dt。属阶段 3 实机调参前置项。
