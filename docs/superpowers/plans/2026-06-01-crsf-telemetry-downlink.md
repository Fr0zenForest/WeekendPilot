# CRSF 遥测下行子系统 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 给 WeekendPilot 加 CRSF 遥测下行：把姿态/飞行模式（电池留接口）编码成标准 CRSF 帧，经现有 Serial1(TX=GPIO43) 半双工发回遥控器，EdgeTX/OpenTX 原生显示。

**Architecture:** 沿用两层——core 纯帧编码（`core/telemetry/crsf_telem.*`，PC ctest 全测，移植 TBS CRSF spec）+ HAL 发送调度（`hal_esp32/src/telemetry/crsf_telem_tx.*`，门控 WP_HAS_TELEMETRY，按帧分频不淹没上行 RC）。数据源复用 Controller 已有只读 getter（attitude/activeMode），控制核零改动。TelemSnapshot 纯值结构仿 diag/status_line.h 的 StatusSnapshot。

**Tech Stack:** C++17、ESP32-S3 Arduino（HardwareSerial）、CRSF 协议（CRC8/DVB-S2 poly 0xD5）、Unity（PC 单测）、CMake+Ninja（PC）、PlatformIO（板载）。

**设计文档：** `docs/superpowers/specs/2026-06-01-crsf-telemetry-downlink-design.md`

---

## 关键设计决策（实现前必读）

1. **core 纯编码、HAL 只发**：帧字节摆放/CRC/单位换算全在 core，PC 用 Unity 测（给定值→期望字节）。HAL 只做"到点取 snapshot→调 core 编码→写 Serial1"。沿用 ist8310_decode/icm42688_decode/status_line 模式。
2. **帧格式**（TBS CRSF spec）：`[sync=0xC8][len][type][payload][crc8]`，整帧 ≤64B。`len`=payload_len+2（type+crc）。`crc8`=CRC8/DVB-S2(poly 0xD5)，覆盖 type..payload 末尾。
3. **标准帧**：0x1E 姿态（3×int16 大端，rad×10000）、0x21 飞行模式（NUL 结尾 ASCII）。电池 0x08 留编码接口但本轮不接数据源。
4. **半双工调度**：ELRS 下行带宽有限，不每拍发。HAL 维护各帧"下次发送时刻"，loop 末尾 RC 解析后按到点发一个帧。频率初值：姿态 10Hz、模式 2Hz。
5. **门控 `WP_HAS_TELEMETRY`**：PC 默认开（验编码），板级默认 0。core 编码恒编入，HAL 发送层 #if 门控。
6. **TDD + 频繁提交**：core 件先写失败测试再实现；HAL 件只 pio 编译检查（无法 PC 测，沿用项目惯例）。

## 构建命令（core 任务）
```bash
cd /c/Repository/WeekendPilot
cmake --preset dev                              # 配置（改 CMakeLists 或加新 .cpp 后重跑）
cmake --build build
ctest --test-dir build --output-on-failure
```

## 板载编译检查（HAL 任务）
```bash
cd /c/Repository/WeekendPilot
pio run -e weekendpilot_s3 2>&1 | tail -6                                          # 默认门控关
PLATFORMIO_BUILD_FLAGS="-DWP_HAS_TELEMETRY=1" pio run -e weekendpilot_s3 2>&1 | tail -6   # 开门控
```

## 文件结构总览

**新建（core）：**
- `core/telemetry/crsf_telem.h` / `.cpp` — `crc8_dvbs2` + `buildFrame` + `encodeAttitude`/`encodeFlightMode`/`encodeBattery` + `TelemSnapshot` 值结构

**新建（HAL，门控）：**
- `hal_esp32/src/telemetry/crsf_telem_tx.h` / `.cpp` — `CrsfTelemetryTx` 发送调度器

**新建（测试）：**
- `test/test_crsf_telem/test_crsf_telem.cpp`

**修改：**
- `core/config/capabilities.h` — 加 WP_HAS_TELEMETRY + constexpr 镜像
- `hal_esp32/src/main.cpp` — 门控接入：填 TelemSnapshot + loop 末尾 tick
- `test/CMakeLists.txt` — 注册 test_crsf_telem

---

## Task 1: 能力宏门控 WP_HAS_TELEMETRY

**Files:**
- Modify: `core/config/capabilities.h`

- [ ] **Step 1: PC 默认全开块加宏**

`core/config/capabilities.h`，在 `#if !defined(BOARD_DEVKIT_S3)` 块内（最后一个 `WP_HAS_*` 之后，参 WP_HAS_LANDING_GEAR 的位置）追加：

```cpp
  #ifndef WP_HAS_TELEMETRY
    #define WP_HAS_TELEMETRY 1
  #endif
```

- [ ] **Step 2: 兜底默认 0 块加宏**

在兜底块（最后一个 `#ifndef WP_HAS_... #define ... 0 #endif` 之后，参 WP_HAS_LANDING_GEAR）追加：

```cpp
#ifndef WP_HAS_TELEMETRY
  #define WP_HAS_TELEMETRY 0
#endif
```

- [ ] **Step 3: constexpr 镜像块加镜像**

在 `namespace wp {` 内 `kHasLandingGear` 之后追加：

```cpp
constexpr bool kHasTelemetry = WP_HAS_TELEMETRY;
```

- [ ] **Step 4: 构建验证**

Run: `cmake --build build 2>&1 | tail -3`
Expected: 编译通过（新宏 PC 侧=1，无引用方）。

Run: `pio run -e weekendpilot_s3 2>&1 | tail -5`
Expected: SUCCESS（板载 WP_HAS_TELEMETRY=0）。

- [ ] **Step 5: Commit**

```bash
git add core/config/capabilities.h
git commit -m "feat(config): 加 WP_HAS_TELEMETRY 门控 + constexpr 镜像"
```

---

## Task 2: CRC8/DVB-S2 + buildFrame（core，TDD）

CRSF 帧封装的地基：CRC8 计算 + 通用帧组装。纯函数，PC 可测。

**Files:**
- Create: `core/telemetry/crsf_telem.h`
- Create: `core/telemetry/crsf_telem.cpp`
- Test: `test/test_crsf_telem/test_crsf_telem.cpp`
- Modify: `test/CMakeLists.txt`

- [ ] **Step 1: 写头文件（本任务部分，后续 Task 3 追加 encode*）**

`core/telemetry/crsf_telem.h`：

```cpp
#pragma once
#include <cstdint>
#include <cstddef>

namespace wp {

// CRSF 帧：[sync/addr][len][type][payload...][crc8]，整帧 <=64B。
// 移植自 TBS CRSF spec：crc8 = CRC8/DVB-S2(poly 0xD5)，覆盖 type..payload 末尾。
constexpr uint8_t  kCrsfSyncAddr = 0xC8;   // 飞控遥测下行 sync 字节
constexpr int      kCrsfMaxFrame = 64;     // 整帧字节上限

// CRC8/DVB-S2，多项式 0xD5。对 data[0..len) 计算。
uint8_t crc8_dvbs2(const uint8_t* data, size_t len);

// 组装一帧到 out：[addr][len][type][payload][crc8]。
//   payload/plen 可为 nullptr/0（无 payload 帧）。
//   crc8 覆盖 type..payload（即 out[2..2+plen]）。
//   返回写入的总字节数；out 容量不足（< plen+4）返回 0。
int buildFrame(uint8_t addr, uint8_t type, const uint8_t* payload, int plen,
               uint8_t* out, int cap);

}  // namespace wp
```

- [ ] **Step 2: 写失败测试**（见下方代码块 → `test/test_crsf_telem/test_crsf_telem.cpp`）

- [ ] **Step 2b: 注册测试**

`test/CMakeLists.txt` 末尾加：
```cmake
wp_add_test(test_crsf_telem)
```

- [ ] **Step 3: 跑测试确认失败**

Run: `cmake --preset dev && cmake --build build 2>&1 | tail -10`
Expected: 链接失败（crc8_dvbs2/buildFrame 未定义）。

**Step 2 测试代码** — `test/test_crsf_telem/test_crsf_telem.cpp`（本任务的 CRC/buildFrame 部分；Task 3 会向同文件追加 encode 测试与 RUN_TEST）：

```cpp
#include "unity.h"
#include "telemetry/crsf_telem.h"
#include <cstring>
using namespace wp;

void setUp() {} void tearDown() {}

// CRC8/DVB-S2 已知向量（poly 0xD5），Python 参考实现交叉验证
void test_crc8_known_vectors() {
    uint8_t a[] = {0x00};
    TEST_ASSERT_EQUAL_HEX8(0x00, crc8_dvbs2(a, 1));
    uint8_t b[] = {0x01, 0x02, 0x03};
    TEST_ASSERT_EQUAL_HEX8(0x3F, crc8_dvbs2(b, 3));
    uint8_t c[] = {0x21, 0x4F, 0x4B, 0x00};   // type 0x21 + "OK\0"
    TEST_ASSERT_EQUAL_HEX8(0x97, crc8_dvbs2(c, 4));
}

// buildFrame：[addr][len][type][payload][crc8]，len=plen+2，crc 覆盖 type..payload
void test_build_frame_layout() {
    uint8_t payload[] = {0x4F, 0x4B, 0x00};   // "OK\0"
    uint8_t out[kCrsfMaxFrame];
    int n = buildFrame(kCrsfSyncAddr, 0x21, payload, 3, out, sizeof(out));
    TEST_ASSERT_EQUAL_INT(7, n);              // addr+len+type+3payload+crc = 7
    TEST_ASSERT_EQUAL_HEX8(0xC8, out[0]);     // sync
    TEST_ASSERT_EQUAL_HEX8(5, out[1]);        // len = type+payload+crc = 1+3+1
    TEST_ASSERT_EQUAL_HEX8(0x21, out[2]);     // type
    TEST_ASSERT_EQUAL_HEX8(0x4F, out[3]);
    TEST_ASSERT_EQUAL_HEX8(0x00, out[5]);
    // crc 覆盖 out[2..6) = {0x21,0x4F,0x4B,0x00}
    TEST_ASSERT_EQUAL_HEX8(0x97, out[6]);
}

// 无 payload 帧：len=2（type+crc）
void test_build_frame_no_payload() {
    uint8_t out[kCrsfMaxFrame];
    int n = buildFrame(kCrsfSyncAddr, 0x21, nullptr, 0, out, sizeof(out));
    TEST_ASSERT_EQUAL_INT(4, n);              // addr+len+type+crc
    TEST_ASSERT_EQUAL_HEX8(2, out[1]);        // len
    TEST_ASSERT_EQUAL_HEX8(0x21, out[2]);
}

// 容量不足返回 0，不越界写
void test_build_frame_cap_guard() {
    uint8_t payload[] = {1,2,3};
    uint8_t out[4];                            // 需要 7，给 4
    int n = buildFrame(kCrsfSyncAddr, 0x21, payload, 3, out, sizeof(out));
    TEST_ASSERT_EQUAL_INT(0, n);
}
```

- [ ] **Step 4: 写实现**

`core/telemetry/crsf_telem.cpp`：

```cpp
#include "telemetry/crsf_telem.h"

namespace wp {

uint8_t crc8_dvbs2(const uint8_t* data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b)
            crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0xD5)
                               : static_cast<uint8_t>(crc << 1);
    }
    return crc;
}

int buildFrame(uint8_t addr, uint8_t type, const uint8_t* payload, int plen,
               uint8_t* out, int cap) {
    if (plen < 0) return 0;
    const int total = plen + 4;          // addr+len+type+payload+crc
    if (cap < total || total > kCrsfMaxFrame) return 0;
    out[0] = addr;
    out[1] = static_cast<uint8_t>(plen + 2);   // len = type+payload+crc
    out[2] = type;
    for (int i = 0; i < plen; ++i) out[3 + i] = payload[i];
    // crc 覆盖 type..payload = out[2 .. 2+1+plen)
    out[3 + plen] = crc8_dvbs2(out + 2, plen + 1);
    return total;
}

}  // namespace wp
```

- [ ] **Step 5: 跑测试确认通过**

Run: `cmake --build build && ctest --test-dir build -R test_crsf_telem --output-on-failure`
Expected: 4 个测试 PASS。

- [ ] **Step 6: 全量回归**

Run: `ctest --test-dir build --output-on-failure 2>&1 | tail -5`
Expected: 全过（26 套）。

- [ ] **Step 7: Commit**

```bash
git add core/telemetry/crsf_telem.h core/telemetry/crsf_telem.cpp test/test_crsf_telem/ test/CMakeLists.txt
git commit -m "feat(telemetry): CRSF crc8/DVB-S2 + buildFrame 帧封装 + 4 单测"
```

---

## Task 3: 帧编码函数 + TelemSnapshot（core，TDD）

姿态/飞行模式/电池帧编码 + 纯值快照结构。

**Files:**
- Modify: `core/telemetry/crsf_telem.h`（追加 TelemSnapshot + encode 声明）
- Modify: `core/telemetry/crsf_telem.cpp`（追加 encode 实现）
- Modify: `test/test_crsf_telem/test_crsf_telem.cpp`（追加 encode 测试 + RUN_TEST/main）

- [ ] **Step 1: 头文件追加（在 buildFrame 声明之后、`}  // namespace wp` 之前）**

```cpp
// 飞控遥测快照（纯值，由板载每帧组装；core 不碰硬件）。仿 diag/status_line.h。
struct TelemSnapshot {
    float roll_rad = 0.0f, pitch_rad = 0.0f, yaw_rad = 0.0f;
    const char* flight_mode = "OFF ";   // 4 字符模式串（指向静态字面量）
    // 电池（留接口，本轮可不填/不发）：
    float battery_v = 0.0f, battery_a = 0.0f;
    uint32_t battery_mah = 0;
    uint8_t  battery_pct = 0;
};

// 0x1E 姿态：payload = 3×int16 大端，单位 rad×10000（roll/pitch/yaw）。
// 返回整帧字节数；cap 不足返回 0。
int encodeAttitude(float roll_rad, float pitch_rad, float yaw_rad,
                   uint8_t* out, int cap);

// 0x21 飞行模式：payload = NUL 结尾 ASCII。mode 为 C 字符串（自动加 NUL）。
int encodeFlightMode(const char* mode, uint8_t* out, int cap);

// 0x08 电池：电压(int16,0.1V) 电流(int16,0.1A) 容量(uint24 大端,mAh) 剩余(uint8,%)。
int encodeBattery(float volts, float amps, uint32_t mah, uint8_t pct,
                  uint8_t* out, int cap);
```

- [ ] **Step 2: 追加失败测试（插到现有测试函数之后、main() 之前）**

```cpp
// 0x1E 姿态：roll=0.5 pitch=-0.25 yaw=1.0 rad -> int16 5000/-2500/10000 大端
void test_encode_attitude_vector() {
    uint8_t out[kCrsfMaxFrame];
    int n = encodeAttitude(0.5f, -0.25f, 1.0f, out, sizeof(out));
    TEST_ASSERT_EQUAL_INT(10, n);             // C8 08 1E +6payload +crc
    TEST_ASSERT_EQUAL_HEX8(0xC8, out[0]);
    TEST_ASSERT_EQUAL_HEX8(0x08, out[1]);     // len=type+6+crc=8
    TEST_ASSERT_EQUAL_HEX8(0x1E, out[2]);
    TEST_ASSERT_EQUAL_HEX8(0x13, out[3]);     // roll 5000 = 0x1388
    TEST_ASSERT_EQUAL_HEX8(0x88, out[4]);
    TEST_ASSERT_EQUAL_HEX8(0xF6, out[5]);     // pitch -2500 = 0xF63C
    TEST_ASSERT_EQUAL_HEX8(0x3C, out[6]);
    TEST_ASSERT_EQUAL_HEX8(0x27, out[7]);     // yaw 10000 = 0x2710
    TEST_ASSERT_EQUAL_HEX8(0x10, out[8]);
    TEST_ASSERT_EQUAL_HEX8(0x7E, out[9]);     // crc
}

// 负角符号：roll=-0.25 -> -2500 = 0xF63C（确认 int16 符号摆放）
void test_encode_attitude_negative_sign() {
    uint8_t out[kCrsfMaxFrame];
    encodeAttitude(-0.25f, 0.0f, 0.0f, out, sizeof(out));
    TEST_ASSERT_EQUAL_HEX8(0xF6, out[3]);
    TEST_ASSERT_EQUAL_HEX8(0x3C, out[4]);
}

// 0x21 飞行模式 "ANGL" -> payload "ANGL\0"
void test_encode_flight_mode() {
    uint8_t out[kCrsfMaxFrame];
    int n = encodeFlightMode("ANGL", out, sizeof(out));
    TEST_ASSERT_EQUAL_INT(9, n);              // C8 07 21 +5("ANGL\0") +crc
    TEST_ASSERT_EQUAL_HEX8(0x07, out[1]);     // len=type+5+crc=7
    TEST_ASSERT_EQUAL_HEX8(0x21, out[2]);
    TEST_ASSERT_EQUAL_HEX8(0x41, out[3]);     // 'A'
    TEST_ASSERT_EQUAL_HEX8(0x4C, out[6]);     // 'L'
    TEST_ASSERT_EQUAL_HEX8(0x00, out[7]);     // NUL
    TEST_ASSERT_EQUAL_HEX8(0xA7, out[8]);     // crc
}

// 容量守卫：太小返回 0
void test_encode_cap_guard() {
    uint8_t out[5];
    TEST_ASSERT_EQUAL_INT(0, encodeAttitude(0,0,0, out, sizeof(out)));
}
```

并把 `main()` 替换为含全部 8 个测试的版本：

```cpp
int main() {
    UNITY_BEGIN();
    RUN_TEST(test_crc8_known_vectors);
    RUN_TEST(test_build_frame_layout);
    RUN_TEST(test_build_frame_no_payload);
    RUN_TEST(test_build_frame_cap_guard);
    RUN_TEST(test_encode_attitude_vector);
    RUN_TEST(test_encode_attitude_negative_sign);
    RUN_TEST(test_encode_flight_mode);
    RUN_TEST(test_encode_cap_guard);
    return UNITY_END();
}
```
（删掉 Task 2 里那份只含前 4 个的 main()，避免重复定义。）

- [ ] **Step 3: 跑测试确认失败**

Run: `cmake --build build 2>&1 | tail -8`
Expected: 链接失败（encodeAttitude 等未定义）。

- [ ] **Step 4: 实现追加（crsf_telem.cpp，buildFrame 之后、`}  // namespace wp` 之前）**

```cpp
static void put_i16_be(uint8_t* p, int16_t v) {
    p[0] = static_cast<uint8_t>((v >> 8) & 0xFF);
    p[1] = static_cast<uint8_t>(v & 0xFF);
}

static int16_t rad_to_i16(float rad) {
    float scaled = rad * 10000.0f;
    if (scaled >  32767.0f) scaled =  32767.0f;   // 饱和防溢出
    if (scaled < -32768.0f) scaled = -32768.0f;
    return static_cast<int16_t>(scaled > 0 ? scaled + 0.5f : scaled - 0.5f);
}

int encodeAttitude(float roll_rad, float pitch_rad, float yaw_rad,
                   uint8_t* out, int cap) {
    uint8_t payload[6];
    put_i16_be(payload + 0, rad_to_i16(roll_rad));
    put_i16_be(payload + 2, rad_to_i16(pitch_rad));
    put_i16_be(payload + 4, rad_to_i16(yaw_rad));
    return buildFrame(kCrsfSyncAddr, 0x1E, payload, 6, out, cap);
}

int encodeFlightMode(const char* mode, uint8_t* out, int cap) {
    uint8_t payload[16];
    int n = 0;
    while (mode && mode[n] != '\0' && n < 15) { payload[n] = static_cast<uint8_t>(mode[n]); ++n; }
    payload[n++] = 0x00;   // NUL 结尾
    return buildFrame(kCrsfSyncAddr, 0x21, payload, n, out, cap);
}

int encodeBattery(float volts, float amps, uint32_t mah, uint8_t pct,
                  uint8_t* out, int cap) {
    int16_t v = static_cast<int16_t>(volts * 10.0f + 0.5f);   // 0.1V
    int16_t a = static_cast<int16_t>(amps  * 10.0f + 0.5f);   // 0.1A
    uint8_t payload[8];
    put_i16_be(payload + 0, v);
    put_i16_be(payload + 2, a);
    payload[4] = static_cast<uint8_t>((mah >> 16) & 0xFF);    // uint24 大端
    payload[5] = static_cast<uint8_t>((mah >> 8) & 0xFF);
    payload[6] = static_cast<uint8_t>(mah & 0xFF);
    payload[7] = pct;
    return buildFrame(kCrsfSyncAddr, 0x08, payload, 8, out, cap);
}
```

- [ ] **Step 5: 跑测试确认通过**

Run: `cmake --build build && ctest --test-dir build -R test_crsf_telem --output-on-failure`
Expected: 8 个测试 PASS。

- [ ] **Step 6: 全量回归**

Run: `ctest --test-dir build --output-on-failure 2>&1 | tail -5`
Expected: 全过（26 套）。

- [ ] **Step 7: Commit**

```bash
git add core/telemetry/crsf_telem.h core/telemetry/crsf_telem.cpp test/test_crsf_telem/
git commit -m "feat(telemetry): 姿态/模式/电池帧编码 + TelemSnapshot + 4 单测"
```

---

## Task 4: CrsfTelemetryTx 发送调度器（HAL，门控）

半双工调度：维护各帧"下次发送时刻"，每次 tick 到点发一个帧。门控 WP_HAS_TELEMETRY。HAL 件，pio 编译检查。

**Files:**
- Create: `hal_esp32/src/telemetry/crsf_telem_tx.h`
- Create: `hal_esp32/src/telemetry/crsf_telem_tx.cpp`

- [ ] **Step 1: 写头文件**

`hal_esp32/src/telemetry/crsf_telem_tx.h`：

```cpp
#pragma once
#include "config/capabilities.h"
#if WP_HAS_TELEMETRY
#include <Arduino.h>
#include "telemetry/crsf_telem.h"

namespace wp {

// CRSF 遥测发送调度器。半双工：每次 tick 至多发一个到点的帧，避免突发占满下行。
// 频率初值：姿态 10Hz、模式 2Hz（电池本轮不发，留接口）。
class CrsfTelemetryTx {
public:
    explicit CrsfTelemetryTx(HardwareSerial& uart) : uart_(uart) {}
    // now_ms：millis()。snap：本帧数据。到点则编码并写 uart（每次至多一帧）。
    void tick(uint32_t now_ms, const TelemSnapshot& snap);

private:
    HardwareSerial& uart_;
    uint32_t next_att_ms_ = 0;   // 姿态下次发送时刻
    uint32_t next_fm_ms_  = 0;   // 飞行模式下次发送时刻
};

}  // namespace wp
#endif  // WP_HAS_TELEMETRY
```

- [ ] **Step 2: 写实现**

`hal_esp32/src/telemetry/crsf_telem_tx.cpp`：

```cpp
#include "telemetry/crsf_telem_tx.h"
#if WP_HAS_TELEMETRY

namespace wp {

static constexpr uint32_t kAttPeriodMs = 100;   // 姿态 10Hz
static constexpr uint32_t kFmPeriodMs  = 500;   // 飞行模式 2Hz

void CrsfTelemetryTx::tick(uint32_t now_ms, const TelemSnapshot& snap) {
    uint8_t frame[kCrsfMaxFrame];
    // 半双工：每拍至多发一个帧。姿态优先（变化快），其次飞行模式。
    if ((int32_t)(now_ms - next_att_ms_) >= 0) {
        next_att_ms_ = now_ms + kAttPeriodMs;
        int n = encodeAttitude(snap.roll_rad, snap.pitch_rad, snap.yaw_rad,
                               frame, sizeof(frame));
        if (n > 0) uart_.write(frame, n);
        return;
    }
    if ((int32_t)(now_ms - next_fm_ms_) >= 0) {
        next_fm_ms_ = now_ms + kFmPeriodMs;
        int n = encodeFlightMode(snap.flight_mode, frame, sizeof(frame));
        if (n > 0) uart_.write(frame, n);
        return;
    }
}

}  // namespace wp
#endif  // WP_HAS_TELEMETRY
```

> 注：`(int32_t)(now - next) >= 0` 是 millis() 回绕安全的到点判断。两帧用 `return` 分隔确保每拍至多发一个（半双工不淹没上行）。

- [ ] **Step 3: 板载编译检查（门控关）**

Run: `pio run -e weekendpilot_s3 2>&1 | tail -5`
Expected: SUCCESS（WP_HAS_TELEMETRY=0，文件 #if 包空）。

- [ ] **Step 4: 板载编译检查（门控开）**

Run: `PLATFORMIO_BUILD_FLAGS="-DWP_HAS_TELEMETRY=1" pio run -e weekendpilot_s3 2>&1 | tail -6`
Expected: SUCCESS（CrsfTelemetryTx 编入，无未定义符号；main.cpp 此时尚未引用它，仅验证编译）。

- [ ] **Step 5: Commit**

```bash
git add hal_esp32/src/telemetry/crsf_telem_tx.h hal_esp32/src/telemetry/crsf_telem_tx.cpp
git commit -m "feat(hal): CrsfTelemetryTx 半双工遥测发送调度器（门控 WP_HAS_TELEMETRY）"
```

---

## Task 5: main.cpp 接入遥测（门控）

填 TelemSnapshot（复用 Controller getter）+ loop 末尾 tick。全程门控 WP_HAS_TELEMETRY。

**Files:**
- Modify: `hal_esp32/src/main.cpp`

先读现有 main.cpp 确认锚点：include 区末尾、全局对象区、loop() 末尾（status_line 打印之后、`delay(2)` 之前）。`g_controller.attitude()` 返回 `wp::Attitude{roll_deg,pitch_deg,yaw_deg}`（度）；`g_controller.activeMode()` 返回 `wp::FlightMode{Off=0,Angle=1,Rate=2}`。CRSF 姿态帧要 **弧度**，需 deg→rad 换算。

- [ ] **Step 1: include（在现有 telemetry/landing-gear include 区附近，门控）**

在 `#include "diag/status_line.h"` 之后加：

```cpp
#if WP_HAS_TELEMETRY
#include "telemetry/crsf_telem.h"
#include "telemetry/crsf_telem_tx.h"
#endif
```

- [ ] **Step 2: 全局对象（门控，放在 g_controller 附近）**

```cpp
#if WP_HAS_TELEMETRY
static wp::CrsfTelemetryTx g_telem(Serial1);   // 复用 CRSF UART（TX=GPIO43）
// FlightMode -> 4 字符模式串（CRSF 0x21 帧）
static const char* flightModeStr(wp::FlightMode m) {
    switch (m) {
        case wp::FlightMode::Angle: return "ANGL";
        case wp::FlightMode::Rate:  return "RATE";
        case wp::FlightMode::Off:
        default:                    return "OFF ";
    }
}
#endif
```

- [ ] **Step 3: loop() 末尾接入（在 WP_DEBUG_LOG 状态行打印块之后、`delay(2)` 之前）**

```cpp
#if WP_HAS_TELEMETRY
    {
        wp::TelemSnapshot ts;
        wp::Attitude a = g_controller.attitude();
        const float kDegToRad = 0.01745329252f;   // π/180
        ts.roll_rad  = a.roll_deg  * kDegToRad;
        ts.pitch_rad = a.pitch_deg * kDegToRad;
        ts.yaw_rad   = a.yaw_deg   * kDegToRad;
        ts.flight_mode = flightModeStr(g_controller.activeMode());
        g_telem.tick(millis(), ts);   // 半双工：内部每拍至多发一帧
    }
#endif
```

- [ ] **Step 4: 板载编译检查（门控关）**

Run: `pio run -e weekendpilot_s3 2>&1 | tail -5`
Expected: SUCCESS（遥测块 #if 包空，main 行为不变）。

- [ ] **Step 5: 板载编译检查（门控开）**

Run: `PLATFORMIO_BUILD_FLAGS="-DWP_HAS_TELEMETRY=1" pio run -e weekendpilot_s3 2>&1 | tail -6`
Expected: SUCCESS（遥测编排编入，无未定义符号）。

- [ ] **Step 6: Commit**

```bash
git add hal_esp32/src/main.cpp
git commit -m "feat(hal): main.cpp 接入 CRSF 遥测下行（姿态/模式，门控 WP_HAS_TELEMETRY）"
```

---

## Task 6: 文档与台账更新

**Files:**
- Modify: `docs/superpowers/specs/2026-06-01-crsf-telemetry-downlink-design.md`
- Modify: 记忆 `weekendpilot-dev-progress.md`（经 Write 工具）

- [ ] **Step 1: 设计文档状态改"已实现"**

把头部 `- 状态：设计待评审` 改为 `- 状态：已实现（PC 测试全绿 + 板载门控编译通过）`。

- [ ] **Step 2: 更新进度台账记忆**

在 `weekendpilot-dev-progress.md` 已完成区追加一段（仿现有风格）：CRSF 遥测下行已落地（core crc8/buildFrame/encode* + HAL 半双工调度 + main 接入，门控默认关，单测 +1 套 8 例），标注 ⚠️ 实物遥控器 telemetry 页待验、半双工频率为初值待实物调、电池帧留接口无数据源、纯下行不含上行命令（校准 spec 再做）。

- [ ] **Step 3: Commit**

```bash
git add docs/superpowers/specs/2026-06-01-crsf-telemetry-downlink-design.md
git commit -m "docs: CRSF 遥测下行设计文档标记已实现"
```

---

## 完成标准（Definition of Done）

- [ ] PC ctest 全绿：原 25 套 + 新增 test_crsf_telem（8 例）= 26 套。
- [ ] 板载 `pio run` 门控关 SUCCESS（行为同基线）+ 门控开 SUCCESS。
- [ ] core 编码经验证字节向量测试（CRC8 已知向量、姿态 rad×10000 大端含负角、模式串 NUL、cap 守卫）。
- [ ] 遥测默认门控关，符合"全写好先不使能"。

## 已知限制与待整定（⚠️ 本计划不覆盖）
- 实物遥控器 telemetry 页是否出现 Roll/Pitch/Yaw/FM 条目，只能实物观察（本轮仅 PC 编码单测 + pio 编译）。
- 半双工发送频率（姿态 10Hz/模式 2Hz）为初值，须实物调（遥控器丢帧/RC 卡顿即降频）。
- 电池帧编码已写但 main 未填数据源（无电压/电流采样），本轮不发电池帧。
- 纯下行，不含遥控器→飞控的上行命令（留给校准 spec）。
- yaw 来自 AHRS，缺磁力计时 yaw 会漂（已知传感器限制，非本子系统问题）。
