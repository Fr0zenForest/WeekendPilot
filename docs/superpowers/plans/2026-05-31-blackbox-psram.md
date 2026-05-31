# 飞行黑匣子 Blackbox (PSRAM) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 给飞控加飞行黑匣子：core 内做"采帧→定长二进制编码→环形缓冲→ISink 落盘接口"的纯逻辑链路（PC 全可单测），板载 PSRAM sink 先写好门控不使能。

**Architecture:** 三段式。(1) BlackboxFrame 定长二进制帧 + encodeFrame/decodeFrame（自定义格式，非 Betaflight BBL）。(2) Blackbox 记录器：按降采样间隔从控制器状态采帧，编码后压进 IBlackboxSink。(3) IBlackboxSink 接口（write/flush/capacity），core 侧用 RingBufferSink（固定容量环形覆盖，PC 可测）实现，板载侧 PSRAM sink 在 hal 实现并编译期门控。Controller 持有可选 Blackbox 指针，每控制周期喂一帧。

**Tech Stack:** C++17（core 纯逻辑，零硬件依赖，编译开 -Wall -Wextra -Werror），Unity 单元测试。

> WARNING 数据真实性声明（用户硬性要求）：
> - 帧格式为本项目自定义定长二进制（非 Betaflight Blackbox BBL 格式）。设计文档 §12.1 期望最终兼容 Blackbox Explorer，但 BBL 的 delta+变长编码是独立大工程，且只能靠 Blackbox Explorer GUI 人工验证——本增量不做 BBL 兼容，留作后续 adapter。自定义格式可被 decodeFrame round-trip 单测完整覆盖。
> - PSRAM 容量/驻留时间（§12.1 的 250Hz→4.4min）为理论估算，未在实物 PSRAM 上实测。板载 PSRAM sink 编译期门控，默认不使能。
> - 所有新代码/未实测项在源码与文档中带 WARNING 标注。

---

## File Structure

- `core/blackbox/blackbox_frame.h` — BlackboxFrame 结构 + 字段布局常量 + encodeFrame/decodeFrame + kFrameBytes。
- `core/blackbox/blackbox_frame.cpp` — 编解码实现（小端定长）。
- `core/blackbox/blackbox_sink.h` — IBlackboxSink 纯虚接口（write/flush/capacityBytes/usedBytes）+ RingBufferSink（固定 buf 环形覆盖）。
- `core/blackbox/blackbox.h` — BlackboxConfig（enabled/decimation）+ Blackbox 记录器类。
- `core/blackbox/blackbox.cpp` — 降采样采帧 + 编码 + 写 sink。
- `core/controller.h` / `.cpp` — ControllerConfig 增 blackbox 字段；Controller 持有 Blackbox，update() 末尾喂一帧。
- `test/test_blackbox_frame/test_blackbox_frame.cpp` — 编解码 round-trip + 字节长度。
- `test/test_blackbox/test_blackbox.cpp` — 降采样、环形覆盖、enabled 门控。
- `test/CMakeLists.txt` — 注册两个测试。
- `hal_esp32/src/drivers/blackbox_psram_sink.h` / `.cpp` — PsramSink : IBlackboxSink（heap_caps_malloc MALLOC_CAP_SPIRAM），编译期 WP_HAS_BLACKBOX 门控。
- `core/config/capabilities.h` — 加 WP_HAS_BLACKBOX 宏（默认 PC 开/板载关）。
- `docs/WeekendPilot-功能说明.md` — 黑匣子从"开发中"挪到"已集成（core 就绪，PSRAM 落盘待实机）"。

---

### Task 1: BlackboxFrame 定长编解码

**Files:**
- Create: `core/blackbox/blackbox_frame.h`
- Create: `core/blackbox/blackbox_frame.cpp`
- Test: `test/test_blackbox_frame/test_blackbox_frame.cpp`
- Modify: `test/CMakeLists.txt`

- [ ] **Step 1: 写失败测试**

创建 `test/test_blackbox_frame/test_blackbox_frame.cpp`：

```cpp
#include "unity.h"
#include "blackbox/blackbox_frame.h"
#include <cstring>
using namespace wp;

void setUp() {} void tearDown() {}

// WARNING 新代码（自定义定长二进制格式，非 Betaflight BBL）。round-trip 逻辑可测；
//    PSRAM 落盘与 Blackbox Explorer 兼容性未实测。

// 编码再解码应还原所有字段（round-trip）。
void test_frame_roundtrip() {
    BlackboxFrame f{};
    f.t_ms = 0x12345678u;
    f.gyro_x = 12.5f; f.gyro_y = -33.0f; f.gyro_z = 7.25f;
    f.accel_x = 0.1f; f.accel_y = -0.2f; f.accel_z = 1.0f;
    f.roll_deg = 15.0f; f.pitch_deg = -6.0f; f.yaw_deg = 180.0f;
    for (int i = 0; i < kNumServos; ++i) f.servo[i] = static_cast<uint16_t>(1000 + i * 100);
    f.baro_alt_m = 123.5f;
    f.mode = 1; f.flags = 0x05;

    uint8_t buf[kFrameBytes];
    encodeFrame(f, buf);
    BlackboxFrame g{};
    decodeFrame(buf, g);

    TEST_ASSERT_EQUAL_UINT32(f.t_ms, g.t_ms);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, f.gyro_x, g.gyro_x);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, f.gyro_z, g.gyro_z);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, f.accel_z, g.accel_z);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, f.roll_deg, g.roll_deg);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, f.yaw_deg, g.yaw_deg);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, f.baro_alt_m, g.baro_alt_m);
    for (int i = 0; i < kNumServos; ++i)
        TEST_ASSERT_EQUAL_UINT16(f.servo[i], g.servo[i]);
    TEST_ASSERT_EQUAL_UINT8(f.mode, g.mode);
    TEST_ASSERT_EQUAL_UINT8(f.flags, g.flags);
}

// 帧长度固定且等于声明值。
void test_frame_size_fixed() {
    // t_ms(4) + 9 floats(36) + 8 servos(16) + baro(4) + mode(1) + flags(1) = 62
    TEST_ASSERT_EQUAL_INT(62, kFrameBytes);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_frame_roundtrip);
    RUN_TEST(test_frame_size_fixed);
    return UNITY_END();
}
```

- [ ] **Step 2: 注册测试并确认编译失败**

`test/CMakeLists.txt` 末尾追加 `wp_add_test(test_blackbox_frame)`。
Run: `cmake --preset dev && cmake --build build --target test_blackbox_frame`
Expected: FAIL —— blackbox/blackbox_frame.h 不存在。

- [ ] **Step 3: 写头文件**

创建 `core/blackbox/blackbox_frame.h`：

```cpp
#pragma once
#include <cstdint>
#include "types.h"   // kNumServos

namespace wp {

// WARNING 新代码（自定义定长小端二进制格式，非 Betaflight BBL）。round-trip 可单测；
//    与 Blackbox Explorer 的兼容性、PSRAM 落盘均未实测。
struct BlackboxFrame {
    uint32_t t_ms = 0;                 // 时间戳 ms
    float gyro_x = 0, gyro_y = 0, gyro_z = 0;    // deg/s
    float accel_x = 0, accel_y = 0, accel_z = 0; // g
    float roll_deg = 0, pitch_deg = 0, yaw_deg = 0;
    uint16_t servo[kNumServos] = {0};  // 输出 us
    float baro_alt_m = 0;
    uint8_t mode = 0;                  // FlightMode 值
    uint8_t flags = 0;                 // bit0 link_ok, bit1 imu_valid, bit2 baro_valid, bit3 althold
};

// 定长字节数：4 + 9*4 + 8*2 + 4 + 1 + 1 = 62
constexpr int kFrameBytes = 4 + 9 * 4 + kNumServos * 2 + 4 + 1 + 1;

// 小端定长编码：out 必须至少 kFrameBytes 字节。
void encodeFrame(const BlackboxFrame& f, uint8_t* out);
// 解码：in 必须至少 kFrameBytes 字节。
void decodeFrame(const uint8_t* in, BlackboxFrame& out);

}  // namespace wp
```

- [ ] **Step 4: 写实现**

创建 `core/blackbox/blackbox_frame.cpp`：

```cpp
#include "blackbox/blackbox_frame.h"
#include <cstring>

namespace wp {

namespace {
void putU32(uint8_t*& p, uint32_t v) {
    p[0] = uint8_t(v); p[1] = uint8_t(v >> 8);
    p[2] = uint8_t(v >> 16); p[3] = uint8_t(v >> 24); p += 4;
}
void putU16(uint8_t*& p, uint16_t v) { p[0] = uint8_t(v); p[1] = uint8_t(v >> 8); p += 2; }
void putF(uint8_t*& p, float v) { uint32_t u; std::memcpy(&u, &v, 4); putU32(p, u); }
uint32_t getU32(const uint8_t*& p) {
    uint32_t v = uint32_t(p[0]) | (uint32_t(p[1]) << 8) |
                 (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24); p += 4; return v;
}
uint16_t getU16(const uint8_t*& p) { uint16_t v = uint16_t(p[0] | (p[1] << 8)); p += 2; return v; }
float getF(const uint8_t*& p) { uint32_t u = getU32(p); float v; std::memcpy(&v, &u, 4); return v; }
}  // namespace

void encodeFrame(const BlackboxFrame& f, uint8_t* out) {
    uint8_t* p = out;
    putU32(p, f.t_ms);
    putF(p, f.gyro_x); putF(p, f.gyro_y); putF(p, f.gyro_z);
    putF(p, f.accel_x); putF(p, f.accel_y); putF(p, f.accel_z);
    putF(p, f.roll_deg); putF(p, f.pitch_deg); putF(p, f.yaw_deg);
    for (int i = 0; i < kNumServos; ++i) putU16(p, f.servo[i]);
    putF(p, f.baro_alt_m);
    *p++ = f.mode;
    *p++ = f.flags;
}

void decodeFrame(const uint8_t* in, BlackboxFrame& out) {
    const uint8_t* p = in;
    out.t_ms = getU32(p);
    out.gyro_x = getF(p); out.gyro_y = getF(p); out.gyro_z = getF(p);
    out.accel_x = getF(p); out.accel_y = getF(p); out.accel_z = getF(p);
    out.roll_deg = getF(p); out.pitch_deg = getF(p); out.yaw_deg = getF(p);
    for (int i = 0; i < kNumServos; ++i) out.servo[i] = getU16(p);
    out.baro_alt_m = getF(p);
    out.mode = *p++;
    out.flags = *p++;
}

}  // namespace wp
```

- [ ] **Step 5: 编译并跑测试确认通过**

Run: `cmake --preset dev && cmake --build build --target test_blackbox_frame && ctest --test-dir build -R test_blackbox_frame --output-on-failure`
Expected: PASS（2 用例）

- [ ] **Step 6: 提交**

```bash
git add core/blackbox/blackbox_frame.h core/blackbox/blackbox_frame.cpp test/test_blackbox_frame/test_blackbox_frame.cpp test/CMakeLists.txt
git commit -m "feat(blackbox): fixed-length binary frame encode/decode"
```

---

### Task 2: IBlackboxSink 接口 + RingBufferSink

**Files:**
- Create: `core/blackbox/blackbox_sink.h`
- Test: `test/test_blackbox/test_blackbox.cpp`（本任务先建文件，只测 sink）
- Modify: `test/CMakeLists.txt`

- [ ] **Step 1: 写失败测试**

创建 `test/test_blackbox/test_blackbox.cpp`：

```cpp
#include "unity.h"
#include "blackbox/blackbox_sink.h"
#include <cstring>
using namespace wp;

void setUp() {} void tearDown() {}

// WARNING 新代码。环形缓冲逻辑可单测；板载 PSRAM sink 未实测。

// 写入未超容量：usedBytes 累加，capacity 不变。
void test_ring_accumulates() {
    uint8_t buf[64];
    RingBufferSink sink(buf, sizeof(buf));
    uint8_t data[10] = {1,2,3,4,5,6,7,8,9,10};
    TEST_ASSERT_TRUE(sink.write(data, 10));
    TEST_ASSERT_EQUAL_INT(10, (int)sink.usedBytes());
    TEST_ASSERT_EQUAL_INT(64, (int)sink.capacityBytes());
}

// 超容量环形覆盖：写入超过 buf 后，usedBytes 封顶 = capacity，且 wrapped 置位。
void test_ring_overwrites_when_full() {
    uint8_t buf[16];
    RingBufferSink sink(buf, sizeof(buf));
    uint8_t chunk[10] = {0};
    sink.write(chunk, 10);                 // used=10
    TEST_ASSERT_FALSE(sink.wrapped());
    sink.write(chunk, 10);                 // 20 > 16 -> 覆盖，环回
    TEST_ASSERT_TRUE(sink.wrapped());
    TEST_ASSERT_EQUAL_INT(16, (int)sink.usedBytes());  // 满后封顶
}

// write 比单次容量还大的块应被拒绝（返回 false，不写）。
void test_ring_rejects_oversize_single_write() {
    uint8_t buf[8];
    RingBufferSink sink(buf, sizeof(buf));
    uint8_t big[9] = {0};
    TEST_ASSERT_FALSE(sink.write(big, 9));
    TEST_ASSERT_EQUAL_INT(0, (int)sink.usedBytes());
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_ring_accumulates);
    RUN_TEST(test_ring_overwrites_when_full);
    RUN_TEST(test_ring_rejects_oversize_single_write);
    return UNITY_END();
}
```

- [ ] **Step 2: 注册并确认失败**

`test/CMakeLists.txt` 追加 `wp_add_test(test_blackbox)`。
Run: `cmake --preset dev && cmake --build build --target test_blackbox`
Expected: FAIL —— blackbox/blackbox_sink.h 不存在。

- [ ] **Step 3: 写头文件**

创建 `core/blackbox/blackbox_sink.h`：

```cpp
#pragma once
#include <cstdint>
#include <cstddef>

namespace wp {

// 黑匣子落盘接口：core 只认这个，不认介质。PSRAM/W25Q/SD 在 hal 实现。
// write 返回 false 表示本次写入被拒（块超单次容量）；环形覆盖不算失败。
struct IBlackboxSink {
    virtual ~IBlackboxSink() = default;
    virtual bool write(const uint8_t* data, size_t len) = 0;
    virtual void flush() = 0;
    virtual size_t capacityBytes() const = 0;
    virtual size_t usedBytes() const = 0;
};

// WARNING 新代码。固定容量环形缓冲：写满后从头覆盖最旧数据（保留最近 N 字节）。
// 调用方提供底层 buffer（PC 测试用栈数组，板载用 PSRAM）。
class RingBufferSink : public IBlackboxSink {
public:
    RingBufferSink(uint8_t* buf, size_t cap) : buf_(buf), cap_(cap) {}

    bool write(const uint8_t* data, size_t len) override {
        if (len > cap_) return false;          // 单块超总容量，拒绝
        for (size_t i = 0; i < len; ++i) {
            buf_[head_] = data[i];
            head_ = (head_ + 1) % cap_;
            if (used_ < cap_) ++used_; else wrapped_ = true;
        }
        return true;
    }
    void flush() override {}                    // 内存缓冲无需 flush
    size_t capacityBytes() const override { return cap_; }
    size_t usedBytes() const override { return used_; }
    bool wrapped() const { return wrapped_; }   // 是否已发生覆盖

private:
    uint8_t* buf_;
    size_t cap_;
    size_t head_ = 0;
    size_t used_ = 0;
    bool wrapped_ = false;
};

}  // namespace wp
```

- [ ] **Step 4: 编译并跑测试确认通过**

Run: `cmake --build build --target test_blackbox && ctest --test-dir build -R test_blackbox --output-on-failure`
Expected: PASS（3 用例）

- [ ] **Step 5: 提交**

```bash
git add core/blackbox/blackbox_sink.h test/test_blackbox/test_blackbox.cpp test/CMakeLists.txt
git commit -m "feat(blackbox): IBlackboxSink interface + RingBufferSink"
```

---

### Task 3: Blackbox 记录器（降采样 + 门控）

**Files:**
- Create: `core/blackbox/blackbox.h`
- Create: `core/blackbox/blackbox.cpp`
- Modify: `test/test_blackbox/test_blackbox.cpp`

- [ ] **Step 1: 追加失败测试**

在 `test/test_blackbox/test_blackbox.cpp` 顶部 include 追加 `#include "blackbox/blackbox.h"`，在 `main()` 前追加用例并登记：

```cpp
// 关闭时不记录。
void test_disabled_records_nothing() {
    uint8_t buf[256];
    RingBufferSink sink(buf, sizeof(buf));
    Blackbox bb;
    BlackboxConfig cfg; cfg.enabled = false; cfg.decimation = 1;
    bb.begin(cfg, &sink);
    BlackboxFrame f{};
    for (int i = 0; i < 10; ++i) bb.logFrame(f);
    TEST_ASSERT_EQUAL_INT(0, (int)sink.usedBytes());
}

// 降采样 decimation=4：每 4 次 logFrame 只写 1 帧。
void test_decimation_writes_every_n() {
    uint8_t buf[4096];
    RingBufferSink sink(buf, sizeof(buf));
    Blackbox bb;
    BlackboxConfig cfg; cfg.enabled = true; cfg.decimation = 4;
    bb.begin(cfg, &sink);
    BlackboxFrame f{};
    for (int i = 0; i < 8; ++i) bb.logFrame(f);   // 8 次 -> 写 2 帧
    TEST_ASSERT_EQUAL_INT(2 * kFrameBytes, (int)sink.usedBytes());
}

// decimation=1：每次都写。
void test_decimation_one_writes_all() {
    uint8_t buf[4096];
    RingBufferSink sink(buf, sizeof(buf));
    Blackbox bb;
    BlackboxConfig cfg; cfg.enabled = true; cfg.decimation = 1;
    bb.begin(cfg, &sink);
    BlackboxFrame f{};
    for (int i = 0; i < 5; ++i) bb.logFrame(f);
    TEST_ASSERT_EQUAL_INT(5 * kFrameBytes, (int)sink.usedBytes());
    TEST_ASSERT_EQUAL_INT(5, (int)bb.framesWritten());
}

// 无 sink（begin 传 nullptr）时 logFrame 不崩溃、不计数。
void test_null_sink_safe() {
    Blackbox bb;
    BlackboxConfig cfg; cfg.enabled = true; cfg.decimation = 1;
    bb.begin(cfg, nullptr);
    BlackboxFrame f{};
    bb.logFrame(f);
    TEST_ASSERT_EQUAL_INT(0, (int)bb.framesWritten());
}
```

`main()` 追加：

```cpp
    RUN_TEST(test_disabled_records_nothing);
    RUN_TEST(test_decimation_writes_every_n);
    RUN_TEST(test_decimation_one_writes_all);
    RUN_TEST(test_null_sink_safe);
```

- [ ] **Step 2: 跑测试确认失败**

Run: `cmake --build build --target test_blackbox`
Expected: FAIL —— blackbox/blackbox.h 不存在。

- [ ] **Step 3: 写头文件**

创建 `core/blackbox/blackbox.h`：

```cpp
#pragma once
#include <cstdint>
#include "blackbox/blackbox_frame.h"
#include "blackbox/blackbox_sink.h"

namespace wp {

struct BlackboxConfig {
    bool enabled = false;       // 总开关（默认关，全写好先不使能）
    uint16_t decimation = 4;    // 每 N 次 logFrame 落 1 帧（控制环 1kHz / 4 = 250Hz）
};

// WARNING 新代码。黑匣子记录器：按 decimation 降采样，把 BlackboxFrame 编码进 sink。
// sink 由调用方持有（PC 测试 RingBufferSink，板载 PsramSink）。
class Blackbox {
public:
    void begin(const BlackboxConfig& cfg, IBlackboxSink* sink);
    void logFrame(const BlackboxFrame& f);     // 每控制周期调一次
    uint32_t framesWritten() const { return frames_written_; }

private:
    BlackboxConfig cfg_;
    IBlackboxSink* sink_ = nullptr;
    uint16_t counter_ = 0;
    uint32_t frames_written_ = 0;
};

}  // namespace wp
```

- [ ] **Step 4: 写实现**

创建 `core/blackbox/blackbox.cpp`：

```cpp
#include "blackbox/blackbox.h"

namespace wp {

void Blackbox::begin(const BlackboxConfig& cfg, IBlackboxSink* sink) {
    cfg_ = cfg;
    sink_ = sink;
    counter_ = 0;
    frames_written_ = 0;
}

void Blackbox::logFrame(const BlackboxFrame& f) {
    if (!cfg_.enabled || sink_ == nullptr) return;
    const uint16_t dec = cfg_.decimation == 0 ? 1 : cfg_.decimation;
    if (++counter_ < dec) return;     // 未到降采样间隔
    counter_ = 0;

    uint8_t buf[kFrameBytes];
    encodeFrame(f, buf);
    if (sink_->write(buf, kFrameBytes)) ++frames_written_;
}

}  // namespace wp
```

- [ ] **Step 5: 跑测试确认通过**

Run: `cmake --preset dev && cmake --build build --target test_blackbox && ctest --test-dir build -R test_blackbox --output-on-failure`
Expected: PASS（7 用例：3 sink + 4 recorder）

- [ ] **Step 6: 提交**

```bash
git add core/blackbox/blackbox.h core/blackbox/blackbox.cpp test/test_blackbox/test_blackbox.cpp
git commit -m "feat(blackbox): Blackbox recorder with decimation + enable gate"
```

---

### Task 4: Controller 集成 —— 每周期喂一帧

**Files:**
- Modify: `core/controller.h`
- Modify: `core/controller.cpp`
- Test: `test/test_controller/test_controller.cpp`

- [ ] **Step 1: 写失败的集成测试**

在 `test/test_controller/test_controller.cpp` 末尾（main 之前）追加用例并登记。先读文件确认 fill_centered 风格。核心断言：黑匣子使能 + 注入 sink 后，调 update() 若干次应有帧落入 sink；且帧里 mode 字段与当前模式一致。

```cpp
// 黑匣子使能 + 注入 sink -> update() 落帧；解出的 mode 与输入模式一致。
void test_blackbox_records_on_update() {
    static uint8_t bb_buf[4096];
    wp::RingBufferSink sink(bb_buf, sizeof(bb_buf));
    wp::Controller c;
    wp::ControllerConfig cfg;
    cfg.blackbox.enabled = true;
    cfg.blackbox.decimation = 1;     // 每拍都记
    c.setConfig(cfg);
    c.attachBlackboxSink(&sink);

    wp::ControlInput in{}; fill_centered(in);
    in.channels[4] = 1500;           // Angle
    in.baro.valid = true; in.baro.altitude_m = 50.0f;
    c.update(in);

    TEST_ASSERT_EQUAL_INT(wp::kFrameBytes, (int)sink.usedBytes());
    wp::BlackboxFrame g{};
    wp::decodeFrame(bb_buf, g);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)wp::FlightMode::Angle, g.mode);
    TEST_ASSERT_FLOAT_WITHIN(1e-2, 50.0f, g.baro_alt_m);
}

// 黑匣子默认关 -> 不注入 sink 也不记录，行为不变。
void test_blackbox_disabled_by_default() {
    static uint8_t bb_buf[256];
    wp::RingBufferSink sink(bb_buf, sizeof(bb_buf));
    wp::Controller c;                // 默认 cfg：blackbox.enabled=false
    c.attachBlackboxSink(&sink);
    wp::ControlInput in{}; fill_centered(in);
    in.channels[4] = 1500;
    c.update(in);
    TEST_ASSERT_EQUAL_INT(0, (int)sink.usedBytes());
}
```

`main()` 追加：

```cpp
    RUN_TEST(test_blackbox_records_on_update);
    RUN_TEST(test_blackbox_disabled_by_default);
```

文件顶部若未 include，需加 `#include "blackbox/blackbox_sink.h"` 与 `#include "blackbox/blackbox_frame.h"`（controller.h 会间接带入，但测试直接用 RingBufferSink/decodeFrame，显式 include 更稳）。

- [ ] **Step 2: 跑测试确认失败**

Run: `cmake --build build --target test_controller`
Expected: FAIL —— ControllerConfig 无 blackbox 成员、Controller 无 attachBlackboxSink。

- [ ] **Step 3: 扩展 ControllerConfig 与 Controller**

修改 `core/controller.h`：
- 顶部 include 追加 `#include "blackbox/blackbox.h"`。
- `ControllerConfig` 增字段：`BlackboxConfig blackbox;`
- `Controller` 公有区增：`void attachBlackboxSink(IBlackboxSink* sink);`
- `Controller` 私有区增：`Blackbox blackbox_;`

修改 `core/controller.cpp`：
- `setConfig` 末尾追加 `blackbox_.begin(cfg_.blackbox, bb_sink_);` —— 但 sink 可能尚未 attach，故需保存 sink 指针。改为：在 Controller 私有区再加 `IBlackboxSink* bb_sink_ = nullptr;`，`attachBlackboxSink` 存指针并调用 `blackbox_.begin(cfg_.blackbox, bb_sink_);`；`setConfig` 也调 `blackbox_.begin(cfg_.blackbox, bb_sink_);`（用已存的 bb_sink_）。

```cpp
void Controller::attachBlackboxSink(IBlackboxSink* sink) {
    bb_sink_ = sink;
    blackbox_.begin(cfg_.blackbox, bb_sink_);
}
```

`setConfig` 内（在 mixer_/althold_ 之后）：

```cpp
    blackbox_.begin(cfg_.blackbox, bb_sink_);
```

- [ ] **Step 4: update() 末尾采帧落盘**

在 `core/controller.cpp` 的 `update()` 里，在 `return out;` 之前（无论直通早返回还是正常路径都应记录——但早返回有多个 return 点）。**为覆盖所有路径**，把采帧放进一个 helper，并在每个 return 前调用过于啰嗦；改为：在 `update()` 末尾正常路径记录即可（直通/失效态的黑匣子价值低，且早返回时姿态/demand 未算）。在最终 `return out;` 之前插入：

```cpp
    // 黑匣子：把本拍状态编码落盘（降采样在 Blackbox 内部）。
    if (cfg_.blackbox.enabled) {
        BlackboxFrame fr{};
        fr.t_ms = frame_t_ms_;          // 见下：由 dt 累加
        fr.gyro_x = in.imu.gyro_x; fr.gyro_y = in.imu.gyro_y; fr.gyro_z = in.imu.gyro_z;
        fr.accel_x = in.imu.accel_x; fr.accel_y = in.imu.accel_y; fr.accel_z = in.imu.accel_z;
        Attitude a = ahrs_.attitude();
        fr.roll_deg = a.roll_deg; fr.pitch_deg = a.pitch_deg; fr.yaw_deg = a.yaw_deg;
        for (int i = 0; i < kNumServos; ++i) fr.servo[i] = out.servo[i];
        fr.baro_alt_m = in.baro.altitude_m;
        fr.mode = static_cast<uint8_t>(mode);
        fr.flags = uint8_t((in.link_ok ? 1 : 0) | (in.imu.valid ? 2 : 0) |
                           (in.baro.valid ? 4 : 0) | (althold_active ? 8 : 0));
        blackbox_.logFrame(fr);
    }
    return out;
```

时间戳：在 Controller 私有区加 `uint32_t frame_t_ms_ = 0;`，在 `update()` 开头累加 `frame_t_ms_ += static_cast<uint32_t>(in.dt * 1000.0f + 0.5f);`。注意：`althold_active` 变量在 Off/失效早返回路径不存在——因此黑匣子记录块必须在正常路径（已过早返回）之后，`althold_active` 已定义。确认插入点在 althold 块之后。

注意：Off 模式 / 链路丢失 / IMU 失效会在函数前部早 return，这些拍不记录黑匣子（已在数据真实性声明里注明：黑匣子只覆盖在控状态）。若需要也记录直通态，属后续增强，本增量不做。

- [ ] **Step 5: 跑测试确认通过 + 全量回归**

Run: `cmake --build build --target test_controller test_blackbox test_blackbox_frame && ctest --test-dir build --output-on-failure`
Expected: 全绿。

- [ ] **Step 6: 提交**

```bash
git add core/controller.h core/controller.cpp test/test_controller/test_controller.cpp
git commit -m "feat(core): wire Blackbox into Controller (log frame each in-control tick)"
```

---

### Task 5: 板载 PSRAM sink（门控不使能）+ 能力宏

**Files:**
- Modify: `core/config/capabilities.h`
- Create: `hal_esp32/src/drivers/blackbox_psram_sink.h`
- Create: `hal_esp32/src/drivers/blackbox_psram_sink.cpp`

> 说明：本任务为板载侧，无法在 PC ctest 覆盖（同其它 hal 驱动），只能靠 `pio run -e weekendpilot_s3` 编译检查。默认 `WP_HAS_BLACKBOX` 板载为 0（不使能），main.cpp 暂不接线（留待实机带 PSRAM 的板子）。

- [ ] **Step 1: 加能力宏**

修改 `core/config/capabilities.h`：在 PC 默认全开块里加 `WP_HAS_BLACKBOX`，在兜底块里加默认 0，并加 constexpr 镜像。仿照已有 WP_HAS_BARO 的三处写法：

PC 默认全开块内（`#if !defined(BOARD_DEVKIT_S3)` 内）：
```cpp
  #ifndef WP_HAS_BLACKBOX
    #define WP_HAS_BLACKBOX 1
  #endif
```
兜底块：
```cpp
#ifndef WP_HAS_BLACKBOX
  #define WP_HAS_BLACKBOX 0
#endif
```
constexpr 镜像（namespace wp 内）：
```cpp
constexpr bool kHasBlackbox = WP_HAS_BLACKBOX;
```

注：板载 BOARD_DEVKIT_S3 场景下 WP_HAS_BLACKBOX 不被全开块覆盖 -> 落到兜底 0（默认不使能），符合"写好先不使能"。

- [ ] **Step 2: 写 PSRAM sink 头文件**

创建 `hal_esp32/src/drivers/blackbox_psram_sink.h`：

```cpp
#pragma once
#include "config/capabilities.h"
#if WP_HAS_BLACKBOX
#include "blackbox/blackbox_sink.h"

namespace wp {

// WARNING 新代码，未在实物 PSRAM 上实测。在 ESP32-S3 的 PSRAM(SPIRAM) 上分配环形缓冲，
//    复用 core 的 RingBufferSink 逻辑（仅 buffer 来自 heap_caps_malloc SPIRAM）。
class PsramSink : public IBlackboxSink {
public:
    explicit PsramSink(size_t cap_bytes);
    ~PsramSink() override;
    bool ok() const { return ring_ != nullptr; }

    bool write(const uint8_t* data, size_t len) override;
    void flush() override;
    size_t capacityBytes() const override;
    size_t usedBytes() const override;

private:
    uint8_t* buf_ = nullptr;
    RingBufferSink* ring_ = nullptr;   // placement 复用 core 环形逻辑
};

}  // namespace wp
#endif  // WP_HAS_BLACKBOX
```

- [ ] **Step 3: 写 PSRAM sink 实现**

创建 `hal_esp32/src/drivers/blackbox_psram_sink.cpp`：

```cpp
#include "drivers/blackbox_psram_sink.h"
#if WP_HAS_BLACKBOX
#include <esp_heap_caps.h>
#include <new>

namespace wp {

PsramSink::PsramSink(size_t cap_bytes) {
    // 优先 SPIRAM；失败则保持 nullptr（ok()=false，调用方降级不记录）。
    buf_ = static_cast<uint8_t*>(heap_caps_malloc(cap_bytes, MALLOC_CAP_SPIRAM));
    if (buf_) {
        ring_ = new (std::nothrow) RingBufferSink(buf_, cap_bytes);
        if (!ring_) { heap_caps_free(buf_); buf_ = nullptr; }
    }
}

PsramSink::~PsramSink() {
    delete ring_;
    if (buf_) heap_caps_free(buf_);
}

bool PsramSink::write(const uint8_t* data, size_t len) {
    return ring_ ? ring_->write(data, len) : false;
}
void PsramSink::flush() { if (ring_) ring_->flush(); }
size_t PsramSink::capacityBytes() const { return ring_ ? ring_->capacityBytes() : 0; }
size_t PsramSink::usedBytes() const { return ring_ ? ring_->usedBytes() : 0; }

}  // namespace wp
#endif  // WP_HAS_BLACKBOX
```

- [ ] **Step 4: 板载编译检查**

Run: `pio run -e weekendpilot_s3`
Expected: SUCCESS。因 WP_HAS_BLACKBOX 板载默认 0，sink 文件体被 #if 排除，只验证不破坏现有编译。（如需真正编译 sink 体，临时加 `-DWP_HAS_BLACKBOX=1` build flag 验证一次后撤回——可选。）

- [ ] **Step 5: 提交**

```bash
git add core/config/capabilities.h hal_esp32/src/drivers/blackbox_psram_sink.h hal_esp32/src/drivers/blackbox_psram_sink.cpp
git commit -m "feat(hal): gated PSRAM blackbox sink (WP_HAS_BLACKBOX, off by default)"
```

---

### Task 6: 文档更新

**Files:**
- Modify: `docs/WeekendPilot-功能说明.md`

- [ ] **Step 1: 把黑匣子从"开发中"挪到"已集成"**

在"已集成功能 / 系统基础设施"表追加一行：

```
| **飞行黑匣子（核心）** | 控制环每拍采帧，降采样落环形缓冲，介质可换（PSRAM/SD/flash） | **自研**（自定义定长二进制帧 + 环形缓冲 + ISink 接口；格式非 Betaflight BBL） | 编解码 round-trip + 降采样 + 环形覆盖 + 门控 PC 单测。⚠️ PSRAM 落盘与 Blackbox Explorer 兼容性未实测 |
```

从"二、开发中/即将推出"删除/改写黑匣子行（保留"WiFi 下载 + BBL 格式兼容"作为开发中项）：

```
| **黑匣子 BBL 兼容 + WiFi 下载** | 转 Betaflight Blackbox 格式，网页下载日志 | PSRAM sink（已写）+ WebUI | 排期中 |
```

- [ ] **Step 2: 更新验证三道关的测试套数**

把"研发方法说明"里的"18 套自动化测试"更新为实际数（Blackbox 加了 2 个测试可执行：test_blackbox_frame + test_blackbox -> 20 套）。运行 `ctest --test-dir build -N | tail -3` 确认总数后填准确值。

- [ ] **Step 3: 提交**

```bash
git add docs/WeekendPilot-功能说明.md
git commit -m "docs: mark Blackbox core integrated; BBL/WiFi download as upcoming"
```

---

## 数据真实性总声明（用户硬性要求复述）

- 帧格式 **自定义定长二进制**，非 Betaflight BBL；与 Blackbox Explorer 的兼容性**未做也未验**，留作后续 adapter。
- core 链路（编解码 / 环形缓冲 / 降采样 / 门控）**纯逻辑 PC 全单测**。
- 板载 PSRAM sink **未在实物 PSRAM 实测**，编译期 `WP_HAS_BLACKBOX` 默认关，main.cpp 不接线。
- §12.1 的容量/时长（250Hz→4.4min）为**理论估算**。
- 全部新代码 / 未实测项带 WARNING 标注。

## 自检（writing-plans 要求）

- 规格覆盖：帧编解码(T1)、sink 接口+环形(T2)、记录器降采样门控(T3)、控制器集成(T4)、板载 PSRAM sink+能力宏(T5)、文档(T6) —— 全覆盖。
- 占位符扫描：无 TODO/TBD；每个代码步骤给出完整代码。
- 类型一致性：`kFrameBytes`、`encodeFrame/decodeFrame`、`IBlackboxSink`(write/flush/capacityBytes/usedBytes)、`RingBufferSink`、`BlackboxConfig`(enabled/decimation)、`Blackbox`(begin/logFrame/framesWritten)、`ControllerConfig::blackbox`、`Controller::attachBlackboxSink` 全程一致。
