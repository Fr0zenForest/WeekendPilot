# NVS 配置 size 字段 uint8 → uint16 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把 config blob 的 size 字段从 uint8(1B,上限255) 扩成 uint16(2B,上限65535)，解除 ControllerConfig 的 255 字节序列化上限；不迁移旧 blob、不加任何配置字段。

**Architecture:** 单文件格式改动 `core/config/config_store.{h,cpp}`：blob 布局 `[magic2][version1][size2小端][payload][crc2]`，header 4→5、version 2→3、静态守卫 255→65535。serialize/deserialize 的 payload/crc 偏移已用 `kConfigHeaderBytes` 常量计算，改常量即自动跟随；只有 size 字段读写(buf[3])需手动改成 buf[3..4] 小端。旧 v2 blob 经 version 门作废、回退默认（不迁移）。

**Tech Stack:** C++17、Unity（PC 单测）、CMake+Ninja。纯 core，无硬件依赖。

**设计文档：** `docs/superpowers/specs/2026-06-01-config-size-uint16-design.md`

---

## 关键设计决策（实现前必读）

1. **只扩 size 字段宽度，不加配置字段**：`sizeof(ControllerConfig)` 本轮不变。
2. **偏移用常量**：现有 `serializeConfig`/`deserializeConfig`（config_store.cpp）的 payload memcpy 和 crc 位置全用 `kConfigHeaderBytes` 计算，改 `kConfigHeaderBytes=5` 即自动跟随。**唯一硬编码** `buf[3]` 的是 size 字段本身（cpp:35 写、:50 读），手动改成 buf[3..4] 小端 uint16。
3. **不迁移旧 blob**：升 version 2→3，旧 v2 blob 经 `buf[2]!=kConfigVersion` 判失败 → 回退默认。当前实物未存过 blob（main.cpp 未调 load/save），代价≈0。
4. **现有 9 个测试不硬编码偏移**：只碰 buf[0](magic)/buf[2](version)/buf[n-1](crc)，这些位置在新布局不变 → 现有测试应零修改通过。先跑确认。
5. **TDD**：先加新测试看失败，再改格式。

## 构建命令
```bash
cd /c/Repository/WeekendPilot
cmake --build build
ctest --test-dir build --output-on-failure
```

## 文件结构
**修改：**
- `core/config/config_store.h` — kConfigVersion 2→3、kConfigHeaderBytes 4→5、布局注释
- `core/config/config_store.cpp` — static_assert 65535、serializeConfig 写 uint16 size、deserializeConfig 读 uint16 size
- `test/test_config_store/test_config_store.cpp` — 加 3 新测试 + 注册（现有 9 个不动）

---

## Task 1: 扩 size 字段为 uint16（TDD）

**Files:**
- Modify: `core/config/config_store.h`
- Modify: `core/config/config_store.cpp`
- Modify: `test/test_config_store/test_config_store.cpp`

- [ ] **Step 1: 先写新测试（看失败）**

在 `test/test_config_store/test_config_store.cpp` 现有最后一个测试函数（`test_landing_gear_config_roundtrip`）之后、`int main()` 之前，追加：

```cpp
// header 现为 5 字节（magic2 + version1 + size2）
void test_header_is_5_bytes() {
    TEST_ASSERT_EQUAL_INT(5, wp::kConfigHeaderBytes);
}

// size 字段为 uint16 小端，记录 payload 字节数
void test_size_field_uint16_little_endian() {
    wp::ControllerConfig cfg{};
    uint8_t buf[wp::kConfigBlobSize];
    uint16_t n = wp::serializeConfig(cfg, buf, sizeof(buf));
    TEST_ASSERT_TRUE(n > 0);
    uint16_t payload = (uint16_t)sizeof(wp::ControllerConfig);
    // buf[3]=低字节, buf[4]=高字节
    TEST_ASSERT_EQUAL_UINT8((uint8_t)(payload & 0xFF), buf[3]);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)(payload >> 8),   buf[4]);
    // 整帧 = 5 header + payload + 2 crc
    TEST_ASSERT_EQUAL_UINT16(5 + payload + 2, n);
}

// 旧 v2 格式 blob（4 字节 header）被新 deserialize 拒绝（version 门先于 crc 拦截）
void test_old_version2_blob_rejected() {
    // 手工造一个旧布局：magic(2) version=2(1) size_u8(1) payload crc(2)
    wp::ControllerConfig cfg{};
    uint16_t payload = (uint16_t)sizeof(wp::ControllerConfig);
    uint8_t buf[wp::kConfigBlobSize];
    buf[0] = (uint8_t)(wp::kConfigMagic & 0xFF);
    buf[1] = (uint8_t)(wp::kConfigMagic >> 8);
    buf[2] = 2;                              // 旧 version
    buf[3] = (uint8_t)payload;               // 旧单字节 size
    // payload 内容无所谓（version 门会先拦截）
    uint16_t oldlen = 4 + payload + 2;
    wp::ControllerConfig out{};
    TEST_ASSERT_FALSE(wp::deserializeConfig(buf, oldlen, out));
}
```

在 `main()` 的 `RUN_TEST(test_landing_gear_config_roundtrip);` 之后追加：

```cpp
    RUN_TEST(test_header_is_5_bytes);
    RUN_TEST(test_size_field_uint16_little_endian);
    RUN_TEST(test_old_version2_blob_rejected);
```

- [ ] **Step 2: 跑测试确认失败**

Run: `cmake --build build && ctest --test-dir build -R test_config_store --output-on-failure 2>&1 | tail -15`
Expected: `test_header_is_5_bytes` 失败（当前 kConfigHeaderBytes=4）、`test_size_field_uint16_little_endian` 失败（当前 buf[4] 是 payload 首字节不是 size 高字节）。`test_old_version2_blob_rejected` 当前可能已通过（version 2≠2... 注意当前 version 就是 2，此测试当前会因 version 匹配而走到 crc 失败返回 false——仍 false 但原因不同；改 version=3 后变为 version 门拦截）。

- [ ] **Step 3: 改 config_store.h**

`core/config/config_store.h`：
- 第 9 行布局注释改为：`// blob 布局：[magic(2) | version(1) | size(2,小端) | payload(memcpy ControllerConfig) | crc16(2)]`
- `constexpr uint8_t kConfigVersion = 2;` → `constexpr uint8_t kConfigVersion = 3;   // was 2: size 字段 uint8->uint16`
- `constexpr int kConfigHeaderBytes = 4;` → `constexpr int kConfigHeaderBytes = 5;   // magic2 + version1 + size2`

- [ ] **Step 4: 改 config_store.cpp**

静态守卫（cpp:8-11 区域），把 255 改 65535 并更新注释：

```cpp
// size 字段是 buf[3..4]（uint16 小端）。一旦 ControllerConfig 超过 65535 字节，该字段
// 截断会导致 serialize/deserialize 静默错配 —— 届时需把 size 扩成 uint32 并升 version。
static_assert(sizeof(ControllerConfig) <= 65535,
    "ControllerConfig exceeds uint16 size field; widen size field + bump kConfigVersion");
```

`serializeConfig` 里把单字节 size 写（原 `buf[3] = static_cast<uint8_t>(payload);`）改为小端 uint16：

```cpp
    buf[3] = static_cast<uint8_t>(payload & 0xFF);   // size 低字节
    buf[4] = static_cast<uint8_t>(payload >> 8);     // size 高字节
```

（memcpy/crc 行不动——它们用 kConfigHeaderBytes，自动跟随到 5。）

`deserializeConfig` 里把单字节 size 校验（原 `if (buf[3] != static_cast<uint8_t>(payload)) return false;`）改为小端 uint16 读+校验：

```cpp
    uint16_t stored_size = static_cast<uint16_t>(buf[3]) |
                           (static_cast<uint16_t>(buf[4]) << 8);
    if (stored_size != payload) return false;
```

（version 检查 `if (buf[2] != kConfigVersion)` 行不动——现在比的是 3。crc/memcpy 用 kConfigHeaderBytes 自动跟随。）

- [ ] **Step 5: 跑测试确认通过（含现有 9 个零回归）**

Run: `cmake --build build && ctest --test-dir build -R test_config_store --output-on-failure 2>&1 | tail -15`
Expected: 全部 12 个测试 PASS（现有 9 + 新 3）。现有往返/magic/version/crc 测试因不硬编码 payload 偏移而自动适配新布局。

- [ ] **Step 6: 全量回归**

Run: `ctest --test-dir build --output-on-failure 2>&1 | tail -5`
Expected: 全过（26 套）。

- [ ] **Step 7: 板载编译检查（格式改动影响固件）**

Run: `pio run -e weekendpilot_s3 2>&1 | tail -5`
Expected: SUCCESS（main.cpp 未调 load/save，运行时零影响，仅验编译）。

- [ ] **Step 8: Commit**

```bash
git add core/config/config_store.h core/config/config_store.cpp test/test_config_store/test_config_store.cpp
git commit -m "feat(config): size 字段 uint8->uint16 解除 255 上限（header 4->5, version 2->3）+ 3 单测"
```

---

## Task 2: 文档与台账更新

**Files:**
- Modify: `docs/superpowers/specs/2026-06-01-config-size-uint16-design.md`
- Modify: 记忆 `weekendpilot-dev-progress.md`（Write 工具）

- [ ] **Step 1: 设计文档状态改"已实现"**

把 spec 头部 `- 状态：设计待评审` 改为 `- 状态：已实现（PC 测试 12/26 套全绿 + 板载编译通过）`。

- [ ] **Step 2: 更新进度台账记忆**

在 `weekendpilot-dev-progress.md` 已完成区追加一段（仿现有风格）：config size 字段 uint8→uint16，解除 ControllerConfig 255 字节序列化上限（之前 226/255 余量告急，起落架/ki/将来 LadrcConfig 都在挤）。header 4→5、version 2→3、守卫 255→65535。不迁移旧 blob（升 version 作废、回退默认；因 main.cpp 未调 load/save 实物无存量 blob 代价≈0）。本轮不加字段 sizeof 不变。从 ADRC 实验分解出的独立基础设施修复，ADRC 实验下一轮建在此之上（LadrcConfig 进 config 无顾虑）。test_config_store 9→12 例。⚠️ 将来 main.cpp 接 load/save 后再改格式才会丢真实配置，故现在做正是窗口期。

- [ ] **Step 3: Commit**

```bash
git add docs/superpowers/specs/2026-06-01-config-size-uint16-design.md
git commit -m "docs: config size uint16 设计文档标记已实现"
```

---

## 完成标准（Definition of Done）
- [ ] PC ctest 全绿：test_config_store 9→12 例，总 26 套不变。
- [ ] 现有 9 个 config 测试零修改通过（不硬编码 payload 偏移）。
- [ ] header=5、version=3、size 字段小端 uint16、守卫 65535。
- [ ] sizeof(ControllerConfig) 不变（本轮不加字段）。
- [ ] 板载 pio 编译通过。

## 已知限制（⚠️）
- 升 version 让旧 v2 blob 失效（不迁移，回退默认）；因 main.cpp 未调 load/save 当前代价≈0。
- 本轮纯格式升级，不加配置字段；LadrcConfig 等留各自 spec。
- size 上限升至 65535；POD/trivially_copyable 守卫不变。
