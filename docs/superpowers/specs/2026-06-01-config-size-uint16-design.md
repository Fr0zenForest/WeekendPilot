# NVS 配置 size 字段 uint8 → uint16（解除 255 字节上限）— 设计文档

- 日期：2026-06-01
- 状态：已实现（PC 测试 config_store 12 例 + 26 套全绿；板载编译通过；现有 9 个 config 测试零回归）
- 关联：[[weekendpilot-dev-progress]]、[[weekendpilot-config-sensor-architecture]]
- 定位：配置持久化基础设施修复。从 ADRC 实验(A)讨论中分解出来——A 要把 LadrcConfig 加进 ControllerConfig 会撞 255 余量(现 228/255 仅余 27B，起落架/ki/LadrcConfig 都在挤)。本 spec 独立先做，解除全局配置容量焦虑；ADRC 实验另起一轮，建在此之上。

## 0. 背景与动机

config blob 序列化布局（`core/config/config_store.cpp`）：
```
[magic 2B][version 1B][size 1B][payload = memcpy(ControllerConfig)][crc16 2B]
                        ↑ buf[3]，uint8，上限 255
```
`size` 字段（`buf[3]`）记录 payload 字节数，反序列化时校验长度。它是 **uint8（1 字节，上限 255）**。`config_store.cpp:8-11` 已有 `static_assert(sizeof(ControllerConfig) <= 255)` 作编译期保险丝，并注释写明解法："超则把 size 扩成 uint16 并升 version"。

当前 `sizeof(ControllerConfig)=228`，余量仅 27B。多个在途/已落地功能（起落架 LandingGearConfig、Mahony ki 预留、ADRC LadrcConfig）都在挤这 27B。255 是**自定义序列化格式的字段宽度限制，与 NVS flash 容量无关**（NVS 分区有 20KB+，配置才占 ~0.2KB）。本 spec 把 size 扩成 uint16（上限→65535），一次性拆掉这个迟早要拆的雷。

**为何现在做代价几乎为零**：板载 main.cpp 目前**尚未调用 ConfigStore load/save**（[[weekendpilot-dev-progress]] 记录的遗留项②），即实物从未真正存过配置 blob。此时升 version 让"旧 blob 失效"几乎无实际损失——没有旧 blob 存在。错过这个窗口，等实物开始存配置后再改，就要付"实物重设参数"的代价。现在是最佳时机。

## 1. 确认的设计决策

| # | 决策点 | 选定 |
|---|---|---|
| D1 | size 字段宽度 | uint8(1B) → uint16(2B，小端)。上限 255→65535。 |
| D2 | header 字节数 | kConfigHeaderBytes 4 → 5（magic2 + version1 + size2）。 |
| D3 | 版本号 | kConfigVersion 2 → 3（格式变更，旧 v2 blob 必须失效）。 |
| D4 | 静态守卫 | `static_assert(sizeof(ControllerConfig) <= 255)` → `<= 65535`。 |
| D5 | 本轮加字段？ | 否。只扩 size 字段宽度，不加任何配置字段，sizeof 不变。 |

## 2. 改动（全部在 `core/config/config_store.{h,cpp}`）

### 2.1 新 blob 布局
```
[magic 2B][version 1B][size 2B 小端][payload][crc16 2B]
 buf[0..1] buf[2]      buf[3..4]      buf[5..]  末尾2B
```

### 2.2 config_store.h
- `kConfigVersion`：2 → 3（注释：was 2，size 字段 uint8→uint16）。
- `kConfigHeaderBytes`：4 → 5。
- 顶部布局注释更新为 size(2)。
- `kConfigBlobSize=512` 不变（足够）。

### 2.3 config_store.cpp
- 静态守卫 255 → 65535，注释更新（"size 字段现 uint16，上限 65535"）。
- `serializeConfig`：
  - `buf[3] = size & 0xFF; buf[4] = size >> 8;`（小端 uint16），替换原单字节 `buf[3]=payload`。
  - crc 与 payload 偏移用 `kConfigHeaderBytes`（=5）计算，改常量即自动跟随。
- `deserializeConfig`：
  - 校验 `buf[2] != kConfigVersion`（version 3）→ 旧 v2 blob 在此判失败。
  - size 读取：`uint16_t size = buf[3] | (buf[4] << 8);` 校验 `size == sizeof(ControllerConfig)`，替换原 `buf[3] != (uint8)payload`。
  - crc 位置 `kConfigHeaderBytes + payload`（=5+payload），改常量自动跟随。
  - `len < total` 守卫沿用（total = 5 + payload + 2）。

## 3. 旧 blob 处理（安全点）
- 升 version 2→3 后，`deserializeConfig` 的 `buf[2] != kConfigVersion` 让**所有旧 v2 blob 判失败返回 false**——期望行为，格式变了旧配置不可读。
- `ConfigStore::load` 返回 false → 调用方回退默认 `ControllerConfig`（现有契约：load 失败用默认）。
- ⚠️ **实物影响**：刷入此固件后首次启动读不到旧配置 → 用默认 → 实物需重设/标定一次。但因 main.cpp 尚未调 load/save，当前实物无存量 blob，实际代价≈0。

## 4. 测试（`test/test_config_store/test_config_store.cpp`）
- **现有往返测试**：serialize→deserialize 拿回原值。若它们未硬编码 buf 偏移/header 字节数，应自动通过；若硬编码了偏移需更新到新布局。先跑确认，再按需改。
- 新增 `test_header_is_5_bytes`：`TEST_ASSERT_EQUAL_INT(5, wp::kConfigHeaderBytes)`。
- 新增 `test_size_field_uint16_little_endian`：序列化当前 ControllerConfig，断言 `buf[3]|(buf[4]<<8) == sizeof(ControllerConfig)`，且 buf[3]=低字节、buf[4]=高字节摆放正确。
- 新增 `test_old_version2_blob_rejected`：手工造一个 version=2、旧 4 字节 header 的 blob，喂 deserializeConfig，断言返回 false（旧格式被拒）。注：deserialize 先查 version（`buf[2]!=3`）再算 crc，故 version 不符会先行返回 false——本测试验的是"版本门拦截"，不依赖 crc 是否凑对。
- 更新任何硬编码 `kConfigVersion==2` 或 header==4 的测试预期。

## 5. 影响面
- 单文件格式改动 + config 往返测试更新。
- `sizeof(ControllerConfig)` 不变（不加字段）。
- NVS backend（`MemoryConfigBackend` / `hal_esp32 nvs_config_backend`）**不改**——它们按字节数搬运 blob，不关心 header 内部结构。
- 板载/SITL 运行时零影响（main.cpp 未调 load/save；纯把格式准备好）。

## 6. 实施顺序
1. config_store.h：改 kConfigVersion=3、kConfigHeaderBytes=5、布局注释。
2. config_store.cpp：静态守卫 65535、serializeConfig 写 uint16 size、deserializeConfig 读 uint16 size + version 校验。
3. 测试：跑现有往返确认/修正 + 加 3 新测试。
4. 全量回归 + 文档台账更新。

## 7. 已知限制（⚠️）
- 升 version 让旧 NVS blob 失效（实物重设参数一次）；因 main.cpp 未调 load/save，当前代价≈0。
- 本轮不加任何配置字段（纯格式升级）；LadrcConfig 等留给各自 spec。
- size 上限升至 65535，远超配置可能体量；is_trivially_copyable POD 守卫不变（仍禁 string/vector）。

## 附录：与现有代码衔接
- `config_store.h:9` 布局注释、`:11` kConfigVersion、`:12` kConfigHeaderBytes。
- `config_store.cpp:10` static_assert、`:28-41` serializeConfig、`:43-57` deserializeConfig。
- `test/test_config_store/test_config_store.cpp` 往返测试 + 新增 3 测试。
