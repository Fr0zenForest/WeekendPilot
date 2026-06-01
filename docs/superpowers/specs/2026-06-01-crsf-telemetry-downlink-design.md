# CRSF 遥测下行子系统 — 设计文档

- 日期：2026-06-01
- 状态：设计待评审
- 关联：[[weekendpilot-dev-progress]]、[[weekendpilot-board-flash-serial]]、起落架设计 `docs/superpowers/specs/2026-06-01-pwm-output-landing-gear-design.md`
- 阶段定位：阶段 4 起步（WiFi WebUI 的替代/前置——飞机内无法接飞控，回传只能走遥控器）

## 0. 背景与动机

飞机装好后无法用串口连接飞控，只能通过遥控器（ELRS/CRSF 链路）遥控与遥测。要实现"地面看飞控状态/传感器值"（以及将来校准时的数值回传），必须有 **CRSF 遥测下行**：飞控 → 接收机 → 遥控器屏幕。

当前代码只解析接收的 RC 帧（`main.cpp:parseCrsfRc`，frame type 0x16），从未往回发遥测帧。本子系统补上下行方向。它是后续"校准/自检模式"回传能力的地基——先单独落地，校准模式建在其上（下一个 spec）。

### 整体路线（本 spec 只做第 1 块）
1. **CRSF 遥测下行**（本 spec）：姿态/模式/电池等发回遥控器。
2. 校准/自检模式框架 + 起落架方向校准 + IMU 零偏/陀螺安装偏角校准（下一个 spec，建在 1 之上）。

校准相关的已对齐决策（记录备查，本 spec 不实现）：触发与回传都走 ELRS（控制通道触发 + 遥测下行回传，不用串口）；校准通道与飞行通道隔离/可复用；进入校准模式需油门低等安全门控；自检（只读）与校准（会动/存偏移）分开；reversed 反转在 HAL 层生效、bool 随 config 存 NVS；堵转式起落架校准靠"驱动到端点+人工判定"；动态校准仅陀螺安装偏角（地面直线低速滑行）。

## 1. 确认的设计决策

| # | 决策点 | 选定 |
|---|---|---|
| D1 | 传输层 | 复用现有 CRSF 链路（Serial1, TX=GPIO43），半双工下行 |
| D2 | 帧类型策略 | 标准+自定义混合：标准帧（姿态/模式/电池）原生显示；自定义帧本轮只留编码接口+占位 |
| D3 | 编解码归属 | core 纯帧编码（PC 可测），HAL 只管发送调度 |
| D4 | 数据来源 | 复用 Controller 只读 getter（attitude/activeMode），控制核零改动 |
| D5 | 门控 | WP_HAS_TELEMETRY 编译期 + 默认关（全写好先不使能） |
| D6 | CRC | CRC8/DVB-S2，多项式 0xD5，覆盖 type..payload（TBS CRSF spec） |

## 2. 架构

两层，沿用现有 `*_decode` 纯函数模式（参 ist8310_decode/icm42688_decode）：

```
core/telemetry/crsf_telem.{h,cpp}        纯帧编码，PC 单测
  - crc8_dvbs2(const uint8_t*, len)               多项式 0xD5
  - buildFrame(addr,type,payload,plen,out,cap)    封 [addr][len][type][payload][crc8]，返回总字节数
  - encodeAttitude(roll,pitch,yaw_rad, out,cap)   -> 0x1E 帧
  - encodeFlightMode(const char* mode, out,cap)   -> 0x21 帧
  - encodeBattery(v,i,mah,pct, out,cap)           -> 0x08 帧（留接口，先不接数据源）

hal_esp32/src/telemetry/crsf_telem_tx.{h,cpp}    HAL 发送，门控 WP_HAS_TELEMETRY
  - CrsfTelemetryTx(HardwareSerial& uart)
  - 调度器：按帧类型分频（姿态~10Hz，模式/电池~2Hz）
  - tick(now_ms, snapshot)：到点则编码对应帧并写 uart
```

### 2.1 帧格式（权威：TBS CRSF spec）
`[sync/addr=0xC8] [len] [type] [payload...] [crc8]`，整帧 ≤64 字节。
- `len` = type + payload + crc 的字节数（即 payload_len + 2）。
- `crc8` = CRC8/DVB-S2(poly 0xD5)，覆盖 type 到 payload 末尾。
- 0x1E 姿态：payload = 3× int16 大端，单位 **rad × 10000**（roll/pitch/yaw）。
- 0x21 飞行模式：payload = 以 NUL 结尾的 ASCII 字符串（如 "ANGL"/"RATE"/"OFF "/"AHLD"）。
- 0x08 电池：payload = 电压(int16,0.1V) + 电流(int16,0.1A) + 容量(uint24,mAh) + 剩余%(uint8)。

### 2.2 发送调度（半双工不淹没上行）
- ELRS 链路半双工，下行带宽有限；不可每个控制拍都发。
- HAL 侧维护各帧的"下次发送时刻"，loop 末尾（RC 解析之后、不打断接收）按到点轮流发**一个**帧，避免突发占满。
- 频率初值：姿态 10Hz、模式 2Hz、电池 2Hz（可后续调）。

## 3. 数据流

```
g_controller.attitude()  ─┐
g_controller.activeMode()─┤→ 组装 TelemSnapshot(值结构,纯数据) → CrsfTelemetryTx.tick()
(电池源,留空占位)        ─┘                                          │
                                                    core encode* → 帧字节 → Serial1.write()
                                                                              │ TX=GPIO43
                                                                       接收机 → 遥控器屏
```
TelemSnapshot 是纯值结构（类似 diag/status_line.h 的 StatusSnapshot），HAL 填好喂 tick，core 编码无 Arduino 依赖。

## 4. 门控与默认值
- `WP_HAS_TELEMETRY`：PC 测试默认开（验编码），板级默认 0；到货实测时 build_flags -D 打开。
- core 编码函数恒编译进 core（PC 全测）；HAL 发送层 #if 门控。

## 5. 测试
**core（PC ctest，Unity）** `test_crsf_telem`：
- crc8_dvbs2 已知向量（取 TBS spec / Betaflight 参考值）。
- encodeAttitude：给定 roll/pitch/yaw（含负角）→ 期望字节序列（rad×10000 大端、符号正确）。
- encodeFlightMode：字符串 payload + NUL + len + crc 正确。
- buildFrame：len 字段、crc 位置、cap 不足返回 0（越界守卫）。

**HAL** `pio run -e weekendpilot_s3` 门控开/关编译通过。⚠️ 实物：遥控器 telemetry 页确认出现 Roll/Pitch/Yaw/FM 条目（只能实物观察）。

## 6. 实施顺序
1. core `crc8_dvbs2` + `buildFrame` + 单测。
2. core `encodeAttitude`/`encodeFlightMode`/`encodeBattery` + 单测。
3. HAL `CrsfTelemetryTx` 调度器 + 门控 + pio 编译。
4. main.cpp 接入（门控）：填 TelemSnapshot，loop 末尾 tick。
5. capabilities.h 加 WP_HAS_TELEMETRY；文档+台账更新。

## 7. 已知限制（⚠️）
- 电池帧留接口但无数据源（无电压/电流采样接入），本轮发占位或不发。
- 自定义帧本轮只留编码接口+占位，实际校准数据等下个 spec。
- 半双工时序与 ELRS 下行带宽未实物验证；发送频率为初值，须实物调（遥控器丢帧/RC 卡顿即降频）。
- 遥测只能在实物遥控器观察，本轮仅 PC 编码单测 + pio 编译。
- 不接收遥控器→飞控的命令（那是校准 spec 的上行命令通道，本轮纯下行）。

## 附录：与现有代码衔接
- `main.cpp:68-72` Serial1/CRSF_ADDR 常量 → 遥测发送复用。
- `main.cpp:parseCrsfRc` 的 CRC/帧结构 → 遥测编码参照（接收已验证帧结构）。
- `controller.h` attitude()/activeMode() → 数据源（已存在只读 getter）。
- `core/diag/status_line.h` StatusSnapshot → TelemSnapshot 仿其纯值结构风格。
