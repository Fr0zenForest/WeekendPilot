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
  // WP_HAS_BLACKBOX 默认不开：PSRAM 环形缓冲未在实物验证；带 PSRAM 的板验证后
  //   在此加 `#define WP_HAS_BLACKBOX 1` 或在 platformio.ini build_flags 加 -DWP_HAS_BLACKBOX=1。
#endif

namespace wp {

// I2C 共享总线 + 传感器地址（§6.3）
constexpr int kPinI2cSda = 18;
constexpr int kPinI2cScl = 39;
constexpr int kAddrImu   = 0x68;  // ICM-42688-P
constexpr int kAddrBaro  = 0x76;  // BMP390L
constexpr int kPinImuInt = 40;    // ICM 数据就绪中断（可选）

// IST8310 罗盘（SR25M10DI 板载，挂主 I2C 总线，地址不撞 IMU/Baro）
constexpr int kAddrMag = 0x0E;

// GPS UART（SR25M10DI 的 M10050，NMEA0183）。引脚 4/5 为 S3 自由 GPIO。
constexpr int kPinGpsRx  = 5;       // MCU RX <- 模块 TX
constexpr int kPinGpsTx  = 4;       // MCU TX -> 模块 RX
constexpr int kGpsBaud   = 38400;   // 模块默认波特率

// MS4525DO 差压空速管（挂主 I2C 总线，地址不撞 IMU/Baro/Mag）
constexpr int kAddrAirspeed = 0x28;

// PWM 8 路（§6.3）
constexpr int kPwmPins[8] = {1, 2, 8, 9, 10, 15, 16, 17};

// CRSF UART（接 SuperX）
constexpr int kPinCrsfRx = 44;
constexpr int kPinCrsfTx = 43;

// 板载 WS2812 状态灯（DevKitC-1 N8R8/N16R8 在 GPIO48）
constexpr int kPinStatusLed = 48;

// PCA9685 PWM 扩展（挂主 I2C 总线）。默认地址 0x40；A0~A5 跳线可改。
constexpr int kAddrPwmExpander = 0x40;
// PCA9685 提供的输出路数（单颗）。CompositeServoOutput 据此分配全局索引区间。
constexpr int kPwmExpanderChannels = 16;

// INA3221 三路电流监测（挂主 I2C 总线）。默认地址 0x40 会撞 PCA9685，故用 0x41。
// （INA3221 地址 0x40~0x43 由 A0 脚接 GND/VS/SDA/SCL 选）
constexpr int kAddrCurrentSense = 0x41;
// INA3221 Critical-Alert 引脚 -> ESP32 GPIO（堵转硬件中断，可选）。
// GPIO40 当前仅声明 kPinImuInt 未实际使用；若 IMU INT 启用需另选自由脚（实物核对）。
constexpr int kPinGearAlert = 41;

// 起落架 H 桥方向脚（仅类型 C 裸电机用；类型 B 连续旋转舵机不占）。
// 单电机 PWM+DIR：PWM 走 CompositeServoOutput，DIR 走此 GPIO。
constexpr int kPinGearHbridgeDir = 42;

}  // namespace wp
