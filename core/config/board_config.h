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

}  // namespace wp
