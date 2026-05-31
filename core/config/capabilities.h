#pragma once
#include "config/board_config.h"   // 先让 BOARD_* 分支设的 WP_HAS_* 生效

// 能力宏框架（编译期门控）。三种来源覆盖默认值：
//   1) 板级 profile（board_config.h）按 BOARD_* 设默认
//   2) PlatformIO build_flags -D 覆盖（板载按手头硬件开关）
//   3) 未定义任何 BOARD_* 时（PC/CMake 测试构建）默认全开 —— 算法都要在 SITL 验
//
// "全写好先不使能"：功能代码恒编译进 core（PC 全开可验）；板载侧用这些宏 +
// 运行时 probe() 双重门控决定是否激活。硬件到货改一个 -D 即 enable。
//
// 注：板载门控路径（BOARD_* 场景）无法在 PC ctest 覆盖，只能靠编译期；
//     如需验证板载组合，用 `pio run -e weekendpilot_s3` 编译检查。

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
  #ifndef WP_HAS_BLACKBOX
    #define WP_HAS_BLACKBOX 1
  #endif
  #ifndef WP_HAS_PWM_EXPANDER
    #define WP_HAS_PWM_EXPANDER 1
  #endif
  #ifndef WP_HAS_LANDING_GEAR
    #define WP_HAS_LANDING_GEAR 1
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
#ifndef WP_HAS_BLACKBOX
  #define WP_HAS_BLACKBOX 0
#endif
#ifndef WP_HAS_PWM_EXPANDER
  #define WP_HAS_PWM_EXPANDER 0
#endif
#ifndef WP_HAS_LANDING_GEAR
  #define WP_HAS_LANDING_GEAR 0
#endif

namespace wp {
// constexpr 镜像：让 core 用 `if constexpr (kHasGps)` 做分支，比裸宏更安全可读。
constexpr bool kHasImu      = WP_HAS_IMU;
constexpr bool kHasBaro     = WP_HAS_BARO;
constexpr bool kHasMag      = WP_HAS_MAG;
constexpr bool kHasGps      = WP_HAS_GPS;
constexpr bool kHasAirspeed = WP_HAS_AIRSPEED;
constexpr bool kHasBlackbox    = WP_HAS_BLACKBOX;
constexpr bool kHasPwmExpander = WP_HAS_PWM_EXPANDER;
constexpr bool kHasLandingGear = WP_HAS_LANDING_GEAR;
}  // namespace wp
