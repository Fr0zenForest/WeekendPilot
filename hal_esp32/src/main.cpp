#include <Arduino.h>
#include "types.h"
#include "controller.h"
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
#include "diag/status_line.h"
#include "io/composite_servo_output.h"
#include "io/ledc_output.h"
#if WP_HAS_PWM_EXPANDER
#include "io/pca9685_output.h"
#endif
#if WP_HAS_LANDING_GEAR
#include "nav/landing_gear.h"
#include "drivers/ina3221.h"
#include "drivers/gear_actuators.h"
#endif

// 调试日志：默认开（开发期）。正式飞行可在 platformio.ini build_flags 加 -DWP_DEBUG_LOG=0 关闭。
#ifndef WP_DEBUG_LOG
#define WP_DEBUG_LOG 1
#endif
#define WP_LOG_PERIOD_MS 200   // 状态行打印周期（~5Hz，不刷屏不拖控制环）

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

static const int kServoPins[wp::kNumServos] = {1, 2, 8, 9, 10, 15, 16, 17};
static wp::CompositeServoOutput g_out;
static wp::LedcOutput g_ledc(kServoPins, wp::kNumServos);
#if WP_HAS_PWM_EXPANDER
static wp::Pca9685Output g_pca(Wire, wp::kAddrPwmExpander);
#endif
#if WP_HAS_LANDING_GEAR
static wp::LandingGear g_gear;
static wp::Ina3221 g_ina(Wire, wp::kAddrCurrentSense);
// 起落架输出通道（全局逻辑索引）：开 PCA9685 时用第一个扩展口（=kNumServos）；
// 没开扩展时退回 LEDC 空闲口 7（标准布局 0~3 主舵面，7 空闲）。
#if WP_HAS_PWM_EXPANDER
constexpr int kGearOutCh = wp::kNumServos;   // PCA9685 的 0 号口
#else
constexpr int kGearOutCh = 7;                // LEDC 空闲口
#endif
static wp::GearServo g_gear_act(g_out, kGearOutCh);
constexpr int    kGearCurrentCh = 0;       // INA3221 通道 0
constexpr float  kGearShuntOhm  = 0.01f;   // 采样电阻（实物标定）
constexpr uint8_t kGearRcChannel = 8;      // ch9：收放拨杆（0-based 索引 8）
#endif
static const int kCrsfRxPin = 44;
static const int kCrsfTxPin = 43;

static const uint8_t CRSF_ADDR = 0xC8;
static const uint8_t CRSF_FRAMETYPE_RC = 0x16;

wp::Controller g_controller;
uint8_t  g_buf[64];
uint16_t g_channels[wp::kNumChannels];
uint32_t g_lastRcMs = 0;

void parseCrsfRc(const uint8_t* p) {
    const uint8_t* d = p + 3;
    uint32_t bits = 0;
    int bitcnt = 0, ch = 0;
    for (int i = 0; i < 22 && ch < 16; ++i) {
        bits |= (uint32_t)d[i] << bitcnt;
        bitcnt += 8;
        while (bitcnt >= 11 && ch < 16) {
            uint16_t raw = bits & 0x7FF;
            bits >>= 11;
            bitcnt -= 11;
            g_channels[ch++] = (uint16_t)(raw * 0.62477f + 881);
        }
    }
    g_lastRcMs = millis();
}

void setup() {
    Serial.begin(115200);
    // 状态灯：开机即红（启动中）。WS2812 用框架内置 neopixelWrite，无需库。
    neopixelWrite(wp::kPinStatusLed, 40, 0, 0);   // 红，中等亮度
#if WP_DEBUG_LOG
    // 原生 USB CDC 需等主机枚举；等 Serial 就绪（最多 ~2s）再打，避免丢开头日志。
    uint32_t t_usb = millis();
    while (!Serial && (millis() - t_usb) < 2000) { delay(10); }
    delay(300);
#endif
    Serial1.begin(420000, SERIAL_8N1, kCrsfRxPin, kCrsfTxPin);
    for (int i = 0; i < wp::kNumChannels; ++i) g_channels[i] = 1500;
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
    g_frontend.begin();   // probe+init registered drivers; missing ones auto-degrade
#if WP_DEBUG_LOG
    Serial.printf("\n[wp] === boot self-check ===\n");
    Serial.printf("[wp] I2C SDA=%d SCL=%d @400k\n", wp::kPinI2cSda, wp::kPinI2cScl);
    Serial.printf("[wp] sensor tier=%d  imu=%d baro=%d mag=%d\n",
                  (int)g_frontend.tier(), g_frontend.has_imu,
                  g_frontend.has_baro, g_frontend.has_mag);
    Serial.printf("[wp] caps(compiled): IMU=%d BARO=%d MAG=%d GPS=%d AIR=%d BBOX=%d\n",
                  WP_HAS_IMU, WP_HAS_BARO, WP_HAS_MAG, WP_HAS_GPS, WP_HAS_AIRSPEED, WP_HAS_BLACKBOX);
    if (!g_frontend.has_imu)
        Serial.printf("[wp] WARN: no IMU detected -> controller will passthrough (no stabilization)\n");
    Serial.printf("[wp] === running ===\n");
#endif
    g_out.addBackend(&g_ledc);
#if WP_HAS_PWM_EXPANDER
    g_out.addBackend(&g_pca);
#endif
    g_out.begin();
    g_out.setFrequencyHz(50);   // D2：默认 50Hz，将来从 NVS config 读
#if WP_HAS_LANDING_GEAR
    {
        wp::LandingGearConfig gc;   // 默认 enabled=false（全写好先不使能）
        g_gear.setConfig(gc);
        g_ina.begin();
        // 上电不主动驱动：状态从默认 Retracted 起（将来从 NVS gear_last_state 恢复）。
        pinMode(wp::kPinGearAlert, INPUT_PULLUP);
    }
#endif
    // 状态灯转暗绿：系统初始化完成、即将进入控制循环。低亮度防晃眼。
    neopixelWrite(wp::kPinStatusLed, 0, 12, 0);   // 暗绿
}

void loop() {
    static int idx = 0;
    static int len = 0;
    while (Serial1.available()) {
        uint8_t b = Serial1.read();
        if (idx == 0) {
            if (b == CRSF_ADDR) g_buf[idx++] = b;
        } else if (idx == 1) {
            len = b;
            g_buf[idx++] = b;
        } else {
            g_buf[idx++] = b;
            if (idx >= len + 2) {
                if (g_buf[2] == CRSF_FRAMETYPE_RC) parseCrsfRc(g_buf);
                idx = 0;
                len = 0;
            }
            if (idx >= (int)sizeof(g_buf)) { idx = 0; len = 0; }
        }
    }

    bool link_ok = (millis() - g_lastRcMs) < 500;

    uint16_t ch[wp::kNumChannels];
    for (int i = 0; i < wp::kNumChannels; ++i)
        ch[i] = link_ok ? g_channels[i] : 1500;

    wp::SensorBundle bundle{};
    g_frontend.poll(bundle);   // missing/failed samples have valid=false

    wp::ServoCommand out = g_controller.updateFromBundle(ch, bundle, 0.001f, link_ok);
    for (int i = 0; i < wp::kNumServos; ++i) g_out.writeUs(i, out.servo[i]);
#if WP_HAS_LANDING_GEAR
    if (g_gear.enabled()) {
        wp::LandingGearInputs gi;
        // 收放指令：拨杆 > 1500µs 视为"放下"。link_ok 交给状态机：失控时它冻结指令沿，
        // 维持当前动作，绝不反转或重启 Fault（见 LandingGear::update）。
        gi.deploy_cmd = (g_channels[kGearRcChannel] > 1500);
        gi.link_ok = link_ok;
        float amps = 0.0f;
        if (g_ina.readCurrent(kGearCurrentCh, kGearShuntOhm, amps)) gi.current_a = amps;
        gi.alert = (digitalRead(wp::kPinGearAlert) == LOW);   // INA3221 ALERT 低有效
        gi.dt = 0.002f;   // 与 loop delay(2) 一致量级
        wp::LandingGearOutput go = g_gear.update(gi);
        g_gear_act.apply(go.drive);
    }
#endif
#if WP_DEBUG_LOG
    static uint32_t last_log_ms = 0;
    uint32_t now = millis();
    if (now - last_log_ms >= WP_LOG_PERIOD_MS) {
        last_log_ms = now;
        wp::StatusSnapshot snap{};
        snap.t_ms = now;
        snap.mode = (uint8_t)g_controller.activeMode();
        wp::Attitude att = g_controller.attitude();
        snap.roll_deg = att.roll_deg; snap.pitch_deg = att.pitch_deg; snap.yaw_deg = att.yaw_deg;
        snap.link_ok = link_ok;
        snap.imu_valid = bundle.imu.valid;
        snap.baro_valid = bundle.baro.valid;
        snap.mag_valid = bundle.mag.valid;
        snap.althold_engaged = g_controller.altHoldEngaged();
        snap.autotrim_learning = g_controller.autoTrimLearning();
        for (int i = 0; i < wp::kNumServos; ++i) snap.servo[i] = out.servo[i];
        snap.tier = (int8_t)g_frontend.tier();
        char line[wp::kStatusLineCap];
        wp::formatStatusLine(snap, line, sizeof(line));
        Serial.println(line);
    }
#endif
    delay(2);
}
