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
static const int kCrsfRxPin = 44;
static const int kCrsfTxPin = 43;

static const uint8_t CRSF_ADDR = 0xC8;
static const uint8_t CRSF_FRAMETYPE_RC = 0x16;

wp::Controller g_controller;
uint8_t  g_buf[64];
uint16_t g_channels[wp::kNumChannels];
uint32_t g_lastRcMs = 0;

void setupPwm() {
    for (int i = 0; i < wp::kNumServos; ++i) {
        ledcSetup(i, 50, 16);
        ledcAttachPin(kServoPins[i], i);
        ledcWrite(i, (uint32_t)(1500.0 / 20000.0 * 65535));
    }
}

void writeServoUs(int ch, uint16_t us) {
    if (us < 1000) us = 1000;
    if (us > 2000) us = 2000;
    ledcWrite(ch, (uint32_t)((double)us / 20000.0 * 65535));
}

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
    Serial.printf("[wp] sensor tier=%d imu=%d baro=%d mag=%d\n",
                  (int)g_frontend.tier(), g_frontend.has_imu,
                  g_frontend.has_baro, g_frontend.has_mag);
    setupPwm();
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
    for (int i = 0; i < wp::kNumServos; ++i) writeServoUs(i, out.servo[i]);
    delay(2);
}
