#include <Arduino.h>
#include "types.h"
#include "controller.h"

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

    wp::ControlInput in{};
    for (int i = 0; i < wp::kNumChannels; ++i)
        in.channels[i] = link_ok ? g_channels[i] : 1500;
    in.dt = 0.001f;
    in.link_ok = link_ok;
    in.imu.valid = false;

    wp::ServoCommand out = g_controller.update(in);
    for (int i = 0; i < wp::kNumServos; ++i) writeServoUs(i, out.servo[i]);
    delay(2);
}
