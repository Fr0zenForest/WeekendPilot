#include "core_c_api.h"
#include "controller.h"
#include "sensors/sensor_frontend.h"
#include "sensors/mock_sensors.h"
#include <cstring>

namespace {
// SITL 句柄：Controller + Frontend + 三个 Mock（喂 SITL 合成数据）。
// ⚠️ Mock 喂的是合成数据（同既有 synth_mag/理想气压），验证 Frontend 通路逻辑，
//    非真实传感器信号。
struct SitlCore {
    wp::Controller controller;
    wp::SensorFrontend frontend;
    wp::MockGyroAccel imu;
    wp::MockMag mag;
    wp::MockBaro baro;
    SitlCore() {
        frontend.setGyroAccel(&imu);
        frontend.setMagnetometer(&mag);
        frontend.setBarometer(&baro);
        frontend.begin();   // probe+init 三个 Mock（present 默认 true）-> 探测档位
        wp::ControllerConfig cfg;
        cfg.althold_enabled = true;
        cfg.althold_channel = 7;   // ch8
        controller.setConfig(cfg);
    }
};
}  // namespace

extern "C" {

void* wp_controller_create() { return new SitlCore(); }
void  wp_controller_destroy(void* h) { delete static_cast<SitlCore*>(h); }

void wp_controller_update(void* h,
                          const unsigned short* channels,
                          const float* imu6,
                          const float* mag3, int mag_valid,
                          float baro_alt_m, int baro_valid,
                          float dt_s, int link_ok,
                          unsigned short* servos_out) {
    auto* c = static_cast<SitlCore*>(h);

    uint16_t ch[wp::kNumChannels];
    for (int i = 0; i < wp::kNumChannels; ++i)
        ch[i] = static_cast<uint16_t>(channels[i]);

    // 把 SITL 入参写进 Mock.sample（⚠️ 合成数据，非真实传感器信号）
    c->imu.sample.gyro_x = imu6[0]; c->imu.sample.gyro_y = imu6[1]; c->imu.sample.gyro_z = imu6[2];
    c->imu.sample.accel_x = imu6[3]; c->imu.sample.accel_y = imu6[4]; c->imu.sample.accel_z = imu6[5];
    c->imu.sample.valid = true;
    c->mag.sample.mag_x = mag3[0]; c->mag.sample.mag_y = mag3[1]; c->mag.sample.mag_z = mag3[2];
    c->mag.sample.valid = (mag_valid != 0);
    c->mag.present = (mag_valid != 0);   // mag_valid=0 时 Mock 装作不在
    c->baro.sample.altitude_m = baro_alt_m; c->baro.sample.valid = (baro_valid != 0);
    c->baro.present = (baro_valid != 0);

    wp::SensorBundle bundle{};
    c->frontend.poll(bundle);

    wp::ServoCommand out = c->controller.updateFromBundle(ch, bundle, dt_s, (link_ok != 0));
    std::memcpy(servos_out, out.servo, sizeof(out.servo));
}

void wp_controller_attitude(void* h, float* rpy_out) {
    auto* c = static_cast<SitlCore*>(h);
    wp::Attitude a = c->controller.attitude();
    rpy_out[0] = a.roll_deg; rpy_out[1] = a.pitch_deg; rpy_out[2] = a.yaw_deg;
}

}  // extern "C"
