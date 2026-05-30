#include "core_c_api.h"
#include "controller.h"
#include <cstring>

extern "C" {

void* wp_controller_create() { return new wp::Controller(); }
void  wp_controller_destroy(void* h) { delete static_cast<wp::Controller*>(h); }

void wp_controller_update(void* h,
                          const unsigned short* channels,
                          const float* imu6,
                          const float* mag3, int mag_valid,
                          float baro_alt_m, int baro_valid,
                          float dt_s, int link_ok,
                          unsigned short* servos_out) {
    auto* c = static_cast<wp::Controller*>(h);

    uint16_t ch[wp::kNumChannels];
    for (int i = 0; i < wp::kNumChannels; ++i)
        ch[i] = static_cast<uint16_t>(channels[i]);

    wp::SensorBundle bundle{};
    bundle.imu.gyro_x = imu6[0]; bundle.imu.gyro_y = imu6[1]; bundle.imu.gyro_z = imu6[2];
    bundle.imu.accel_x = imu6[3]; bundle.imu.accel_y = imu6[4]; bundle.imu.accel_z = imu6[5];
    bundle.imu.valid = true;
    bundle.mag.mag_x = mag3[0]; bundle.mag.mag_y = mag3[1]; bundle.mag.mag_z = mag3[2];
    bundle.mag.valid = (mag_valid != 0);
    bundle.baro.altitude_m = baro_alt_m; bundle.baro.valid = (baro_valid != 0);

    wp::ServoCommand out = c->updateFromBundle(ch, bundle, dt_s, (link_ok != 0));
    std::memcpy(servos_out, out.servo, sizeof(out.servo));
}

void wp_controller_attitude(void* h, float* rpy_out) {
    auto* c = static_cast<wp::Controller*>(h);
    wp::Attitude a = c->attitude();
    rpy_out[0] = a.roll_deg; rpy_out[1] = a.pitch_deg; rpy_out[2] = a.yaw_deg;
}

}  // extern "C"
