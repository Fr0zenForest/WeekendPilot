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
    wp::ControlInput in{};
    for (int i = 0; i < wp::kNumChannels; ++i) in.channels[i] = channels[i];
    in.imu.gyro_x = imu6[0]; in.imu.gyro_y = imu6[1]; in.imu.gyro_z = imu6[2];
    in.imu.accel_x = imu6[3]; in.imu.accel_y = imu6[4]; in.imu.accel_z = imu6[5];
    in.imu.valid = true;
    in.mag.mag_x = mag3[0]; in.mag.mag_y = mag3[1]; in.mag.mag_z = mag3[2];
    in.mag.valid = (mag_valid != 0);
    in.baro.altitude_m = baro_alt_m; in.baro.valid = (baro_valid != 0);
    in.dt = dt_s; in.link_ok = (link_ok != 0);
    wp::ServoCommand out = c->update(in);
    std::memcpy(servos_out, out.servo, sizeof(out.servo));
}

void wp_controller_attitude(void* h, float* rpy_out) {
    auto* c = static_cast<wp::Controller*>(h);
    wp::Attitude a = c->attitude();
    rpy_out[0] = a.roll_deg; rpy_out[1] = a.pitch_deg; rpy_out[2] = a.yaw_deg;
}

}  // extern "C"
