#include "core_c_api.h"
#include "controller.h"
#include <cstring>

extern "C" {

void* wp_controller_create() { return new wp::Controller(); }
void  wp_controller_destroy(void* h) { delete static_cast<wp::Controller*>(h); }

void wp_controller_update(void* h,
                          const unsigned short* channels,
                          const float* imu6,
                          float baro_alt_m, int baro_valid,
                          float dt_s, int link_ok,
                          unsigned short* servos_out) {
    auto* c = static_cast<wp::Controller*>(h);
    wp::ControlInput in{};
    for (int i = 0; i < wp::kNumChannels; ++i) in.channels[i] = channels[i];
    in.imu.gyro_x = imu6[0]; in.imu.gyro_y = imu6[1]; in.imu.gyro_z = imu6[2];
    in.imu.accel_x = imu6[3]; in.imu.accel_y = imu6[4]; in.imu.accel_z = imu6[5];
    in.imu.valid = true;
    in.baro.altitude_m = baro_alt_m; in.baro.valid = (baro_valid != 0);
    in.dt = dt_s; in.link_ok = (link_ok != 0);
    wp::ServoCommand out = c->update(in);
    std::memcpy(servos_out, out.servo, sizeof(out.servo));
}

}  // extern "C"
