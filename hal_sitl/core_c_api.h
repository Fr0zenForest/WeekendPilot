#pragma once
#ifdef __cplusplus
extern "C" {
#endif

void* wp_controller_create();
void  wp_controller_destroy(void* handle);

void wp_controller_update(void* handle,
                          const unsigned short* channels,
                          const float* imu6,
                          const float* mag3, int mag_valid,
                          float baro_alt_m, int baro_valid,
                          float dt_s, int link_ok,
                          unsigned short* servos_out);

#ifdef __cplusplus
}
#endif
