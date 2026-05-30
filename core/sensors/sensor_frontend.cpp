#include "sensors/sensor_frontend.h"

namespace wp {

static bool probeInit(IGyroAccel* p)    { return p && p->probe() && p->init(); }
static bool probeInit(IMagnetometer* p) { return p && p->probe() && p->init(); }
static bool probeInit(IBarometer* p)    { return p && p->probe() && p->init(); }
static bool probeInit(IGnss* p)         { return p && p->probe() && p->init(); }
static bool probeInit(IAirspeed* p)     { return p && p->probe() && p->init(); }

void SensorFrontend::begin() {
    has_imu  = probeInit(gyro_);
    has_mag  = probeInit(mag_);
    has_baro = probeInit(baro_);
    has_gnss = probeInit(gnss_);
    has_air  = probeInit(air_);
    detectTier();
}

void SensorFrontend::detectTier() {
    // base = 陀螺+加速度+磁力计+气压（§6.0/附录 C：少磁力计达不到 base）
    if (!(has_imu && has_mag && has_baro)) { tier_ = SensorTier::None; return; }
    tier_ = SensorTier::Base;
    if (has_gnss) tier_ = SensorTier::Plus;           // plus = base + GPS
    if (has_gnss && has_air) tier_ = SensorTier::Pro; // pro = plus + 空速
    // max（冗余表决）留待后续多实例实现
}

void SensorFrontend::poll(SensorBundle& out) {
    if (!(has_imu  && gyro_ && gyro_->read(out.imu)))     out.imu.valid      = false;
    if (!(has_mag  && mag_  && mag_->read(out.mag)))      out.mag.valid      = false;
    if (!(has_baro && baro_ && baro_->read(out.baro)))    out.baro.valid     = false;
    if (!(has_gnss && gnss_ && gnss_->read(out.gnss)))    out.gnss.valid     = false;
    if (!(has_air  && air_  && air_->read(out.airspeed))) out.airspeed.valid = false;
}

}  // namespace wp
