#pragma once
#include "sensors/sensor_interfaces.h"

namespace wp {

// 喂给 core 的单组干净样本（core 永远只看到这一组，不知背后几个传感器）。
struct SensorBundle {
    ImuSample      imu;
    MagSample      mag;
    BaroSample     baro;
    GnssSample     gnss;
    AirspeedSample airspeed;
};

// 聚合层：持有各类型接口指针，begin() 探测+初始化并推断档位，poll() 读一轮填 bundle。
// 本次为单实例直通；max 档多实例表决留待后续（接口已为此预留）。
class SensorFrontend {
public:
    void setGyroAccel(IGyroAccel* p)      { gyro_ = p; }
    void setMagnetometer(IMagnetometer* p) { mag_  = p; }
    void setBarometer(IBarometer* p)      { baro_ = p; }
    void setGnss(IGnss* p)               { gnss_ = p; }
    void setAirspeed(IAirspeed* p)        { air_  = p; }

    void begin();               // probe + init 全部已注册传感器，推断 tier_
    void poll(SensorBundle& out); // 读一轮；缺失/失败的样本 valid=false
    SensorTier tier() const { return tier_; }

    bool has_imu  = false;
    bool has_mag  = false;
    bool has_baro = false;
    bool has_gnss = false;
    bool has_air  = false;

private:
    IGyroAccel*    gyro_ = nullptr;
    IMagnetometer* mag_  = nullptr;
    IBarometer*    baro_ = nullptr;
    IGnss*         gnss_ = nullptr;
    IAirspeed*     air_  = nullptr;
    SensorTier     tier_ = SensorTier::None;

    void detectTier();
};

}  // namespace wp
