#include "sensors/bmp390_compensate.h"

namespace wp {

Bmp390Calib bmp390ParseCalib(const uint8_t buf[21]) {
    auto u16 = [&](int o) -> uint16_t {
        return (uint16_t)buf[o] | ((uint16_t)buf[o + 1] << 8);
    };
    auto i16 = [&](int o) -> int16_t {
        return (int16_t)((uint16_t)buf[o] | ((uint16_t)buf[o + 1] << 8));
    };
    auto i8 = [&](int o) -> int8_t { return (int8_t)buf[o]; };

    Bmp390Calib c{};
    c.par_t1  = (float)u16(0)  * 256.0f;
    c.par_t2  = (float)u16(2)  / 1073741824.0f;            // 2^30
    c.par_t3  = (float)i8(4)   / 281474976710656.0f;       // 2^48
    c.par_p1  = ((float)i16(5) - 16384.0f) / 1048576.0f;   // 2^20
    c.par_p2  = ((float)i16(7) - 16384.0f) / 536870912.0f; // 2^29
    c.par_p3  = (float)i8(9)   / 4294967296.0f;            // 2^32
    c.par_p4  = (float)i8(10)  / 137438953472.0f;          // 2^37
    c.par_p5  = (float)u16(11) * 8.0f;
    c.par_p6  = (float)u16(13) / 64.0f;
    c.par_p7  = (float)i8(15)  / 256.0f;
    c.par_p8  = (float)i8(16)  / 32768.0f;
    c.par_p9  = (float)i16(17) / 281474976710656.0f;       // 2^48
    c.par_p10 = (float)i8(19)  / 281474976710656.0f;       // 2^48
    c.par_p11 = (float)i8(20)  / 36893488147419103232.0f;  // 2^65
    c.t_lin = 0.0f;
    return c;
}

float bmp390CompensateTemperature(uint32_t uncomp_temp, Bmp390Calib& c) {
    float pd1 = (float)((float)uncomp_temp - c.par_t1);
    float pd2 = pd1 * c.par_t2;
    c.t_lin = pd2 + (pd1 * pd1) * c.par_t3;
    return c.t_lin;
}

float bmp390CompensatePressure(uint32_t uncomp_press, const Bmp390Calib& c) {
    float pd1, pd2, pd3, pd4, po1, po2;
    pd1 = c.par_p6 * c.t_lin;
    pd2 = c.par_p7 * (c.t_lin * c.t_lin);
    pd3 = c.par_p8 * (c.t_lin * c.t_lin * c.t_lin);
    po1 = c.par_p5 + pd1 + pd2 + pd3;

    pd1 = c.par_p2 * c.t_lin;
    pd2 = c.par_p3 * (c.t_lin * c.t_lin);
    pd3 = c.par_p4 * (c.t_lin * c.t_lin * c.t_lin);
    po2 = (float)uncomp_press * (c.par_p1 + pd1 + pd2 + pd3);

    pd1 = (float)uncomp_press * (float)uncomp_press;
    pd2 = c.par_p9 + c.par_p10 * c.t_lin;
    pd3 = pd1 * pd2;
    pd4 = pd3 + ((float)uncomp_press * (float)uncomp_press * (float)uncomp_press) * c.par_p11;

    return po1 + po2 + pd4;   // Pa
}

Bmp390Reading bmp390Decode(const uint8_t data[6], Bmp390Calib& c) {
    Bmp390Reading r{};
    uint32_t raw_press = (uint32_t)data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16);
    uint32_t raw_temp  = (uint32_t)data[3] | ((uint32_t)data[4] << 8) | ((uint32_t)data[5] << 16);
    if (raw_press == 0 || raw_temp == 0) return r;
    r.temperature_c = bmp390CompensateTemperature(raw_temp, c);
    r.pressure_pa   = bmp390CompensatePressure(raw_press, c);
    r.valid = true;
    return r;
}

}  // namespace wp
