#include "sensors/baro_altitude.h"
#include <cmath>

namespace wp {

float baroPressureToAltitude(float pressure_pa, float p_ref_pa) {
    if (pressure_pa <= 0.0f || p_ref_pa <= 0.0f) return 0.0f;
    // ISA 对流层：h = 44330 * (1 - (p/p_ref)^(1/5.255))
    return 44330.0f * (1.0f - std::pow(pressure_pa / p_ref_pa, 0.1902949f));
}

}  // namespace wp
