#include <unity.h>
#include "controller.h"

void setUp() {}
void tearDown() {}

void test_passthrough_maps_channels_to_servos() {
    wp::Controller c;
    wp::ControlInput in{};
    in.dt = 0.001f;
    in.link_ok = true;
    for (int i = 0; i < wp::kNumChannels; ++i) in.channels[i] = 1000 + i * 50;
    wp::ServoCommand out = c.update(in);
    for (int i = 0; i < wp::kNumServos; ++i) {
        TEST_ASSERT_EQUAL_UINT16(1000 + i * 50, out.servo[i]);
    }
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_passthrough_maps_channels_to_servos);
    return UNITY_END();
}
