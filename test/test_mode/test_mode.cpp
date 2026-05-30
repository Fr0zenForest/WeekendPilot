#include <unity.h>
#include "modes/stab_mode.h"

void setUp() {}
void tearDown() {}

void test_mode_thresholds() {
    TEST_ASSERT_EQUAL_INT((int)wp::FlightMode::Off,   (int)wp::modeFromChannel(1000));
    TEST_ASSERT_EQUAL_INT((int)wp::FlightMode::Angle, (int)wp::modeFromChannel(1500));
    TEST_ASSERT_EQUAL_INT((int)wp::FlightMode::Rate,  (int)wp::modeFromChannel(2000));
}

void test_channel_norm_roundtrip() {
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, wp::channelToNorm(1500));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.0f, wp::channelToNorm(2000));
    TEST_ASSERT_EQUAL_UINT16(1500, wp::normToServoUs(0.0f));
    TEST_ASSERT_EQUAL_UINT16(2000, wp::normToServoUs(1.0f));
}

void test_gain_channel() {
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, wp::gainFromChannel(1000));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.0f, wp::gainFromChannel(2000));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.5f, wp::gainFromChannel(1500));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_mode_thresholds);
    RUN_TEST(test_channel_norm_roundtrip);
    RUN_TEST(test_gain_channel);
    return UNITY_END();
}
