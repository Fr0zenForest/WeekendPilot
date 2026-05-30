#include <unity.h>
#include "types.h"

void setUp() {}
void tearDown() {}

void test_sanity() { TEST_ASSERT_EQUAL_INT(4, 2 + 2); }

void test_types_sizes() {
    wp::ControlInput in{};
    in.channels[0] = 1500;
    in.dt = 0.001f;
    TEST_ASSERT_EQUAL_UINT16(1500, in.channels[0]);
    TEST_ASSERT_EQUAL_INT(8, wp::kNumServos);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_sanity);
    RUN_TEST(test_types_sizes);
    return UNITY_END();
}
