#include "unity.h"
#include "blackbox/blackbox_sink.h"
#include <cstring>
using namespace wp;

void setUp() {} void tearDown() {}

// WARNING 新代码。环形缓冲逻辑可单测；板载 PSRAM sink 未实测。

// 写入未超容量：usedBytes 累加，capacity 不变。
void test_ring_accumulates() {
    uint8_t buf[64];
    RingBufferSink sink(buf, sizeof(buf));
    uint8_t data[10] = {1,2,3,4,5,6,7,8,9,10};
    TEST_ASSERT_TRUE(sink.write(data, 10));
    TEST_ASSERT_EQUAL_INT(10, (int)sink.usedBytes());
    TEST_ASSERT_EQUAL_INT(64, (int)sink.capacityBytes());
}

// 超容量环形覆盖：写入超过 buf 后，usedBytes 封顶 = capacity，且 wrapped 置位。
void test_ring_overwrites_when_full() {
    uint8_t buf[16];
    RingBufferSink sink(buf, sizeof(buf));
    uint8_t chunk[10] = {0};
    sink.write(chunk, 10);                 // used=10
    TEST_ASSERT_FALSE(sink.wrapped());
    sink.write(chunk, 10);                 // 20 > 16 -> 覆盖，环回
    TEST_ASSERT_TRUE(sink.wrapped());
    TEST_ASSERT_EQUAL_INT(16, (int)sink.usedBytes());  // 满后封顶
}

// write 比单次容量还大的块应被拒绝（返回 false，不写）。
void test_ring_rejects_oversize_single_write() {
    uint8_t buf[8];
    RingBufferSink sink(buf, sizeof(buf));
    uint8_t big[9] = {0};
    TEST_ASSERT_FALSE(sink.write(big, 9));
    TEST_ASSERT_EQUAL_INT(0, (int)sink.usedBytes());
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_ring_accumulates);
    RUN_TEST(test_ring_overwrites_when_full);
    RUN_TEST(test_ring_rejects_oversize_single_write);
    return UNITY_END();
}
