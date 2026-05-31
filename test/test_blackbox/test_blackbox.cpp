#include "unity.h"
#include "blackbox/blackbox_sink.h"
#include "blackbox/blackbox.h"
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

// 环回后底层字节摆放正确：cap=8，先写 8 个 0x11(填满 head 回 0)，再写 3 个 0x22。
// 3 个 0x22 应覆盖 buf[0..2]，buf[3..7] 仍为 0x11（验证 head 取模与覆盖位置无误）。
void test_ring_wrap_byte_placement() {
    uint8_t buf[8];
    RingBufferSink sink(buf, sizeof(buf));
    uint8_t a[8]; for (int i = 0; i < 8; ++i) a[i] = 0x11;
    sink.write(a, 8);                      // head 0->...->0（环回），used=8
    uint8_t b[3] = {0x22, 0x22, 0x22};
    sink.write(b, 3);                      // 覆盖 buf[0..2]
    TEST_ASSERT_EQUAL_UINT8(0x22, buf[0]);
    TEST_ASSERT_EQUAL_UINT8(0x22, buf[1]);
    TEST_ASSERT_EQUAL_UINT8(0x22, buf[2]);
    TEST_ASSERT_EQUAL_UINT8(0x11, buf[3]);
    TEST_ASSERT_EQUAL_UINT8(0x11, buf[7]);
    TEST_ASSERT_EQUAL_INT(8, (int)sink.usedBytes());
}

// write 比单次容量还大的块应被拒绝（返回 false，不写）。
void test_ring_rejects_oversize_single_write() {
    uint8_t buf[8];
    RingBufferSink sink(buf, sizeof(buf));
    uint8_t big[9] = {0};
    TEST_ASSERT_FALSE(sink.write(big, 9));
    TEST_ASSERT_EQUAL_INT(0, (int)sink.usedBytes());
}

// 关闭时不记录。
void test_disabled_records_nothing() {
    uint8_t buf[256];
    RingBufferSink sink(buf, sizeof(buf));
    Blackbox bb;
    BlackboxConfig cfg; cfg.enabled = false; cfg.decimation = 1;
    bb.begin(cfg, &sink);
    BlackboxFrame f{};
    for (int i = 0; i < 10; ++i) bb.logFrame(f);
    TEST_ASSERT_EQUAL_INT(0, (int)sink.usedBytes());
}

// 降采样 decimation=4：每 4 次 logFrame 只写 1 帧。
void test_decimation_writes_every_n() {
    uint8_t buf[4096];
    RingBufferSink sink(buf, sizeof(buf));
    Blackbox bb;
    BlackboxConfig cfg; cfg.enabled = true; cfg.decimation = 4;
    bb.begin(cfg, &sink);
    BlackboxFrame f{};
    for (int i = 0; i < 8; ++i) bb.logFrame(f);   // 8 次 -> 写 2 帧
    TEST_ASSERT_EQUAL_INT(2 * kFrameBytes, (int)sink.usedBytes());
}

// decimation=1：每次都写。
void test_decimation_one_writes_all() {
    uint8_t buf[4096];
    RingBufferSink sink(buf, sizeof(buf));
    Blackbox bb;
    BlackboxConfig cfg; cfg.enabled = true; cfg.decimation = 1;
    bb.begin(cfg, &sink);
    BlackboxFrame f{};
    for (int i = 0; i < 5; ++i) bb.logFrame(f);
    TEST_ASSERT_EQUAL_INT(5 * kFrameBytes, (int)sink.usedBytes());
    TEST_ASSERT_EQUAL_INT(5, (int)bb.framesWritten());
}

// 无 sink（begin 传 nullptr）时 logFrame 不崩溃、不计数。
void test_null_sink_safe() {
    Blackbox bb;
    BlackboxConfig cfg; cfg.enabled = true; cfg.decimation = 1;
    bb.begin(cfg, nullptr);
    BlackboxFrame f{};
    bb.logFrame(f);
    TEST_ASSERT_EQUAL_INT(0, (int)bb.framesWritten());
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_ring_accumulates);
    RUN_TEST(test_ring_overwrites_when_full);
    RUN_TEST(test_ring_wrap_byte_placement);
    RUN_TEST(test_ring_rejects_oversize_single_write);
    RUN_TEST(test_disabled_records_nothing);
    RUN_TEST(test_decimation_writes_every_n);
    RUN_TEST(test_decimation_one_writes_all);
    RUN_TEST(test_null_sink_safe);
    return UNITY_END();
}
