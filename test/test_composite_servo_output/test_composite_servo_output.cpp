#include "unity.h"
#include "io/composite_servo_output.h"
using namespace wp;

void setUp() {} void tearDown() {}

// 测试替身：记录写入，可配 begin 成功/失败、通道数。
class FakeOutput : public IServoOutput {
public:
    FakeOutput(int n, bool ok) : n_(n), ok_(ok) {}
    bool begin() override { begun_ = true; return ok_; }
    int  channelCount() const override { return n_; }
    void setFrequencyHz(uint16_t hz) override { freq_ = hz; }
    void writeUs(int ch, uint16_t us) override { if (ch >= 0 && ch < 32) last_[ch] = us; }
    int n_; bool ok_; bool begun_ = false; uint16_t freq_ = 0; uint16_t last_[32] = {};
};

// 两后端拼接：全局索引连续，channelCount 求和
void test_global_index_is_concatenated() {
    FakeOutput a(8, true), b(16, true);
    CompositeServoOutput c;
    TEST_ASSERT_TRUE(c.addBackend(&a));
    TEST_ASSERT_TRUE(c.addBackend(&b));
    c.begin();
    TEST_ASSERT_EQUAL_INT(24, c.channelCount());
}

// 写全局索引路由到正确后端的本地索引
void test_write_routes_to_correct_backend() {
    FakeOutput a(8, true), b(16, true);
    CompositeServoOutput c; c.addBackend(&a); c.addBackend(&b); c.begin();
    c.writeUs(0, 1111);     // 后端 a 本地 0
    c.writeUs(7, 1777);     // 后端 a 本地 7
    c.writeUs(8, 1888);     // 后端 b 本地 0
    c.writeUs(23, 1999);    // 后端 b 本地 15
    TEST_ASSERT_EQUAL_UINT16(1111, a.last_[0]);
    TEST_ASSERT_EQUAL_UINT16(1777, a.last_[7]);
    TEST_ASSERT_EQUAL_UINT16(1888, b.last_[0]);
    TEST_ASSERT_EQUAL_UINT16(1999, b.last_[15]);
}

// 后端 begin 失败：其区间仍占位（索引不错位），但写入丢弃、channelReady=false
void test_failed_backend_keeps_index_but_drops_write() {
    FakeOutput a(8, true), b(16, false);   // b 探测失败（未焊 PCA9685）
    CompositeServoOutput c; c.addBackend(&a); c.addBackend(&b); c.begin();
    TEST_ASSERT_EQUAL_INT(24, c.channelCount());   // 区间仍保留
    TEST_ASSERT_TRUE(c.channelReady(7));           // a 可用
    TEST_ASSERT_FALSE(c.channelReady(8));          // b 不可用
    c.writeUs(8, 1888);                            // 静默丢弃，不崩
    TEST_ASSERT_EQUAL_UINT16(0, b.last_[0]);       // 未写入
}

// 越界写入静默丢弃，不崩
void test_out_of_range_write_is_dropped() {
    FakeOutput a(8, true);
    CompositeServoOutput c; c.addBackend(&a); c.begin();
    c.writeUs(99, 1500);   // 越界
    c.writeUs(-1, 1500);   // 负
    TEST_ASSERT_FALSE(c.channelReady(99));
}

// 频率转发到所有后端
void test_set_frequency_forwarded() {
    FakeOutput a(8, true), b(16, true);
    CompositeServoOutput c; c.addBackend(&a); c.addBackend(&b); c.begin();
    c.setFrequencyHz(50);
    TEST_ASSERT_EQUAL_UINT16(50, a.freq_);
    TEST_ASSERT_EQUAL_UINT16(50, b.freq_);
}

// 后端注册上限：超出返回 false
void test_backend_capacity_limit() {
    FakeOutput a(1,true), b(1,true), d(1,true), e(1,true);
    CompositeServoOutput c;
    TEST_ASSERT_TRUE(c.addBackend(&a));
    TEST_ASSERT_TRUE(c.addBackend(&b));
    TEST_ASSERT_TRUE(c.addBackend(&d));
    TEST_ASSERT_FALSE(c.addBackend(&e));   // kMaxServoBackends=3
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_global_index_is_concatenated);
    RUN_TEST(test_write_routes_to_correct_backend);
    RUN_TEST(test_failed_backend_keeps_index_but_drops_write);
    RUN_TEST(test_out_of_range_write_is_dropped);
    RUN_TEST(test_set_frequency_forwarded);
    RUN_TEST(test_backend_capacity_limit);
    return UNITY_END();
}
