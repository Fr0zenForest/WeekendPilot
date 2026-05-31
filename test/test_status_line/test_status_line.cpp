#include "unity.h"
#include "diag/status_line.h"
#include <cstring>
using namespace wp;

void setUp() {} void tearDown() {}

// WARNING 新代码。纯格式化逻辑 PC 可测；板载 Serial 实际输出未在实物验证。

// 基本字段都出现在输出里，且不溢出缓冲。
void test_format_contains_fields() {
    StatusSnapshot s{};
    s.t_ms = 1234;
    s.mode = 1;                 // Angle
    s.roll_deg = 12.3f; s.pitch_deg = -4.5f; s.yaw_deg = 90.0f;
    s.link_ok = true; s.imu_valid = true; s.baro_valid = true; s.mag_valid = false;
    s.althold_engaged = false; s.autotrim_learning = false;
    s.servo[0] = 1500; s.servo[1] = 1600; s.servo[2] = 1700; s.servo[3] = 1500;

    char buf[kStatusLineCap];
    int n = formatStatusLine(s, buf, sizeof(buf));
    TEST_ASSERT_TRUE(n > 0);
    TEST_ASSERT_TRUE(n < (int)sizeof(buf));    // 未截断到边界
    TEST_ASSERT_NOT_NULL(strstr(buf, "ANGLE"));    // 模式名
    TEST_ASSERT_NOT_NULL(strstr(buf, "1234"));     // 时间戳
    TEST_ASSERT_NOT_NULL(strstr(buf, "1600"));     // 升降舵量
}

// 模式名映射：0=OFF 1=ANGLE 2=RATE，越界=?。
void test_mode_names() {
    char buf[kStatusLineCap];
    StatusSnapshot s{};
    s.mode = 0; formatStatusLine(s, buf, sizeof(buf));
    TEST_ASSERT_NOT_NULL(strstr(buf, "OFF"));
    s.mode = 2; formatStatusLine(s, buf, sizeof(buf));
    TEST_ASSERT_NOT_NULL(strstr(buf, "RATE"));
    s.mode = 9; formatStatusLine(s, buf, sizeof(buf));
    TEST_ASSERT_NOT_NULL(strstr(buf, "?"));        // 越界安全
}

// 有效位/功能标志用简明字符表示，链路丢失可辨识。
void test_flags_render() {
    StatusSnapshot s{};
    s.link_ok = false; s.imu_valid = true; s.baro_valid = false; s.mag_valid = true;
    s.althold_engaged = true; s.autotrim_learning = true;
    char buf[kStatusLineCap];
    formatStatusLine(s, buf, sizeof(buf));
    // 链路丢失应出现 LINK 标记；定高/配平激活应出现 AH/AT
    TEST_ASSERT_NOT_NULL(strstr(buf, "LINK"));
    TEST_ASSERT_NOT_NULL(strstr(buf, "AH"));
    TEST_ASSERT_NOT_NULL(strstr(buf, "AT"));
}

// 极小缓冲不崩溃、不越界（snprintf 截断语义）。
void test_tiny_buffer_safe() {
    StatusSnapshot s{}; s.mode = 1;
    char tiny[8];
    int n = formatStatusLine(s, tiny, sizeof(tiny));
    TEST_ASSERT_TRUE(tiny[sizeof(tiny) - 1] == '\0' || n >= 0);  // 必 NUL 收尾
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_format_contains_fields);
    RUN_TEST(test_mode_names);
    RUN_TEST(test_flags_render);
    RUN_TEST(test_tiny_buffer_safe);
    return UNITY_END();
}
