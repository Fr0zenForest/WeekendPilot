#include "unity.h"
#include "nav/landing_gear.h"
using namespace wp;

void setUp() {} void tearDown() {}

static LandingGearConfig cfg() {
    LandingGearConfig c;
    c.enabled = true;
    c.stall_current_a = 2.0f;
    c.stall_debounce_ms = 80;
    c.timeout_ms = 4000;
    return c;
}

// 未使能：永远 Stop，状态不变
void test_disabled_always_stop() {
    LandingGear g; LandingGearConfig c = cfg(); c.enabled = false; g.setConfig(c);
    LandingGearInputs in; in.deploy_cmd = true; in.dt = 0.02f;
    LandingGearOutput o = g.update(in);
    TEST_ASSERT_EQUAL(GearDrive::Stop, o.drive);
    TEST_ASSERT_EQUAL(GearState::Retracted, o.state);
}

// 上电首拍不因 deploy_cmd 初值误触发：have_last_cmd 播种，需跳变沿才动
void test_no_trigger_on_first_tick() {
    LandingGear g; g.setConfig(cfg());
    LandingGearInputs in; in.deploy_cmd = true; in.dt = 0.02f;  // 上电就是 true
    LandingGearOutput o = g.update(in);
    TEST_ASSERT_EQUAL(GearState::Retracted, o.state);   // 不动
    TEST_ASSERT_EQUAL(GearDrive::Stop, o.drive);
}

// 收->放跳变沿：进入 Deploying 并输出 Deploy 驱动
void test_deploy_command_starts_deploying() {
    LandingGear g; g.setConfig(cfg());
    LandingGearInputs in; in.dt = 0.02f;
    in.deploy_cmd = false; g.update(in);     // 播种"收"
    in.deploy_cmd = true;  LandingGearOutput o = g.update(in);  // 跳变 -> 放
    TEST_ASSERT_EQUAL(GearState::Deploying, o.state);
    TEST_ASSERT_EQUAL(GearDrive::Deploy, o.drive);
}

// Deploying 中电流超阈持续够久（去抖）-> Deployed + Stop
void test_stall_debounce_reaches_deployed() {
    LandingGear g; g.setConfig(cfg());
    LandingGearInputs in; in.dt = 0.02f;
    in.deploy_cmd = false; g.update(in);
    in.deploy_cmd = true;  g.update(in);     // Deploying
    in.current_a = 3.0f;                      // 超阈 2.0
    // 80ms 去抖 @20ms/拍 = 需 >=4 拍持续超阈
    LandingGearOutput o;
    for (int i = 0; i < 5; ++i) o = g.update(in);
    TEST_ASSERT_EQUAL(GearState::Deployed, o.state);
    TEST_ASSERT_EQUAL(GearDrive::Stop, o.drive);
}

// 电流抖动（未连续超阈去抖时长）不应误判到位
void test_current_glitch_does_not_latch() {
    LandingGear g; g.setConfig(cfg());
    LandingGearInputs in; in.dt = 0.02f;
    in.deploy_cmd = false; g.update(in);
    in.deploy_cmd = true;  g.update(in);     // Deploying
    in.current_a = 3.0f; g.update(in);        // 1 拍超阈(20ms)
    in.current_a = 0.5f;                      // 回落
    LandingGearOutput o = g.update(in);
    TEST_ASSERT_EQUAL(GearState::Deploying, o.state);   // 未到 80ms，仍在 Deploying
}

// 硬件 ALERT 为快速路径：alert=true 等价电流超阈累计去抖
void test_alert_flag_counts_as_overcurrent() {
    LandingGear g; g.setConfig(cfg());
    LandingGearInputs in; in.dt = 0.02f;
    in.deploy_cmd = false; g.update(in);
    in.deploy_cmd = true;  g.update(in);
    in.current_a = 0.0f; in.alert = true;     // 电流读数低但 ALERT 触发
    LandingGearOutput o;
    for (int i = 0; i < 5; ++i) o = g.update(in);
    TEST_ASSERT_EQUAL(GearState::Deployed, o.state);
}

// Deployed 后收到"收"跳变 -> Retracting -> 堵转 -> Retracted
void test_retract_cycle() {
    LandingGear g; g.setConfig(cfg());
    LandingGearInputs in; in.dt = 0.02f;
    in.deploy_cmd = false; g.update(in);
    in.deploy_cmd = true;  g.update(in);
    in.current_a = 3.0f; for (int i=0;i<5;++i) g.update(in);   // -> Deployed
    in.current_a = 0.0f; in.deploy_cmd = false;                 // 跳变"收"
    LandingGearOutput o = g.update(in);
    TEST_ASSERT_EQUAL(GearState::Retracting, o.state);
    TEST_ASSERT_EQUAL(GearDrive::Retract, o.drive);
    in.current_a = 3.0f; for (int i=0;i<5;++i) o = g.update(in);   // 堵转
    TEST_ASSERT_EQUAL(GearState::Retracted, o.state);
    TEST_ASSERT_EQUAL(GearDrive::Stop, o.drive);
}

// 行程超时无堵转 -> Fault + Stop（防烧）
void test_timeout_enters_fault() {
    LandingGear g; g.setConfig(cfg());   // timeout 4000ms
    LandingGearInputs in; in.dt = 0.02f;
    in.deploy_cmd = false; g.update(in);
    in.deploy_cmd = true;  g.update(in);     // Deploying
    in.current_a = 0.0f;                      // 永不堵转
    LandingGearOutput o;
    for (int i = 0; i < 205; ++i) o = g.update(in);   // 205*20ms=4100ms > 4000
    TEST_ASSERT_EQUAL(GearState::Fault, o.state);
    TEST_ASSERT_EQUAL(GearDrive::Stop, o.drive);
}

// initState 播种：上电恢复 Deployed，不主动驱动
void test_init_state_no_drive() {
    LandingGear g; g.setConfig(cfg());
    g.initState(GearState::Deployed);
    LandingGearInputs in; in.deploy_cmd = true; in.dt = 0.02f;   // 指令与状态一致
    LandingGearOutput o = g.update(in);
    TEST_ASSERT_EQUAL(GearState::Deployed, o.state);
    TEST_ASSERT_EQUAL(GearDrive::Stop, o.drive);   // 不动
}

// 链路丢失不在行程中反转：Deploying 中途丢链 -> 仍 Deploying（不退到 Retracting）
void test_link_loss_does_not_reverse_mid_travel() {
    LandingGear g; g.setConfig(cfg());
    LandingGearInputs in; in.dt = 0.02f;
    in.deploy_cmd = false; in.link_ok = true; g.update(in);   // 播种"收"
    in.deploy_cmd = true;  g.update(in);                       // -> Deploying
    // 链路丢失，遥控值此刻可能任意（模拟收到 deploy_cmd=false 的杂值）
    in.link_ok = false; in.deploy_cmd = false; in.current_a = 0.0f;
    LandingGearOutput o = g.update(in);
    TEST_ASSERT_EQUAL(GearState::Deploying, o.state);   // 不反转
    TEST_ASSERT_EQUAL(GearDrive::Deploy, o.drive);
}

// 链路丢失不重新驱动已 Fault 的执行机构
void test_link_loss_does_not_restart_fault() {
    LandingGear g; g.setConfig(cfg());
    LandingGearInputs in; in.dt = 0.02f;
    in.deploy_cmd = false; in.link_ok = true; g.update(in);
    in.deploy_cmd = true;  g.update(in);                       // Deploying
    in.current_a = 0.0f;
    for (int i = 0; i < 205; ++i) g.update(in);                // 超时 -> Fault
    // 链路丢失 + 杂值 deploy_cmd=false
    in.link_ok = false; in.deploy_cmd = false;
    LandingGearOutput o = g.update(in);
    TEST_ASSERT_EQUAL(GearState::Fault, o.state);   // 仍 Fault
    TEST_ASSERT_EQUAL(GearDrive::Stop, o.drive);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_disabled_always_stop);
    RUN_TEST(test_no_trigger_on_first_tick);
    RUN_TEST(test_deploy_command_starts_deploying);
    RUN_TEST(test_stall_debounce_reaches_deployed);
    RUN_TEST(test_current_glitch_does_not_latch);
    RUN_TEST(test_alert_flag_counts_as_overcurrent);
    RUN_TEST(test_retract_cycle);
    RUN_TEST(test_timeout_enters_fault);
    RUN_TEST(test_init_state_no_drive);
    RUN_TEST(test_link_loss_does_not_reverse_mid_travel);
    RUN_TEST(test_link_loss_does_not_restart_fault);
    return UNITY_END();
}
