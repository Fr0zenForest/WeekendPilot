#include "nav/landing_gear.h"

namespace wp {

void LandingGear::setConfig(const LandingGearConfig& c) { cfg_ = c; }

void LandingGear::initState(GearState s) {
    state_ = s;
    stall_timer_ms_ = 0.0f;
    travel_timer_ms_ = 0.0f;
    // 不动 last_deploy_cmd_/have_last_cmd_：上电首拍仍需播种指令、靠跳变沿才动作。
}

LandingGearOutput LandingGear::update(const LandingGearInputs& in) {
    LandingGearOutput out;
    if (!cfg_.enabled) {
        out.drive = GearDrive::Stop;
        out.state = state_;
        return out;
    }

    // 链路丢失：冻结指令沿处理 —— 不记录新指令、不触发收放、不退出 Fault。
    // 失控时维持当前动作（Deploying/Retracting 继续靠堵转自然停；稳态保持），
    // 绝不因失控反转起落架或重新驱动已 Fault 的执行机构。
    // 指令跳变沿检测：首拍只播种，不动作（防上电误触发，设计文档 §9）。
    bool edge_deploy = false, edge_retract = false;
    if (in.link_ok) {
        if (!have_last_cmd_) {
            have_last_cmd_ = true;
        } else if (in.deploy_cmd != last_deploy_cmd_) {
            edge_deploy  = in.deploy_cmd;     // false->true：放下
            edge_retract = !in.deploy_cmd;    // true->false：收起
        }
        last_deploy_cmd_ = in.deploy_cmd;
    }

    // 跳变触发状态切换（任意稳态/行程态收到反向指令都重启对应行程）。
    if (edge_deploy && state_ != GearState::Deploying) {
        state_ = GearState::Deploying;
        stall_timer_ms_ = 0.0f; travel_timer_ms_ = 0.0f;
    } else if (edge_retract && state_ != GearState::Retracting) {
        state_ = GearState::Retracting;
        stall_timer_ms_ = 0.0f; travel_timer_ms_ = 0.0f;
    }

    const float dt_ms = in.dt * 1000.0f;
    const bool overcurrent = in.alert || (in.current_a >= cfg_.stall_current_a);

    switch (state_) {
        case GearState::Deploying:
        case GearState::Retracting: {
            travel_timer_ms_ += dt_ms;
            // 启动浪涌屏蔽：行程头 inrush_mask_ms 内的电流超阈不计入堵转去抖。
            const bool past_inrush = travel_timer_ms_ > cfg_.inrush_mask_ms;
            if (overcurrent && past_inrush) stall_timer_ms_ += dt_ms;
            else                            stall_timer_ms_ = 0.0f;

            if (stall_timer_ms_ >= cfg_.stall_debounce_ms) {
                state_ = (state_ == GearState::Deploying) ? GearState::Deployed
                                                          : GearState::Retracted;
                out.drive = GearDrive::Stop;
            } else if (travel_timer_ms_ >= cfg_.timeout_ms) {
                state_ = GearState::Fault;             // 行程超时无堵转 -> 防烧
                out.drive = GearDrive::Stop;
            } else {
                out.drive = (state_ == GearState::Deploying) ? GearDrive::Deploy
                                                             : GearDrive::Retract;
            }
            break;
        }
        case GearState::Retracted:
        case GearState::Deployed:
        case GearState::Fault:
        default:
            out.drive = GearDrive::Stop;               // 稳态/故障：软件停舵
            break;
    }

    out.state = state_;
    return out;
}

}  // namespace wp
