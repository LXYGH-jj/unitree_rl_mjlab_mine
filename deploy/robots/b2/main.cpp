#include "FSM/CtrlFSM.h"
#include "FSM/State_Passive.h"
#include "FSM/State_FixStand.h"
#include "FSM/State_LieDown.h"
#include "FSM/State_RLBase.h"
#include "B2Runtime.h"

std::unique_ptr<LowCmd_t> FSMState::lowcmd = nullptr;
std::shared_ptr<LowState_t> FSMState::lowstate = nullptr;
std::shared_ptr<Keyboard> FSMState::keyboard = nullptr;
std::shared_ptr<KeyboardJoystick> FSMState::keyboard_joystick = nullptr;
SportModeState_t::SharedPtr g_b2_sport_state = nullptr;

void release_mode_before_start(int max_attempts = 5)
{
    auto msc = std::make_shared<unitree::robot::b2::MotionSwitcherClient>();
    msc->SetTimeout(3.0f);
    msc->Init();

    for (int i = 1; i <= max_attempts; ++i)
    {
        std::string form, name;
        const int32_t check_ret = msc->CheckMode(form, name);
        if (check_ret != 0)
        {
            spdlog::warn("CheckMode failed before start. ret={}", check_ret);
            sleep(1);
            continue;
        }
        if (name.empty())
        {
            spdlog::info("Motion switcher is already in release mode.");
            return;
        }

        spdlog::info("Active motion mode '{}' found. Sending ReleaseMode ({}/{})...", name, i, max_attempts);
        const int32_t release_ret = msc->ReleaseMode();
        if (release_ret != 0)
        {
            spdlog::warn("ReleaseMode failed. ret={}", release_ret);
        }
        sleep(1);
    }
    spdlog::warn("ReleaseMode attempts finished. Continuing with low-level startup.");
}

void init_fsm_state()
{
    auto lowcmd_sub = std::make_shared<unitree::robot::go2::subscription::LowCmd>();
    usleep(0.2 * 1e6);
    if(!lowcmd_sub->isTimeout())
    {
        spdlog::critical("The other process is using the lowcmd channel, please close it first.");
        unitree::robot::go2::shutdown();
        // exit(0);
    }
    FSMState::lowcmd = std::make_unique<LowCmd_t>();
    FSMState::lowstate = std::make_shared<LowState_t>();
    FSMState::keyboard = std::make_shared<Keyboard>();
    FSMState::keyboard_joystick = std::make_shared<KeyboardJoystick>(FSMState::keyboard);
    g_b2_sport_state = std::make_shared<SportModeState_t>();
    spdlog::info("Waiting for connection to robot...");
    FSMState::lowstate->wait_for_connection();
    spdlog::info("Connected to robot.");
    if (g_b2_sport_state->isTimeout()) {
        spdlog::warn("sportmodestate is not available yet, base_linear_velocity will fallback to IMU integration.");
    }
}

int main(int argc, char** argv)
{
    // Load parameters
    auto vm = param::helper(argc, argv);
    const std::string network = vm["network"].as<std::string>();

    std::cout << " --- Unitree Robotics --- \n";
    std::cout << "     B2 Controller \n";

    // Unitree DDS Config
    unitree::robot::ChannelFactory::Instance()->Init(0, network);
    if (network == "lo" || network == "127.0.0.1" || network == "localhost")
    {
        spdlog::info("Loopback network detected ({}), skip MotionSwitcher release step.", network);
    }
    else
    {
        release_mode_before_start();
    }

    init_fsm_state();

    // Initialize FSM
    auto fsm = std::make_unique<CtrlFSM>(param::config["FSM"]);
    fsm->start();

    std::cout << "Press [L2 + up] to enter FixStand mode.\n";
    std::cout << "And then press [R2 + A] to start controlling the robot.\n";
    std::cout << "Safety fallback: [R2 + B] Velocity -> LieDown, [L2 + B] -> Passive.\n";
    std::cout << "Keyboard: [f] FixStand, [v] Velocity, [p] Passive, "
              << "[w/s] forward/back, [a/d] strafe, [q/e] yaw.\n";

    while (true)
    {
        sleep(1);
    }

    
    return 0;
}
