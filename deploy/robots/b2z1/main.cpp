#include "FSM/CtrlFSM.h"
#include "FSM/State_Passive.h"
#include "FSM/State_FixStand.h"
#include "FSM/State_RLBase.h"
#include "FSM/State_SlowLieDown.h"

std::unique_ptr<LowCmd_t> FSMState::lowcmd = nullptr;
std::shared_ptr<LowState_t> FSMState::lowstate = nullptr;
std::shared_ptr<Keyboard> FSMState::keyboard = nullptr;
std::shared_ptr<KeyboardJoystick> FSMState::keyboard_joystick = nullptr;
std::shared_ptr<Z1SdkController> FSMState::z1_sdk_controller = nullptr;

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
    spdlog::info("Waiting for connection to robot...");
    FSMState::lowstate->wait_for_connection();
    spdlog::info("Connected to robot.");
}

int main(int argc, char** argv)
{
    // Load parameters
    auto vm = param::helper(argc, argv);

    std::cout << " --- Unitree Robotics --- \n";
    std::cout << "     B2Z1 Controller \n";

    // Unitree DDS Config
    const std::string network = vm["network"].as<std::string>();
    const bool loopback_network =
        (network == "lo" || network == "127.0.0.1" || network == "localhost");
    unitree::robot::ChannelFactory::Instance()->Init(0, network);
    if (loopback_network)
    {
        spdlog::info("Loopback network detected ({}), skip MotionSwitcher release step.", network);
    }
    else
    {
        release_mode_before_start();
    }

    init_fsm_state();

    const auto z1_sdk_cfg = param::config["z1_sdk"];
    const bool z1_sdk_enabled = z1_sdk_cfg && z1_sdk_cfg["enabled"] && z1_sdk_cfg["enabled"].as<bool>();
    const bool z1_sdk_enable_on_loopback =
        z1_sdk_cfg && z1_sdk_cfg["enable_on_loopback"] && z1_sdk_cfg["enable_on_loopback"].as<bool>();
    if (z1_sdk_enabled && (!loopback_network || z1_sdk_enable_on_loopback)) {
        FSMState::z1_sdk_controller = std::make_shared<Z1SdkController>(z1_sdk_cfg);
        if (FSMState::z1_sdk_controller->enabled()) {
            spdlog::info("Z1 SDK direct bridge enabled.");
        }
    } else if (z1_sdk_enabled && loopback_network) {
        spdlog::info("Loopback network detected, keep Z1 on simulated lowcmd path and skip direct SDK bridge.");
    }

    // Initialize FSM
    auto fsm = std::make_unique<CtrlFSM>(param::config["FSM"]);
    fsm->start();

    std::cout << "Press [L2 + Up] to enter FixStand mode.\n";
    std::cout << "And then press [R2 + A] to start controlling the robot.\n";
    std::cout << "Safety fallback: [L2 + B] -> Passive, [L2 + Down] -> SlowLieDown.\n";
    std::cout << "Keyboard: [f] FixStand, [v] Velocity, [p] Passive, "
              << "[w/s] forward/back, [a/d] strafe, [q/e] yaw, "
              << "[o] toggle arm trajectory, [l] slow lie down.\n";

    while (true)
    {
        sleep(1);
    }
    
    return 0;
}
