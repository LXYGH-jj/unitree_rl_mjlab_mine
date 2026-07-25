#include "FSM/CtrlFSM.h"
#include "FSM/State_Passive.h"
#include "FSM/State_FixStand.h"
#include "FSM/State_RLBase.h"

#include <atomic>
#include <csignal>

#ifdef ISAACLAB_ENABLE_ROS2_DEPTH_PROVIDER
#include <rclcpp/rclcpp.hpp>
#endif

std::unique_ptr<LowCmd_t> FSMState::lowcmd = nullptr;
std::shared_ptr<LowState_t> FSMState::lowstate = nullptr;
std::shared_ptr<Keyboard> FSMState::keyboard = nullptr;
std::shared_ptr<SportModeState_t> g_go2_depth_sport_state = nullptr;

namespace
{
std::atomic_bool running{true};

void handle_signal(int)
{
    running.store(false);
}
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
    g_go2_depth_sport_state = std::make_shared<SportModeState_t>();
    FSMState::keyboard = std::make_shared<Keyboard>();
    spdlog::info("Waiting for connection to robot...");
    FSMState::lowstate->wait_for_connection();
    spdlog::info("Connected to robot.");
}

int main(int argc, char** argv)
{
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    // Load parameters
    auto vm = param::helper(argc, argv);

    std::cout << " --- Unitree Robotics --- \n";
    std::cout << "     Go2 Depth Student Controller \n";

    // Unitree DDS Config
    unitree::robot::ChannelFactory::Instance()->Init(0, vm["network"].as<std::string>());

    init_fsm_state();

    // Initialize FSM
    auto fsm = std::make_unique<CtrlFSM>(param::config["FSM"]);
    fsm->start();

    std::cout << "Press [L2 + up] to enter FixStand mode.\n";
    std::cout << "And then press [R2 + A] to start the depth-student policy.\n";

    while (running.load())
    {
        sleep(1);
    }

#ifdef ISAACLAB_ENABLE_ROS2_DEPTH_PROVIDER
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
#endif
    
    return 0;
}
