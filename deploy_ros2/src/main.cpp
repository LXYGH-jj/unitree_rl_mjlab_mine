#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <spdlog/spdlog.h>
#include "unitree_api/msg/request.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"

#include "FSM/CtrlFSM.h"
#include "FSM/State_FixStand.h"
#include "FSM/State_Passive.h"
#include "FSM/State_RLBase.h"
#include "param.h"

std::unique_ptr<LowCmd_t> FSMState::lowcmd = nullptr;
std::shared_ptr<LowState_t> FSMState::lowstate = nullptr;
std::shared_ptr<Keyboard> FSMState::keyboard = nullptr;

namespace
{

constexpr int32_t MOTION_SWITCHER_API_ID_RELEASE_MODE = 1003;

template <typename T>
T yaml_or(const YAML::Node& node, const std::string& key, const T& fallback)
{
    return node[key] ? node[key].as<T>() : fallback;
}

void publish_release_mode(
    const rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr& publisher,
    int attempts,
    double interval_sec)
{
    for (int i = 0; rclcpp::ok() && i < attempts; ++i)
    {
        unitree_api::msg::Request req;
        req.header.identity.id = i + 1;
        req.header.identity.api_id = MOTION_SWITCHER_API_ID_RELEASE_MODE;
        req.parameter = "";
        publisher->publish(req);
        spdlog::info("Sent MotionSwitcher ReleaseMode request ({}/{})", i + 1, attempts);
        std::this_thread::sleep_for(std::chrono::duration<double>(interval_sec));
    }
}

} // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    const auto config_file = param::helper(argc, argv);
    spdlog::info("Loaded config: {}", config_file.string());

    auto node = std::make_shared<rclcpp::Node>("go2_rl_mjlab_ros2");
    const auto ros_cfg = param::config["ROS2"];
    const auto lowstate_topic = yaml_or<std::string>(ros_cfg, "lowstate_topic", "/lowstate");
    const auto lowcmd_topic = yaml_or<std::string>(ros_cfg, "lowcmd_topic", "/lowcmd");
    const auto motion_switch_topic = yaml_or<std::string>(ros_cfg, "motion_switch_request_topic", "/api/motion_switcher/request");
    const auto lowstate_timeout_sec = yaml_or<double>(ros_cfg, "lowstate_timeout_sec", 1.0);
    const auto release_interval_sec = yaml_or<double>(ros_cfg, "release_interval_sec", 1.0);
    const auto release_max_attempts = yaml_or<int>(ros_cfg, "release_max_attempts", 5);

    auto lowstate = std::make_shared<LowState_t>();
    lowstate->set_timeout_ms(static_cast<uint32_t>(lowstate_timeout_sec * 1000.0));

    auto lowcmd_pub = node->create_publisher<unitree_go::msg::LowCmd>(lowcmd_topic, 10);
    auto motion_switch_pub = node->create_publisher<unitree_api::msg::Request>(motion_switch_topic, 10);

    auto lowstate_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    auto lowstate_sub = node->create_subscription<unitree_go::msg::LowState>(
        lowstate_topic,
        lowstate_qos,
        [lowstate](const unitree_go::msg::LowState::SharedPtr msg) {
            lowstate->on_message(msg);
        });
    (void)lowstate_sub;

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::atomic_bool spin_running{true};
    std::thread spin_thread([&] {
        while (rclcpp::ok() && spin_running)
        {
            executor.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });

    FSMState::lowstate = lowstate;
    FSMState::lowcmd = std::make_unique<LowCmd_t>(lowcmd_pub);

    spdlog::info("ROS2 Go2 controller");
    spdlog::info("Waiting for lowstate topic: {}", lowstate_topic);
    FSMState::lowstate->wait_for_connection();
    spdlog::info("Connected to lowstate.");

    publish_release_mode(motion_switch_pub, release_max_attempts, release_interval_sec);

    auto fsm = std::make_unique<CtrlFSM>(param::config["FSM"]);
    fsm->start();

    spdlog::info("Press [LT + A] to enter FixStand mode.");
    spdlog::info("Then press [RT + X] to start Velocity policy control.");
    spdlog::info("Press [LT + B] to return to Passive.");

    while (rclcpp::ok())
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    spin_running = false;
    if (spin_thread.joinable())
    {
        spin_thread.join();
    }
    executor.remove_node(node);
    rclcpp::shutdown();
    return 0;
}
