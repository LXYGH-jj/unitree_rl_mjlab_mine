#include "Types.h"

#include "unitree/robot/channel/channel_factory.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <csignal>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

namespace
{
std::atomic_bool g_running{true};

void handle_signal(int)
{
    g_running.store(false);
}

struct Args
{
    std::string network;
    int period_ms = 200;
};

Args parse_args(int argc, char** argv)
{
    Args args;
    for (int i = 1; i < argc; ++i) {
        const std::string key = argv[i];
        if ((key == "--network" || key == "-n") && i + 1 < argc) {
            args.network = argv[++i];
        } else if ((key == "--period-ms" || key == "--period") && i + 1 < argc) {
            args.period_ms = std::max(10, std::atoi(argv[++i]));
        } else if (key == "--help" || key == "-h") {
            std::cout
                << "Usage: " << argv[0] << " [--network IFACE] [--period-ms N]\n"
                << "  --network, -n     DDS network interface, for example eth0 or wlan0\n"
                << "  --period-ms       Print period in milliseconds, default 200\n";
            std::exit(0);
        }
    }
    return args;
}

template <typename T, size_t N>
void print_array(std::ostream& os, const std::array<T, N>& values)
{
    os << "[";
    os << std::fixed << std::setprecision(3);
    for (size_t i = 0; i < N; ++i) {
        os << values[i];
        if (i + 1 < N) {
            os << ", ";
        }
    }
    os << "]";
}
}  // namespace

int main(int argc, char** argv)
{
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    const Args args = parse_args(argc, argv);

    unitree::robot::ChannelFactory::Instance()->Init(0, args.network);

    auto lowstate = std::make_shared<LowState_t>();
    std::cout << "Waiting for Go2 LowState..." << std::endl;
    lowstate->wait_for_connection();
    std::cout << "Connected. Printing foot forces every " << args.period_ms << " ms." << std::endl;
    std::cout << "Columns: FL FR RL RR" << std::endl;

    size_t sample_idx = 0;
    while (g_running.load()) {
        lowstate->update();

        std::array<float, 4> foot_force = {0.0f, 0.0f, 0.0f, 0.0f};
        std::array<float, 4> foot_force_est = {0.0f, 0.0f, 0.0f, 0.0f};

        {
            std::lock_guard<std::mutex> lock(lowstate->mutex_);
            for (size_t i = 0; i < 4; ++i) {
                foot_force[i] = static_cast<float>(lowstate->msg_.foot_force()[i]);
                foot_force_est[i] = static_cast<float>(lowstate->msg_.foot_force_est()[i]);
            }
        }

        const auto now = std::chrono::steady_clock::now().time_since_epoch();
        const double t_sec = std::chrono::duration<double>(now).count();

        std::cout << std::fixed << std::setprecision(3)
                  << "[" << sample_idx++ << "] "
                  << "t=" << t_sec << "s "
                  << "foot_force=";
        print_array(std::cout, foot_force);
        std::cout << " foot_force_est=";
        print_array(std::cout, foot_force_est);
        std::cout << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(args.period_ms));
    }

    std::cout << "Stopped." << std::endl;
    return 0;
}
