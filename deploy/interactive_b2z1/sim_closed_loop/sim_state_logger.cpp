// M1.5m read-only *simulation* telemetry logger.
// Hard-coded DDS domain 0 / network "lo". NEVER sends LowCmd or SportClient RPCs.
// The SIM bridge publishes imu-site frame_pos/frame_vel in rt/sportmodestate.
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/idl/go2/LowState_.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

namespace {
using Clock = std::chrono::steady_clock;
std::atomic<bool> g_running{true};
void stop(int) { g_running.store(false); }
struct Snapshot {
    std::array<float,3> pos{}, vel{};
    std::uint32_t tick_ms{0};
    bool have_sport{false}, have_low{false};
    Clock::time_point sport_rx{}, low_rx{};
};
}

int main(int argc, char** argv) {
    try {
        if (argc != 3 || std::string(argv[1]).empty()) {
            std::cerr << "Usage: b2z1_sim_state_logger <output.csv> <wall_seconds:1..180>\n";
            return 2;
        }
        const int wall_seconds = std::stoi(argv[2]);
        if (wall_seconds<1 || wall_seconds>180) throw std::invalid_argument("wall_seconds must be 1..180");
        std::filesystem::path path(argv[1]);
        if (!path.parent_path().empty()) std::filesystem::create_directories(path.parent_path());
        std::ofstream csv(path, std::ios::trunc);
        if (!csv) throw std::runtime_error("cannot open CSV output");
        csv << "sim_t_s,x_world_m,y_world_m,z_world_m,"
               "vx_world_m_s,vy_world_m_s,vz_world_m_s\n";
        csv << std::fixed << std::setprecision(6);
        std::signal(SIGINT,stop);
        std::signal(SIGTERM,stop);
        unitree::robot::ChannelFactory::Instance()->Init(0,"lo");
        std::mutex mu;
        Snapshot snapshot;
        // Exactly the SDK2 topic and message type used by simulate/src/unitree_sdk2_bridge.h.
        unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_> sport("rt/sportmodestate");
        sport.InitChannel([&](const void* data) {
            if (!data) return;
            const auto& msg=*static_cast<const unitree_go::msg::dds_::SportModeState_*>(data);
            std::lock_guard<std::mutex> lock(mu);
            for(int i=0;i<3;++i){
                snapshot.pos[i]=msg.position()[i];
                snapshot.vel[i]=msg.velocity()[i];
            }
            snapshot.have_sport=true;
            snapshot.sport_rx=Clock::now();
        },1);
        unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_> low("rt/lowstate");
        low.InitChannel([&](const void* data) {
            if (!data) return;
            const auto& msg=*static_cast<const unitree_go::msg::dds_::LowState_*>(data);
            std::lock_guard<std::mutex> lock(mu);
            snapshot.tick_ms=msg.tick();
            snapshot.have_low=true;
            snapshot.low_rx=Clock::now();
        },1);
        std::cerr << "[state] SIM read-only SDK2 on lo, domain 0.\n"
                     "[state] Listening rt/sportmodestate and rt/lowstate.\n"
                     "[state] Start logger BEFORE unpausing MuJoCo; Ctrl+C to finish.\n";
        const auto deadline=Clock::now()+std::chrono::seconds(wall_seconds);
        auto next=Clock::now();
        std::uint32_t last_tick=0;
        bool wrote=false;
        std::size_t count=0;
        bool announced=false;
        while(g_running.load() && Clock::now()<deadline) {
            Snapshot s;
            {std::lock_guard<std::mutex> lock(mu);s=snapshot;}
            const auto now=Clock::now();
            const bool fresh = s.have_sport && s.have_low &&
                now-s.sport_rx<std::chrono::milliseconds(250) &&
                now-s.low_rx<std::chrono::milliseconds(250);
            if(fresh && (!wrote || s.tick_ms!=last_tick)) {
                if(s.tick_ms < last_tick && wrote) {
                    throw std::runtime_error("simulation tick reset; restart logger for a new episode");
                }
                if (!std::isfinite(s.pos[0]) || !std::isfinite(s.pos[1]) ||
                    !std::isfinite(s.pos[2]) || !std::isfinite(s.vel[0]) ||
                    !std::isfinite(s.vel[1]) || !std::isfinite(s.vel[2])) {
                    throw std::runtime_error("non-finite highstate telemetry");
                }
                csv << s.tick_ms/1000.0;
                for(int i=0;i<3;++i) csv << ',' << s.pos[i];
                for(int i=0;i<3;++i) csv << ',' << s.vel[i];
                csv << '\n';
                last_tick=s.tick_ms;
                wrote=true;
                ++count;
                if (!announced) {
                    std::cerr << "[state] SportModeState + LowState received; recording sim time and pose.\n";
                    announced=true;
                }
                if(count%20==0){
                    csv.flush();
                    std::cerr << "[state] sim=" << s.tick_ms/1000.0 << "s x=" << s.pos[0]
                              << " y=" << s.pos[1] << " z=" << s.pos[2] << '\n';
                }
            }
            next+=std::chrono::milliseconds(50); // read-only logging at approx 20Hz wall time
            std::this_thread::sleep_until(next);
            if(Clock::now()>next+std::chrono::milliseconds(250)) next=Clock::now();
        }
        csv.flush();
        std::cerr << "[state] saved " << count << " rows -> " << path.string() << '\n';
        if(!wrote){
            std::cerr << "[state] No fresh samples. Check MuJoCo simulator is active on lo/domain0.\n";
            return 1;
        }
        return 0;
    } catch(const std::exception& e) {
        std::cerr << "[state] STOP: " << e.what() << '\n';
        return 1;
    }
}
