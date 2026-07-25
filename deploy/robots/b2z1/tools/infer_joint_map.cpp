#include "Types.h"
#include "param.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unistd.h>
#include <vector>

struct CandidateResult
{
    std::array<std::string, 4> sdk_leg_order;
    std::vector<int> policy_to_sdk_map;
    std::vector<float> expected_sdk_q;
    float leg_rmse = 0.0f;
};

static std::vector<int> build_policy_to_sdk_map(const std::array<std::string, 4>& sdk_leg_order)
{
    static const std::array<std::string, 4> policy_leg_order = {"FL", "FR", "RL", "RR"};
    std::vector<int> map;
    map.reserve(18);

    for (const auto& leg_name : policy_leg_order) {
        int sdk_group = -1;
        for (int g = 0; g < 4; ++g) {
            if (sdk_leg_order[g] == leg_name) {
                sdk_group = g;
                break;
            }
        }
        if (sdk_group < 0) {
            throw std::runtime_error("Invalid leg order candidate");
        }
        map.push_back(3 * sdk_group + 0);
        map.push_back(3 * sdk_group + 1);
        map.push_back(3 * sdk_group + 2);
    }

    // Arm joint1~joint6 are typically contiguous after leg joints on B2Z1.
    for (int i = 12; i < 18; ++i) {
        map.push_back(i);
    }
    return map;
}

static std::vector<float> build_expected_sdk_q(const std::array<std::string, 4>& sdk_leg_order)
{
    // Same nominal pose used in deploy config.
    static const std::map<std::string, std::array<float, 3>> kLegNominal = {
        {"FL", { 0.1f, 0.8f, -1.5f}},
        {"FR", {-0.1f, 0.8f, -1.5f}},
        {"RL", { 0.1f, 1.0f, -1.5f}},
        {"RR", {-0.1f, 1.0f, -1.5f}},
    };
    static const std::array<float, 6> kArmNominal = {0.0f, 0.5f, -1.0f, 0.0f, 0.0f, 0.0f};

    std::vector<float> q(18, 0.0f);
    for (int g = 0; g < 4; ++g) {
        const auto it = kLegNominal.find(sdk_leg_order[g]);
        if (it == kLegNominal.end()) {
            throw std::runtime_error("Unknown leg name in sdk leg order");
        }
        q[3 * g + 0] = it->second[0];
        q[3 * g + 1] = it->second[1];
        q[3 * g + 2] = it->second[2];
    }
    for (int i = 0; i < 6; ++i) {
        q[12 + i] = kArmNominal[i];
    }
    return q;
}

static float calc_leg_rmse(const std::vector<float>& observed_sdk_q, const std::vector<float>& expected_sdk_q)
{
    float mse = 0.0f;
    for (int i = 0; i < 12; ++i) {
        const float d = observed_sdk_q[i] - expected_sdk_q[i];
        mse += d * d;
    }
    mse /= 12.0f;
    return std::sqrt(mse);
}

static std::string fmt_int_list(const std::vector<int>& vals)
{
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < vals.size(); ++i) {
        oss << vals[i];
        if (i + 1 < vals.size()) oss << ",";
    }
    oss << "]";
    return oss.str();
}

static std::string fmt_float_list(const std::vector<float>& vals)
{
    std::ostringstream oss;
    oss << "[";
    oss << std::fixed << std::setprecision(3);
    for (size_t i = 0; i < vals.size(); ++i) {
        oss << vals[i];
        if (i + 1 < vals.size()) oss << ",";
    }
    oss << "]";
    return oss.str();
}

int main(int argc, char** argv)
{
    auto vm = param::helper(argc, argv);
    unitree::robot::ChannelFactory::Instance()->Init(0, vm["network"].as<std::string>());

    auto lowstate = std::make_shared<LowState_t>();
    spdlog::info("Waiting for lowstate connection...");
    lowstate->wait_for_connection();
    spdlog::info("Connected. Sampling motor_state for map inference...");

    constexpr int kSampleCount = 800;
    constexpr int kJointCount = 18;

    std::vector<double> q_sum(kJointCount, 0.0);
    std::vector<double> q_sq_sum(kJointCount, 0.0);

    for (int n = 0; n < kSampleCount; ++n) {
        lowstate->update();
        {
            std::lock_guard<std::mutex> lock(lowstate->mutex_);
            for (int j = 0; j < kJointCount; ++j) {
                const double q = lowstate->msg_.motor_state()[j].q();
                q_sum[j] += q;
                q_sq_sum[j] += q * q;
            }
        }
        usleep(2000);
    }

    std::vector<float> q_mean(kJointCount, 0.0f);
    std::vector<float> q_std(kJointCount, 0.0f);
    for (int j = 0; j < kJointCount; ++j) {
        const double mean = q_sum[j] / static_cast<double>(kSampleCount);
        const double second = q_sq_sum[j] / static_cast<double>(kSampleCount);
        const double var = std::max(0.0, second - mean * mean);
        q_mean[j] = static_cast<float>(mean);
        q_std[j] = static_cast<float>(std::sqrt(var));
    }

    std::cout << "\nObserved mean joint q (sdk index order):\n";
    std::cout << std::fixed << std::setprecision(4);
    for (int j = 0; j < kJointCount; ++j) {
        std::cout << "  idx " << std::setw(2) << j
                  << "  q=" << std::setw(8) << q_mean[j]
                  << "  std=" << std::setw(8) << q_std[j] << "\n";
    }

    std::array<std::string, 4> order = {"FL", "FR", "RL", "RR"};
    std::vector<CandidateResult> results;

    std::sort(order.begin(), order.end());
    do {
        CandidateResult r;
        r.sdk_leg_order = order;
        r.policy_to_sdk_map = build_policy_to_sdk_map(order);
        r.expected_sdk_q = build_expected_sdk_q(order);
        r.leg_rmse = calc_leg_rmse(q_mean, r.expected_sdk_q);
        results.push_back(std::move(r));
    } while (std::next_permutation(order.begin(), order.end()));

    std::sort(results.begin(), results.end(), [](const CandidateResult& a, const CandidateResult& b) {
        return a.leg_rmse < b.leg_rmse;
    });

    std::cout << "\nTop leg-order candidates (lower leg RMSE is better):\n";
    const int top_k = std::min<int>(5, static_cast<int>(results.size()));
    for (int i = 0; i < top_k; ++i) {
        const auto& r = results[i];
        std::cout << "  #" << (i + 1) << " sdk_leg_order=["
                  << r.sdk_leg_order[0] << "," << r.sdk_leg_order[1] << ","
                  << r.sdk_leg_order[2] << "," << r.sdk_leg_order[3] << "]"
                  << "  leg_rmse=" << std::setprecision(5) << r.leg_rmse << "\n";
    }

    const auto& best = results.front();
    std::cout << "\nRecommended deploy snippet:\n";
    std::cout << "joint_ids_map: " << fmt_int_list(best.policy_to_sdk_map) << "\n";
    std::cout << "FixStand qs[1]: " << fmt_float_list(best.expected_sdk_q) << "\n";
    std::cout << "\nNote: Keep robot safely suspended during this tool run.\n";

    return 0;
}
