// M1.3 offline smoke/parity test for the ACTUAL deployment OrtRunner class.
// No SDK2, Z1 SDK, DDS, FSM, or motor commands are used here.
// Usage: test_cpp_ort_runner <interactive_policy.onnx> <obs_fixture.csv>
// Input CSV lines have 800 columns: frame index followed by 799 observations.
// stdout contains exactly one CSV line per frame, with 18 raw policy actions.

#include "isaaclab/algorithms/algorithms.h"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace {
std::vector<float> parse_csv(const std::string& line, std::size_t line_no) {
    std::vector<float> values;
    std::istringstream ss(line);
    std::string token;
    while (std::getline(ss, token, ',')) {
        if (token.empty()) {
            throw std::runtime_error("Empty CSV cell at line " + std::to_string(line_no));
        }
        std::size_t parsed = 0;
        float value = std::stof(token, &parsed);
        if (parsed != token.size() || !std::isfinite(value)) {
            throw std::runtime_error("Bad CSV value at line " + std::to_string(line_no));
        }
        values.push_back(value);
    }
    return values;
}
}

int main(int argc, char** argv) {
    try {
        if (argc != 3) {
            std::cerr << "Usage: " << argv[0] << " <policy.onnx> <obs_fixture.csv>\n";
            return 2;
        }
        std::ifstream fin(argv[2]);
        if (!fin) {
            throw std::runtime_error(std::string("Cannot read fixture CSV: ") + argv[2]);
        }

        // The existing deployment class performs ONNX inference.
        // This tests its current ONNX Runtime binding, not a rewritten runner.
        isaaclab::OrtRunner runner(argv[1]);
        std::cout << std::setprecision(9);
        std::string line;
        std::size_t line_no = 0;
        while (std::getline(fin, line)) {
            ++line_no;
            if (line.empty()) continue;
            auto csv = parse_csv(line, line_no);
            if (csv.size() != 800) {
                throw std::runtime_error("Expected index + 799 observations at line " +
                                         std::to_string(line_no) + ", got " +
                                         std::to_string(csv.size()));
            }
            std::vector<float> obs(csv.begin() + 1, csv.end());
            std::unordered_map<std::string, std::vector<float>> inputs;
            inputs.emplace("obs", std::move(obs));
            auto actions = runner.act(std::move(inputs));
            if (actions.size() != 18) {
                throw std::runtime_error("Expected 18 policy actions, got " +
                                         std::to_string(actions.size()));
            }
            for (std::size_t i = 0; i < actions.size(); ++i) {
                if (!std::isfinite(actions[i])) {
                    throw std::runtime_error("Non-finite policy action at frame " +
                                             std::to_string(line_no));
                }
                if (i) std::cout << ',';
                std::cout << actions[i];
            }
            std::cout << '\n';
        }
        if (line_no == 0) throw std::runtime_error("No observations in fixture file");
        return 0;
    } catch (const Ort::Exception& e) {
        std::cerr << "ONNX Runtime error: " << e.what() << '\n';
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "M1.3 test error: " << e.what() << '\n';
        return 1;
    }
}
