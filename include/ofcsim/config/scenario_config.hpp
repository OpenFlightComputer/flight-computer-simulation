#pragma once

#include "ofcsim/rigid_body.hpp"

#include <filesystem>
#include <string>

#include <nlohmann/json_fwd.hpp>

namespace ofcsim {

struct ScenarioConfig {
    int schema_version = 0;
    std::string name;
    std::filesystem::path vehicle_path;
    std::filesystem::path flight_profile_path;
    double duration_s = 0.0;
    double physics_step_s = 0.0;
    double ground_z_m = 0.0;
    RigidBodyState initial_state{};
};

void from_json(
    const nlohmann::json& document,
    ScenarioConfig& config);

[[nodiscard]] ScenarioConfig load_scenario(
    const std::filesystem::path& path);

[[nodiscard]] bool validate_scenario(const ScenarioConfig& config);

}  // namespace ofcsim
