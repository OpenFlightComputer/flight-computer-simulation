#include "ofcsim/config/scenario_config.hpp"

#include <cmath>
#include <fstream>
#include <stdexcept>

#include <nlohmann/json.hpp>

namespace ofcsim {
namespace {
Vec3 read_vec3(const nlohmann::json& document, const char* field)
{
    const auto& values = document.at(field);
    if (!values.is_array() || values.size() != 3) {
        throw std::invalid_argument(
            std::string("scenario field must contain three values: ") + field);
    }
    return Vec3(values.at(0).get<double>(), values.at(1).get<double>(),
                values.at(2).get<double>());
}
}  // namespace

void from_json(const nlohmann::json& document, ScenarioConfig& config)
{
    document.at("schema_version").get_to(config.schema_version);
    document.at("name").get_to(config.name);
    config.vehicle_path = document.at("vehicle").get<std::string>();
    config.flight_profile_path = document.at("flight_profile").get<std::string>();
    document.at("duration_s").get_to(config.duration_s);
    document.at("physics_step_s").get_to(config.physics_step_s);
    const auto& ground = document.at("ground");
    ground.at("ground_z_m").get_to(config.ground_z_m);
    const auto& initial = document.at("initial_state");
    config.initial_state.position_m = read_vec3(initial, "position_m");
    config.initial_state.velocity_mps = read_vec3(initial, "velocity_mps");
    config.initial_state.rate_rad_s = read_vec3(initial, "rate_rad_s");
    config.initial_state.attitude = quat_from_euler_zyx_deg(
        read_vec3(initial, "euler_deg"));
}

ScenarioConfig load_scenario(const std::filesystem::path& path)
{
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("could not open scenario: " + path.string());
    }
    ScenarioConfig config = nlohmann::json::parse(input).get<ScenarioConfig>();
    if (!validate_scenario(config)) {
        throw std::invalid_argument("scenario failed validation");
    }
    return config;
}

bool validate_scenario(const ScenarioConfig& config)
{
    return config.schema_version == 1 && !config.name.empty() &&
        !config.vehicle_path.empty() && !config.flight_profile_path.empty() &&
        std::isfinite(config.duration_s) && config.duration_s > 0.0 &&
        std::isfinite(config.physics_step_s) && config.physics_step_s > 0.0 &&
        std::isfinite(config.ground_z_m) && config.initial_state.position_m.allFinite() &&
        config.initial_state.velocity_mps.allFinite() &&
        config.initial_state.attitude.coeffs().allFinite() &&
        config.initial_state.rate_rad_s.allFinite();
}

}  // namespace ofcsim
