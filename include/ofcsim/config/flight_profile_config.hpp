#pragma once

#include "ofcsim/config/vehicle_config.hpp"

#include <array>
#include <string>
#include <vector>

#include <nlohmann/json_fwd.hpp>

namespace ofcsim {

struct FlightProfileMetadata {
    int schema_version = 0;
    std::string name;
    std::string type;
};

struct KeyframeConfig {
    double time_s = 0.0;
    MotorArray motor_commands{};
};

struct KeyframeProfileConfig {
    FlightProfileMetadata metadata;
    std::string interpolation;
    std::vector<KeyframeConfig> keyframes;
};

[[nodiscard]] FlightProfileMetadata metadata_from_json(
    const nlohmann::json& document);

[[nodiscard]] bool validate_profile_metadata(
    const FlightProfileMetadata& metadata);

}  // namespace ofcsim
