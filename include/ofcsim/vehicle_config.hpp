#pragma once

#include "ofcsim/rigid_body.hpp"

#include <array>
#include <filesystem>

#include <nlohmann/json_fwd.hpp>

namespace ofcsim {

struct MotorConfig {
    Vec3 position_m = Vec3::Zero();
    int spin_direction = 0;
    double thrust_coeff = 0.0;
    double torque_coeff = 0.0;
    double max_speed_rad_s = 0.0;
    double time_constant_s = 0.0;
};

using MotorArray = std::array<double, 4>;

struct VehicleConfig {
    double mass_kg = 0.0;
    Mat3 inertia_kgm2 = Mat3::Zero();
    Mat3 inertia_inv_kgm2 = Mat3::Zero();
    std::array<MotorConfig, 4> motors{};
};

void from_json(const nlohmann::json& document, VehicleConfig& config);

[[nodiscard]] VehicleConfig load_vehicle_config(
    const std::filesystem::path& path);

[[nodiscard]] bool validate_vehicle_config(const VehicleConfig& config);

}  // namespace ofcsim
