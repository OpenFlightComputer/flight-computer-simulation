#include "ofcsim/vehicle_config.hpp"

#include <cmath>
#include <fstream>
#include <stdexcept>

#include <nlohmann/json.hpp>

namespace ofcsim {
namespace {

using Json = nlohmann::json;

}  // namespace

void from_json(const Json& document, VehicleConfig& config)
{
    document.at("mass_kg").get_to(config.mass_kg);

    const Json& inertia = document.at("inertia_kgm2");
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t column = 0; column < 3; ++column) {
            config.inertia_kgm2(
                static_cast<Eigen::Index>(row),
                static_cast<Eigen::Index>(column)) =
                inertia.at(row).at(column).get<double>();
        }
    }
    const Json& defaults = document.at("motor_defaults");
    const auto& motors = document.at("motors");
    if (motors.size() != config.motors.size()) {
        throw std::invalid_argument("vehicle configuration must define four motors");
    }

    constexpr const char* expected_corners[] = {
        "front_left", "rear_left", "front_right", "rear_right"};

    for (std::size_t index = 0; index < config.motors.size(); ++index) {
        MotorConfig& motor = config.motors[index];
        const std::string corner =
            motors.at(index).at("corner").get<std::string>();
        if (corner != expected_corners[index]) {
            throw std::invalid_argument(
                "motor corner does not match its logical index: " + corner);
        }
        const Json& position = motors.at(index).at("position_m");
        if (!position.is_array() || position.size() != 3) {
            throw std::invalid_argument(
                "motor position must contain exactly three values");
        }
        motor.position_m = Vec3(
            position.at(0).get<double>(),
            position.at(1).get<double>(),
            position.at(2).get<double>());
        motor.spin_direction = motors.at(index).at("spin_direction").get<int>();
        motor.thrust_coeff = defaults.at("thrust_coeff").get<double>();
        motor.torque_coeff = defaults.at("torque_coeff").get<double>();
        motor.max_speed_rad_s = defaults.at("max_speed_rad_s").get<double>();
        motor.time_constant_s = defaults.at("time_constant_s").get<double>();
    }
}

VehicleConfig load_vehicle_config(const std::filesystem::path& path)
{
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("could not open vehicle configuration: " +
                                 path.string());
    }

    VehicleConfig config = Json::parse(input).get<VehicleConfig>();
    config.inertia_inv_kgm2 = config.inertia_kgm2.inverse();

    if (!validate_vehicle_config(config)) {
        throw std::invalid_argument("vehicle configuration failed validation");
    }
    return config;
}

bool validate_vehicle_config(const VehicleConfig& config)
{
    const Eigen::LLT<Mat3> inertia_factorization(config.inertia_kgm2);
    const bool inertia_is_valid =
        config.inertia_kgm2.allFinite() &&
        config.inertia_kgm2.isApprox(
            config.inertia_kgm2.transpose(), 1.0e-12) &&
        inertia_factorization.info() == Eigen::Success;
    const bool inverse_is_valid =
        config.inertia_inv_kgm2.allFinite() &&
        (config.inertia_kgm2 * config.inertia_inv_kgm2).isApprox(
            Mat3::Identity(), 1.0e-12);

    if (!std::isfinite(config.mass_kg) || config.mass_kg <= 0.0 ||
        !inertia_is_valid || !inverse_is_valid) {
        return false;
    }

    for (std::size_t index = 0; index < config.motors.size(); ++index) {
        const MotorConfig& motor = config.motors[index];
        if ((motor.spin_direction != 1 && motor.spin_direction != -1) ||
            !motor.position_m.allFinite() ||
            !std::isfinite(motor.thrust_coeff) || motor.thrust_coeff <= 0.0 ||
            !std::isfinite(motor.torque_coeff) || motor.torque_coeff <= 0.0 ||
            !std::isfinite(motor.max_speed_rad_s) || motor.max_speed_rad_s <= 0.0 ||
            !std::isfinite(motor.time_constant_s) || motor.time_constant_s <= 0.0) {
            return false;
        }
        for (std::size_t other = 0; other < index; ++other) {
            if ((motor.position_m - config.motors[other].position_m).norm() <
                1.0e-12) {
                return false;
            }
        }
    }
    return true;
}

}  // namespace ofcsim
