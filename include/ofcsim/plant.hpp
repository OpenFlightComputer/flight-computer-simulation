#pragma once

#include "ofcsim/contact.hpp"
#include "ofcsim/propulsion.hpp"

#include <cstddef>

namespace ofcsim {

// Complete physical state: rigid-body motion plus actuator state.
struct PlantState {
    RigidBodyState rigid_body{};
    MotorArray rotor_speed_rad_s{};
};

struct PlantDerivative {
    StateDerivative rigid_body{};
    MotorArray rotor_speed_rad_s2{};
};

inline PlantDerivative operator+(
    const PlantDerivative& lhs,
    const PlantDerivative& rhs)
{
    PlantDerivative result{};
    result.rigid_body = lhs.rigid_body + rhs.rigid_body;
    for (std::size_t index = 0; index < result.rotor_speed_rad_s2.size(); ++index) {
        result.rotor_speed_rad_s2[index] =
            lhs.rotor_speed_rad_s2[index] + rhs.rotor_speed_rad_s2[index];
    }
    return result;
}

inline PlantDerivative operator*(
    const PlantDerivative& value,
    double scale)
{
    PlantDerivative result{};
    result.rigid_body = value.rigid_body * scale;
    for (std::size_t index = 0; index < result.rotor_speed_rad_s2.size(); ++index) {
        result.rotor_speed_rad_s2[index] =
            value.rotor_speed_rad_s2[index] * scale;
    }
    return result;
}

inline PlantDerivative operator*(
    double scale,
    const PlantDerivative& value)
{
    return value * scale;
}

inline PlantDerivative operator/(
    const PlantDerivative& value,
    double divisor)
{
    return value * (1.0 / divisor);
}

[[nodiscard]] PlantDerivative derivative(
    const PlantState& state,
    const MotorArray& motor_commands,
    const Propulsion& propulsion,
    const VehicleConfig& vehicle,
    const RigidBodyParameters& rigid_body_parameters);

[[nodiscard]] PlantState advance(
    const PlantState& state,
    const PlantDerivative& derivative,
    double dt_s);

[[nodiscard]] PlantState rk4_step(
    const PlantState& state,
    const MotorArray& motor_commands,
    const Propulsion& propulsion,
    const VehicleConfig& vehicle,
    const RigidBodyParameters& rigid_body_parameters,
    double dt_s);

// Advance the unconstrained plant one step, then apply ground contact.
[[nodiscard]] PlantState step(
    const PlantState& state,
    const MotorArray& motor_commands,
    const Propulsion& propulsion,
    const VehicleConfig& vehicle,
    const RigidBodyParameters& rigid_body_parameters,
    const GroundContact& contact,
    double dt_s);

}  // namespace ofcsim
