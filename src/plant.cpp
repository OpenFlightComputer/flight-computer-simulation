#include "ofcsim/plant.hpp"

namespace ofcsim {

PlantDerivative derivative(
    const PlantState& state,
    const MotorArray& motor_commands,
    const Propulsion& propulsion,
    const VehicleConfig& vehicle,
    const RigidBodyParameters& rigid_body_parameters)
{
    PlantDerivative result{};

    for (std::size_t index = 0; index < state.rotor_speed_rad_s.size(); ++index) {
        const double target_speed =
            propulsion.commanded_speed_rad_s(index, motor_commands.at(index));
        result.rotor_speed_rad_s2[index] =
            (target_speed - state.rotor_speed_rad_s[index]) /
            vehicle.motors.at(index).time_constant_s;
    }

    const BodyWrench wrench = propulsion.wrench(state.rotor_speed_rad_s);
    result.rigid_body = ofcsim::derivative(
        state.rigid_body, wrench, rigid_body_parameters);
    return result;
}

Vec3 specific_force_body_mps2(
    const PlantState& state,
    const Propulsion& propulsion,
    const RigidBodyParameters& rigid_body_parameters,
    const GroundContact& contact)
{
    const BodyWrench wrench = propulsion.wrench(state.rotor_speed_rad_s);
    const Vec3 force_body = wrench.force_n;
    const Vec3 force_world = state.rigid_body.attitude * force_body;
    const Vec3 acceleration_world =
        force_world / rigid_body_parameters.mass_kg + kGravityNed;
    const Vec3 acceleration_world_constrained =
        contact.constrain_acceleration(state.rigid_body, acceleration_world);
    const Vec3 specific_force_world =
        acceleration_world_constrained - kGravityNed;
    const Vec3 specific_force_body =
        state.rigid_body.attitude.conjugate() * specific_force_world;

    return specific_force_body;
}

PlantState advance(
    const PlantState& state,
    const PlantDerivative& derivative,
    double dt_s)
{
    PlantState result{};
    result.rigid_body = ofcsim::advance(
        state.rigid_body, derivative.rigid_body, dt_s);
    for (std::size_t index = 0; index < state.rotor_speed_rad_s.size(); ++index) {
        result.rotor_speed_rad_s[index] =
            state.rotor_speed_rad_s[index] +
            dt_s * derivative.rotor_speed_rad_s2[index];
    }
    return result;
}

PlantState rk4_step(
    const PlantState& state,
    const MotorArray& motor_commands,
    const Propulsion& propulsion,
    const VehicleConfig& vehicle,
    const RigidBodyParameters& rigid_body_parameters,
    double dt_s)
{
    const auto plant_derivative =
        [&](const PlantState& current_state) {
            return derivative(
                current_state,
                motor_commands,
                propulsion,
                vehicle,
                rigid_body_parameters);
        };

    const PlantDerivative k1 = plant_derivative(state);
    const PlantDerivative k2 = plant_derivative(
        advance(state, k1, dt_s / 2.0));
    const PlantDerivative k3 = plant_derivative(
        advance(state, k2, dt_s / 2.0));
    const PlantDerivative k4 = plant_derivative(
        advance(state, k3, dt_s));

    const PlantDerivative average =
        (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0;
    return advance(state, average, dt_s);
}

PlantState step(
    const PlantState& state,
    const MotorArray& motor_commands,
    const Propulsion& propulsion,
    const VehicleConfig& vehicle,
    const RigidBodyParameters& rigid_body_parameters,
    const GroundContact& contact,
    double dt_s)
{
    PlantState result = rk4_step(
        state,
        motor_commands,
        propulsion,
        vehicle,
        rigid_body_parameters,
        dt_s);
    contact.enforce(result.rigid_body);
    return result;
}

}  // namespace ofcsim
