#include "ofcsim/plant.hpp"

#include <cmath>

#include <gtest/gtest.h>

namespace {

ofcsim::VehicleConfig test_vehicle()
{
    ofcsim::VehicleConfig vehicle{};
    vehicle.mass_kg = 2.0;
    vehicle.inertia_kgm2 = ofcsim::Mat3::Identity();
    vehicle.inertia_inv_kgm2 = ofcsim::Mat3::Identity();

    const ofcsim::Vec3 positions[] = {
        {1.0, -1.0, 0.0},
        {-1.0, -1.0, 0.0},
        {1.0, 1.0, 0.0},
        {-1.0, 1.0, 0.0},
    };
    const int spin_directions[] = {1, -1, -1, 1};
    for (std::size_t index = 0; index < vehicle.motors.size(); ++index) {
        vehicle.motors[index].position_m = positions[index];
        vehicle.motors[index].spin_direction = spin_directions[index];
        vehicle.motors[index].thrust_coeff = 2.0;
        vehicle.motors[index].torque_coeff = 0.5;
        vehicle.motors[index].max_speed_rad_s = 100.0;
        vehicle.motors[index].time_constant_s = 0.1;
    }
    return vehicle;
}

ofcsim::RigidBodyParameters test_parameters()
{
    return {
        .mass_kg = 2.0,
        .inertia_kgm2 = ofcsim::Mat3::Identity(),
        .inertia_inv_kgm2 = ofcsim::Mat3::Identity(),
    };
}

}  // namespace

TEST(Plant, DerivativeUsesCurrentRotorSpeedAndLagTarget)
{
    const ofcsim::VehicleConfig vehicle = test_vehicle();
    const ofcsim::Propulsion propulsion(vehicle);
    const ofcsim::RigidBodyParameters parameters = test_parameters();
    ofcsim::PlantState state{};
    state.rotor_speed_rad_s = {50.0, 50.0, 50.0, 50.0};

    const ofcsim::PlantDerivative result = ofcsim::derivative(
        state, {0.2, 0.2, 0.2, 0.2}, propulsion, vehicle, parameters);

    for (const double rate : result.rotor_speed_rad_s2) {
        EXPECT_DOUBLE_EQ(rate, -300.0);
    }
}

TEST(Plant, Rk4IntegratesRotorLag)
{
    const ofcsim::VehicleConfig vehicle = test_vehicle();
    const ofcsim::Propulsion propulsion(vehicle);
    const ofcsim::RigidBodyParameters parameters = test_parameters();
    const ofcsim::PlantState state{};

    const ofcsim::PlantState result = ofcsim::rk4_step(
        state,
        {0.5, 0.5, 0.5, 0.5},
        propulsion,
        vehicle,
        parameters,
        0.01);

    const double expected_speed =
        50.0 * (1.0 - std::exp(-0.01 / 0.1));
    for (const double speed : result.rotor_speed_rad_s) {
        EXPECT_NEAR(speed, expected_speed, 1.0e-5);
    }
}

TEST(Plant, PropulsionUsesActualRotorSpeedDuringIntegration)
{
    const ofcsim::VehicleConfig vehicle = test_vehicle();
    const ofcsim::Propulsion propulsion(vehicle);
    const ofcsim::RigidBodyParameters parameters = test_parameters();
    ofcsim::PlantState state{};
    state.rotor_speed_rad_s = {1.0, 1.0, 1.0, 1.0};

    const ofcsim::PlantDerivative result = ofcsim::derivative(
        state, {0.0, 0.0, 0.0, 0.0}, propulsion, vehicle, parameters);

    EXPECT_DOUBLE_EQ(result.rigid_body.velocity_mps2.z(),
                     ofcsim::kGravityNed.z() - 4.0);
}

TEST(Plant, BelowHoverCommandRemainsOnGround)
{
    const ofcsim::VehicleConfig vehicle = test_vehicle();
    const ofcsim::Propulsion propulsion(vehicle);
    const ofcsim::RigidBodyParameters parameters = test_parameters();
    const ofcsim::GroundContact contact;
    ofcsim::PlantState state{};

    const double hover_speed = std::sqrt(
        vehicle.mass_kg * ofcsim::kGravityNed.z() /
        (4.0 * vehicle.motors[0].thrust_coeff));
    const double rotor_speed = 0.9 * hover_speed;
    state.rotor_speed_rad_s.fill(rotor_speed);
    const ofcsim::MotorArray command{
        rotor_speed / vehicle.motors[0].max_speed_rad_s,
        rotor_speed / vehicle.motors[1].max_speed_rad_s,
        rotor_speed / vehicle.motors[2].max_speed_rad_s,
        rotor_speed / vehicle.motors[3].max_speed_rad_s};

    for (int step = 0; step < 1000; ++step) {
        state = ofcsim::step(
            state, command, propulsion, vehicle, parameters, contact, 0.001);
    }

    EXPECT_DOUBLE_EQ(state.rigid_body.position_m.z(), 0.0);
    EXPECT_DOUBLE_EQ(state.rigid_body.velocity_mps.z(), 0.0);
}

TEST(Plant, AboveHoverCommandProducesLiftoff)
{
    const ofcsim::VehicleConfig vehicle = test_vehicle();
    const ofcsim::Propulsion propulsion(vehicle);
    const ofcsim::RigidBodyParameters parameters = test_parameters();
    const ofcsim::GroundContact contact;
    ofcsim::PlantState state{};

    const double hover_speed = std::sqrt(
        vehicle.mass_kg * ofcsim::kGravityNed.z() /
        (4.0 * vehicle.motors[0].thrust_coeff));
    const double rotor_speed = 1.1 * hover_speed;
    state.rotor_speed_rad_s.fill(rotor_speed);
    const ofcsim::MotorArray command{
        rotor_speed / vehicle.motors[0].max_speed_rad_s,
        rotor_speed / vehicle.motors[1].max_speed_rad_s,
        rotor_speed / vehicle.motors[2].max_speed_rad_s,
        rotor_speed / vehicle.motors[3].max_speed_rad_s};

    for (int step = 0; step < 1000; ++step) {
        state = ofcsim::step(
            state, command, propulsion, vehicle, parameters, contact, 0.001);
    }

    EXPECT_LT(state.rigid_body.position_m.z(), 0.0);
    EXPECT_LT(state.rigid_body.velocity_mps.z(), 0.0);
}
