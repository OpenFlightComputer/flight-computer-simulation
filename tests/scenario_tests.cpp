#include "ofcsim/config/scenario_config.hpp"
#include "ofcsim/config/vehicle_config.hpp"
#include "ofcsim/plant.hpp"
#include "ofcsim/profiles/flight_profile.hpp"
#include "ofcsim/propulsion.hpp"

#include <algorithm>
#include <filesystem>
#include <memory>

#include <gtest/gtest.h>

namespace {

std::filesystem::path resolve_reference(
    const std::filesystem::path& scenario_path,
    const std::filesystem::path& reference)
{
    return reference.is_absolute()
        ? reference
        : scenario_path.parent_path() / reference;
}

}  // namespace

TEST(Scenario, OpenLoopHopLoadsAndLiftsOff)
{
    const std::filesystem::path scenario_path = OFCSIM_OPEN_LOOP_SCENARIO;
    const ofcsim::ScenarioConfig scenario =
        ofcsim::load_scenario(scenario_path);
    const ofcsim::VehicleConfig vehicle = ofcsim::load_vehicle_config(
        resolve_reference(scenario_path, scenario.vehicle_path));
    const std::unique_ptr<ofcsim::FlightProfile> profile =
        ofcsim::load_flight_profile(
            resolve_reference(scenario_path, scenario.flight_profile_path));

    const ofcsim::RigidBodyParameters parameters{
        .mass_kg = vehicle.mass_kg,
        .inertia_kgm2 = vehicle.inertia_kgm2,
        .inertia_inv_kgm2 = vehicle.inertia_inv_kgm2};
    const ofcsim::Propulsion propulsion(vehicle);
    const ofcsim::GroundContact contact(scenario.ground_z_m);
    ofcsim::PlantState state{};
    state.rigid_body = scenario.initial_state;

    double time_s = 0.0;
    double minimum_z_m = state.rigid_body.position_m.z();
    int sample_count = 1;
    constexpr double kTimeToleranceS = 1.0e-12;
    while (time_s < scenario.duration_s - kTimeToleranceS) {
        const double step_s = std::min(
            scenario.physics_step_s, scenario.duration_s - time_s);
        const ofcsim::MotorArray command =
            profile->motor_commands_at(time_s);
        state = ofcsim::step(
            state,
            command,
            propulsion,
            vehicle,
            parameters,
            contact,
            step_s);
        time_s += step_s;
        if (scenario.duration_s - time_s < kTimeToleranceS) {
            time_s = scenario.duration_s;
        }
        minimum_z_m = std::min(
            minimum_z_m, state.rigid_body.position_m.z());
        ++sample_count;

        if (time_s <= 0.5) {
            EXPECT_DOUBLE_EQ(state.rigid_body.position_m.z(), 0.0);
        }
    }

    EXPECT_EQ(sample_count, 2001);
    EXPECT_LT(minimum_z_m, -0.1);
    EXPECT_LT(state.rigid_body.position_m.z(), 0.0);
}
