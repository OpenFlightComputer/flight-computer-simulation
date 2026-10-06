#include "ofcsim/ofcsim.hpp"
#include "ofcsim/config/scenario_config.hpp"
#include "ofcsim/config/vehicle_config.hpp"
#include "ofcsim/propulsion.hpp"
#include "ofcsim/profiles/flight_profile.hpp"
#include "ofcsim/trajectory_csv_writer.hpp"

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <stdexcept>

namespace {

std::filesystem::path resolve_reference(
    const std::filesystem::path& scenario_path,
    const std::filesystem::path& reference)
{
    if (reference.is_absolute()) {
        return reference;
    }
    return scenario_path.parent_path() / reference;
}

}  // namespace

int main(int argc, char* argv[])
{
    try {

        //Get parameters
        const std::filesystem::path scenario_path = argc > 1 ? argv[1] : "scenarios/open_loop_hop.json";
        const std::filesystem::path output_path = argc > 2 ? argv[2] : "trajectory.csv";

        //Load Configs
        const ofcsim::ScenarioConfig scenario = ofcsim::load_scenario(scenario_path);
        const ofcsim::VehicleConfig vehicle = ofcsim::load_vehicle_config(resolve_reference(scenario_path, scenario.vehicle_path));
        const std::unique_ptr<ofcsim::FlightProfile> flight_profile =ofcsim::load_flight_profile(resolve_reference(scenario_path, scenario.flight_profile_path));

        //Create Variables
        const ofcsim::RigidBodyParameters rigid_body_parameters{
            .mass_kg = vehicle.mass_kg,
            .inertia_kgm2 = vehicle.inertia_kgm2,
            .inertia_inv_kgm2 = vehicle.inertia_inv_kgm2};
        const ofcsim::Propulsion propulsion(vehicle);
        const ofcsim::GroundContact ground_contact(scenario.ground_z_m);
        ofcsim::PlantState state{};
        state.rigid_body = scenario.initial_state;
        ofcsim::TrajectoryCsvWriter writer(output_path);

        //Starting time
        double time_s = 0.0;

        //Get first command
        ofcsim::MotorArray command = flight_profile->motor_commands_at(time_s);

        //Log starting point
        writer.write(time_s, state, command);
        constexpr double kTimeToleranceS = 1.0e-12;
        while (time_s < scenario.duration_s - kTimeToleranceS) {
            const double step_s = std::min(scenario.physics_step_s, scenario.duration_s - time_s);
            command = flight_profile->motor_commands_at(time_s);
            state = ofcsim::step(
                state,
                command,
                propulsion,
                vehicle,
                rigid_body_parameters,
                ground_contact,
                step_s);
            time_s += step_s;
            if (scenario.duration_s - time_s < kTimeToleranceS) {
                time_s = scenario.duration_s;
            }
            writer.write(time_s, state, command);
        }

        std::cout << "ofcsim " << ofcsim::version() << '\n'
                  << "scenario: " << scenario.name << '\n'
                  << "vehicle mass: " << vehicle.mass_kg << " kg\n"
                  << "firmware: " << vehicle.firmware.repository << '@'
                  << vehicle.firmware.commit << '\n'
                  << "flight profile: " << flight_profile->name() << " ("
                  << flight_profile->type() << ")\n"
                  << "duration: " << scenario.duration_s << " s\n"
                  << "physics step: " << scenario.physics_step_s << " s\n"
                  << "trajectory: " << output_path << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "ofcsim: " << error.what() << '\n';
        return 1;
    }
}
