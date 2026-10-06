#include "ofcsim/trajectory_csv_writer.hpp"

#include <iomanip>
#include <stdexcept>

namespace ofcsim {

TrajectoryCsvWriter::TrajectoryCsvWriter(const std::filesystem::path& path)
    : output_(path, std::ios::out | std::ios::trunc)
{
    if (!output_) {
        throw std::runtime_error(
            "could not open trajectory output: " + path.string());
    }

    output_ << "time_s,"
               "position_x_m,position_y_m,position_z_m,"
               "velocity_x_mps,velocity_y_mps,velocity_z_mps,"
               "rotor_speed_0_rad_s,rotor_speed_1_rad_s,"
               "rotor_speed_2_rad_s,rotor_speed_3_rad_s,"
               "motor_command_0,motor_command_1,"
               "motor_command_2,motor_command_3\n";
}

TrajectoryCsvWriter::~TrajectoryCsvWriter()
{
    output_.close();
}

void TrajectoryCsvWriter::write(
    double time_s,
    const PlantState& state,
    const MotorArray& motor_commands)
{
    output_ << std::setprecision(17)
            << time_s << ','
            << state.rigid_body.position_m.x() << ','
            << state.rigid_body.position_m.y() << ','
            << state.rigid_body.position_m.z() << ','
            << state.rigid_body.velocity_mps.x() << ','
            << state.rigid_body.velocity_mps.y() << ','
            << state.rigid_body.velocity_mps.z() << ','
            << state.rotor_speed_rad_s[0] << ','
            << state.rotor_speed_rad_s[1] << ','
            << state.rotor_speed_rad_s[2] << ','
            << state.rotor_speed_rad_s[3] << ','
            << motor_commands[0] << ','
            << motor_commands[1] << ','
            << motor_commands[2] << ','
            << motor_commands[3] << '\n';

    if (!output_) {
        throw std::runtime_error("failed while writing trajectory output");
    }
}

}  // namespace ofcsim
