#include "ofcsim/propulsion.hpp"
#include <algorithm>

namespace ofcsim {

Propulsion::Propulsion(const VehicleConfig& vehicle)
    : vehicle_(vehicle)
{
}

double Propulsion::commanded_speed_rad_s(
    std::size_t motor_index,
    double command) const
{
    const double clamped_command = std::clamp(command, 0.0, 1.0);
    return clamped_command *
        vehicle_.motors.at(motor_index).max_speed_rad_s;
}

BodyWrench Propulsion::wrench(
    const MotorArray& rotor_speed_rad_s) const
{
    BodyWrench result{};
    for (std::size_t i = 0; i < vehicle_.motors.size(); ++i) {
        const MotorConfig& motor = vehicle_.motors.at(i);
        const double speed_squared =
            rotor_speed_rad_s.at(i) * rotor_speed_rad_s.at(i);
        const Vec3 thrust_n(0.0, 0.0, -motor.thrust_coeff * speed_squared);
        result.force_n += thrust_n;
        result.torque_nm += motor.position_m.cross(thrust_n);
        result.torque_nm.z() +=
          -motor.spin_direction *
          motor.torque_coeff *
          speed_squared;
    }
    return result;
}

}  // namespace ofcsim
