#pragma once

#include "ofcsim/config/vehicle_config.hpp"

#include <cstddef>

namespace ofcsim {

class Propulsion {
public:
    explicit Propulsion(const VehicleConfig& vehicle);

    [[nodiscard]] double commanded_speed_rad_s(
        std::size_t motor_index,
        double command) const;

    [[nodiscard]] BodyWrench wrench(
        const MotorArray& rotor_speed_rad_s) const;

private:
    VehicleConfig vehicle_;
};

}  // namespace ofcsim
