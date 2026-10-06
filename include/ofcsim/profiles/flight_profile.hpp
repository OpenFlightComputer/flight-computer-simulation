#pragma once

#include "ofcsim/config/flight_profile_config.hpp"
#include "ofcsim/config/vehicle_config.hpp"

#include <filesystem>
#include <memory>
#include <string_view>

namespace ofcsim {

class FlightProfile {
public:
    virtual ~FlightProfile() = default;

    [[nodiscard]] virtual MotorArray motor_commands_at(
        double time_s) const = 0;

    [[nodiscard]] virtual std::string_view name() const = 0;

    [[nodiscard]] virtual std::string_view type() const = 0;
};

[[nodiscard]] std::unique_ptr<FlightProfile> load_flight_profile(
    const std::filesystem::path& path);

}  // namespace ofcsim
