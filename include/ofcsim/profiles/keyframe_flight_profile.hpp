#pragma once

#include "ofcsim/profiles/flight_profile.hpp"

#include <memory>
#include <vector>

#include <nlohmann/json_fwd.hpp>

namespace ofcsim {

class KeyframeFlightProfile final : public FlightProfile {
public:
    [[nodiscard]] static KeyframeProfileConfig from_json(
        const nlohmann::json& document);

    [[nodiscard]] static std::unique_ptr<FlightProfile> preprocess(
        KeyframeProfileConfig config);

    [[nodiscard]] MotorArray motor_commands_at(
        double time_s) const override;

    [[nodiscard]] std::string_view name() const override;

    [[nodiscard]] std::string_view type() const override;

private:
    KeyframeFlightProfile(
        FlightProfileMetadata metadata,
        std::vector<KeyframeConfig> keyframes);

    FlightProfileMetadata metadata_;
    std::vector<KeyframeConfig> keyframes_;
};

}  // namespace ofcsim
