#include "ofcsim/profiles/flight_profile.hpp"
#include "ofcsim/profiles/keyframe_flight_profile.hpp"

#include <fstream>
#include <stdexcept>

#include <nlohmann/json.hpp>

namespace ofcsim {

std::unique_ptr<FlightProfile> load_flight_profile(
    const std::filesystem::path& path)
{
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("could not open flight profile: " + path.string());
    }
    const nlohmann::json document = nlohmann::json::parse(input);
    const FlightProfileMetadata metadata = metadata_from_json(document);
    if (metadata.type == "keyframe_motor_commands") {
        return KeyframeFlightProfile::preprocess(
            KeyframeFlightProfile::from_json(document));
    }
    throw std::invalid_argument("unsupported flight profile type: " + metadata.type);
}

}  // namespace ofcsim
