#include "ofcsim/config/flight_profile_config.hpp"

#include <stdexcept>

#include <nlohmann/json.hpp>

namespace ofcsim {

FlightProfileMetadata metadata_from_json(const nlohmann::json& document)
{
    FlightProfileMetadata metadata{};
    document.at("schema_version").get_to(metadata.schema_version);
    document.at("name").get_to(metadata.name);
    document.at("type").get_to(metadata.type);
    if (!validate_profile_metadata(metadata)) {
        throw std::invalid_argument("flight profile metadata failed validation");
    }
    return metadata;
}

bool validate_profile_metadata(const FlightProfileMetadata& metadata)
{
    return metadata.schema_version == 1 && !metadata.name.empty() &&
        !metadata.type.empty();
}

}  // namespace ofcsim
