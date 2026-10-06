#include "ofcsim/profiles/keyframe_flight_profile.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

#include <nlohmann/json.hpp>

namespace ofcsim {

KeyframeProfileConfig KeyframeFlightProfile::from_json(
    const nlohmann::json& document)
{
    KeyframeProfileConfig config{};
    config.metadata = metadata_from_json(document);
    config.interpolation = document.at("interpolation").get<std::string>();
    const auto& documents = document.at("keyframes");
    if (!documents.is_array()) {
        throw std::invalid_argument("keyframes must be an array");
    }
    for (const auto& keyframe_document : documents) {
        KeyframeConfig keyframe{};
        keyframe.time_s = keyframe_document.at("time_s").get<double>();
        const auto& commands = keyframe_document.at("motor_commands");
        if (!commands.is_array() || commands.size() != keyframe.motor_commands.size()) {
            throw std::invalid_argument("keyframe must contain four motor commands");
        }
        for (std::size_t index = 0; index < keyframe.motor_commands.size(); ++index) {
            keyframe.motor_commands[index] = commands.at(index).get<double>();
        }
        config.keyframes.push_back(keyframe);
    }
    return config;
}

KeyframeFlightProfile::KeyframeFlightProfile(
    FlightProfileMetadata metadata,
    std::vector<KeyframeConfig> keyframes)
    : metadata_(std::move(metadata)), keyframes_(std::move(keyframes))
{
}

std::unique_ptr<FlightProfile> KeyframeFlightProfile::preprocess(
    KeyframeProfileConfig config)
{
    if (config.metadata.type != "keyframe_motor_commands" ||
        config.interpolation != "step" || config.keyframes.empty()) {
        throw std::invalid_argument("invalid keyframe flight profile");
    }
    double previous_time_s = -1.0;
    for (const KeyframeConfig& keyframe : config.keyframes) {
        if (!std::isfinite(keyframe.time_s) || keyframe.time_s < 0.0 ||
            keyframe.time_s < previous_time_s) {
            throw std::invalid_argument(
                "keyframe times must be ordered and non-negative");
        }
        previous_time_s = keyframe.time_s;
        for (const double command : keyframe.motor_commands) {
            if (!std::isfinite(command) || command < 0.0 || command > 1.0) {
                throw std::invalid_argument(
                    "keyframe motor commands must be within [0, 1]");
            }
        }
    }
    return std::unique_ptr<FlightProfile>(new KeyframeFlightProfile(
        std::move(config.metadata), std::move(config.keyframes)));
}

MotorArray KeyframeFlightProfile::motor_commands_at(double time_s) const
{
    if (!std::isfinite(time_s)) {
        throw std::invalid_argument("flight profile time must be finite");
    }

    MotorArray command = keyframes_.front().motor_commands;
    for (const KeyframeConfig& keyframe : keyframes_) {
        if (time_s < keyframe.time_s) {
            break;
        }
        command = keyframe.motor_commands;
    }
    return command;
}

std::string_view KeyframeFlightProfile::name() const { return metadata_.name; }

std::string_view KeyframeFlightProfile::type() const { return metadata_.type; }

}  // namespace ofcsim
