#include "ofcsim/config/scenario_config.hpp"
#include "ofcsim/profiles/flight_profile.hpp"
#include "ofcsim/profiles/keyframe_flight_profile.hpp"

#include <filesystem>
#include <fstream>
#include <limits>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

namespace {

std::filesystem::path resource(const char* name)
{
    return std::filesystem::path(OFCSIM_CONFIG_RESOURCE_DIR) / name;
}

}  // namespace

TEST(FlightProfileConfig, LoadsOpenLoopKeyframes)
{
    const std::unique_ptr<ofcsim::FlightProfile> profile =
        ofcsim::load_flight_profile(resource("open_loop_hop.json"));

    ASSERT_NE(profile, nullptr);
    EXPECT_EQ(profile->name(), "open_loop_hop");
    EXPECT_EQ(profile->type(), "keyframe_motor_commands");
}

TEST(FlightProfileConfig, FactoryCreatesProfileByType)
{
    const std::unique_ptr<ofcsim::FlightProfile> profile =
        ofcsim::load_flight_profile(resource("open_loop_hop.json"));

    ASSERT_NE(profile, nullptr);
    EXPECT_EQ(profile->name(), "open_loop_hop");
}

TEST(FlightProfileConfig, DecodesThenPreprocessesInTwoStages)
{
    std::ifstream input(resource("open_loop_hop.json"));
    ASSERT_TRUE(input);
    const nlohmann::json document = nlohmann::json::parse(input);

    const ofcsim::KeyframeProfileConfig parsed =
        ofcsim::KeyframeFlightProfile::from_json(document);
    ASSERT_EQ(parsed.keyframes.size(), 3U);
    EXPECT_DOUBLE_EQ(parsed.keyframes[1].time_s, 0.5);

    const std::unique_ptr<ofcsim::FlightProfile> profile =
        ofcsim::KeyframeFlightProfile::preprocess(parsed);
    ASSERT_NE(profile, nullptr);
    EXPECT_EQ(profile->type(), "keyframe_motor_commands");
}

TEST(FlightProfileConfig, KeyframeProfileReturnsMostRecentCommand)
{
    const std::unique_ptr<ofcsim::FlightProfile> profile =
        ofcsim::load_flight_profile(resource("open_loop_hop.json"));

    EXPECT_EQ(
        profile->motor_commands_at(-1.0),
        (ofcsim::MotorArray{0.0, 0.0, 0.0, 0.0}));
    EXPECT_EQ(
        profile->motor_commands_at(0.5),
        (ofcsim::MotorArray{0.35, 0.35, 0.35, 0.35}));
    EXPECT_EQ(
        profile->motor_commands_at(1.0),
        (ofcsim::MotorArray{0.35, 0.35, 0.35, 0.35}));
    EXPECT_EQ(
        profile->motor_commands_at(2.0),
        (ofcsim::MotorArray{0.0, 0.0, 0.0, 0.0}));
    EXPECT_THROW(
        (void)profile->motor_commands_at(
            std::numeric_limits<double>::quiet_NaN()),
        std::invalid_argument);
}

TEST(ScenarioConfig, LoadsOpenLoopScenario)
{
    const ofcsim::ScenarioConfig scenario =
        ofcsim::load_scenario(resource("open_loop_hop_scenario.json"));

    EXPECT_EQ(scenario.name, "open_loop_hop");
    EXPECT_EQ(
        scenario.vehicle_path,
        std::filesystem::path("../vehicles/ofc_v1_5inch_placeholder.json"));
    EXPECT_EQ(
        scenario.flight_profile_path,
        std::filesystem::path("../flight_profiles/open_loop_hop.json"));
    EXPECT_DOUBLE_EQ(scenario.duration_s, 2.0);
    EXPECT_DOUBLE_EQ(scenario.physics_step_s, 0.001);
    EXPECT_DOUBLE_EQ(scenario.ground_z_m, 0.0);
    EXPECT_TRUE(ofcsim::validate_scenario(scenario));
}
