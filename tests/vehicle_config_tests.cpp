#include "ofcsim/vehicle_config.hpp"

#include <filesystem>
#include <fstream>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

namespace {

std::filesystem::path test_vehicle_path()
{
    return std::filesystem::path(OFCSIM_TEST_RESOURCE_DIR) /
        "ofc_vehicle_config_test.json";
}

std::filesystem::path malformed_vehicle_path()
{
    return std::filesystem::path(OFCSIM_TEST_RESOURCE_DIR) /
        "malformed_vehicle_config.json";
}

nlohmann::json test_vehicle_json()
{
    std::ifstream input(test_vehicle_path());
    return nlohmann::json::parse(input);
}

}  // namespace

TEST(VehicleConfig, LoadsPhysicalParameters)
{
    const ofcsim::VehicleConfig config =
        ofcsim::load_vehicle_config(test_vehicle_path());
    const ofcsim::Mat3 expected_inertia =
        ofcsim::Vec3(2.0, 3.0, 4.0).asDiagonal();
    const ofcsim::Mat3 expected_inverse =
        ofcsim::Vec3(0.5, 1.0 / 3.0, 0.25).asDiagonal();

    EXPECT_DOUBLE_EQ(config.mass_kg, 2.0);
    EXPECT_TRUE(config.inertia_kgm2.isApprox(expected_inertia));
    EXPECT_TRUE(config.inertia_inv_kgm2.isApprox(expected_inverse));
    EXPECT_TRUE(ofcsim::validate_vehicle_config(config));
}

TEST(VehicleConfig, LoadsMotorPositions)
{
    const ofcsim::VehicleConfig config =
        ofcsim::load_vehicle_config(test_vehicle_path());

    EXPECT_TRUE(config.motors[0].position_m.isApprox(
        ofcsim::Vec3(1.0, -1.0, 0.0)));
    EXPECT_TRUE(config.motors[1].position_m.isApprox(
        ofcsim::Vec3(-1.0, -1.0, 0.0)));
    EXPECT_TRUE(config.motors[2].position_m.isApprox(
        ofcsim::Vec3(1.0, 1.0, 0.0)));
    EXPECT_TRUE(config.motors[3].position_m.isApprox(
        ofcsim::Vec3(-1.0, 1.0, 0.0)));
}

TEST(VehicleConfig, LoadsMotorDefaultsAndSpinDirections)
{
    const ofcsim::VehicleConfig config =
        ofcsim::load_vehicle_config(test_vehicle_path());

    EXPECT_EQ(config.motors[0].spin_direction, 1);
    EXPECT_EQ(config.motors[1].spin_direction, -1);
    EXPECT_EQ(config.motors[2].spin_direction, -1);
    EXPECT_EQ(config.motors[3].spin_direction, 1);
    for (const ofcsim::MotorConfig& motor : config.motors) {
        EXPECT_DOUBLE_EQ(motor.thrust_coeff, 2.0);
        EXPECT_DOUBLE_EQ(motor.torque_coeff, 0.5);
        EXPECT_DOUBLE_EQ(motor.max_speed_rad_s, 100.0);
        EXPECT_DOUBLE_EQ(motor.time_constant_s, 0.1);
    }
}

TEST(VehicleConfig, RejectsMissingFile)
{
    EXPECT_THROW(
        (void)ofcsim::load_vehicle_config("does-not-exist.json"),
        std::runtime_error);
}

TEST(VehicleConfig, RejectsMalformedJson)
{
    EXPECT_THROW(
        (void)ofcsim::load_vehicle_config(malformed_vehicle_path()),
        nlohmann::json::parse_error);
}

TEST(VehicleConfig, RejectsMissingRequiredField)
{
    nlohmann::json document = test_vehicle_json();
    document.erase("mass_kg");

    EXPECT_THROW(
        document.get<ofcsim::VehicleConfig>(),
        nlohmann::json::out_of_range);
}

TEST(VehicleConfig, RejectsMissingMotorPosition)
{
    nlohmann::json document = test_vehicle_json();
    document["motors"][0].erase("position_m");

    EXPECT_THROW(
        document.get<ofcsim::VehicleConfig>(),
        nlohmann::json::out_of_range);
}

TEST(VehicleConfig, RejectsWrongMotorCount)
{
    nlohmann::json document = test_vehicle_json();
    document["motors"].erase(3);

    EXPECT_THROW(
        document.get<ofcsim::VehicleConfig>(),
        std::invalid_argument);
}

TEST(VehicleConfig, RejectsUnknownMotorCorner)
{
    nlohmann::json document = test_vehicle_json();
    document["motors"][0]["corner"] = "middle";

    EXPECT_THROW(
        document.get<ofcsim::VehicleConfig>(),
        std::invalid_argument);
}

TEST(VehicleConfig, RejectsInvalidPhysicalValues)
{
    ofcsim::VehicleConfig config =
        ofcsim::load_vehicle_config(test_vehicle_path());

    config.mass_kg = 0.0;
    EXPECT_FALSE(ofcsim::validate_vehicle_config(config));

    config = ofcsim::load_vehicle_config(test_vehicle_path());
    config.motors[0].spin_direction = 0;
    EXPECT_FALSE(ofcsim::validate_vehicle_config(config));

    config = ofcsim::load_vehicle_config(test_vehicle_path());
    config.motors[0].thrust_coeff = -1.0;
    EXPECT_FALSE(ofcsim::validate_vehicle_config(config));
}

TEST(VehicleConfig, RejectsInvalidInertia)
{
    ofcsim::VehicleConfig config =
        ofcsim::load_vehicle_config(test_vehicle_path());

    config.inertia_kgm2(0, 1) = 0.1;
    EXPECT_FALSE(ofcsim::validate_vehicle_config(config));

    config = ofcsim::load_vehicle_config(test_vehicle_path());
    config.inertia_kgm2(0, 0) = -1.0;
    config.inertia_inv_kgm2 = config.inertia_kgm2.inverse();
    EXPECT_FALSE(ofcsim::validate_vehicle_config(config));

    config = ofcsim::load_vehicle_config(test_vehicle_path());
    config.inertia_inv_kgm2.setZero();
    EXPECT_FALSE(ofcsim::validate_vehicle_config(config));
}

TEST(VehicleConfig, RejectsDuplicateMotorPositions)
{
    ofcsim::VehicleConfig config =
        ofcsim::load_vehicle_config(test_vehicle_path());
    config.motors[1].position_m = config.motors[0].position_m;

    EXPECT_FALSE(ofcsim::validate_vehicle_config(config));
}
