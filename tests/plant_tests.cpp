#include "ofcsim/imu_model.hpp"
#include "ofcsim/plant.hpp"

#include <cmath>
#include <numbers>

#include <gtest/gtest.h>

namespace {

ofcsim::VehicleConfig test_vehicle()
{
    ofcsim::VehicleConfig vehicle{};
    vehicle.mass_kg = 2.0;
    vehicle.inertia_kgm2 = ofcsim::Mat3::Identity();
    vehicle.inertia_inv_kgm2 = ofcsim::Mat3::Identity();

    const ofcsim::Vec3 positions[] = {
        {1.0, -1.0, 0.0},
        {-1.0, -1.0, 0.0},
        {1.0, 1.0, 0.0},
        {-1.0, 1.0, 0.0},
    };
    const int spin_directions[] = {1, -1, -1, 1};
    for (std::size_t index = 0; index < vehicle.motors.size(); ++index) {
        vehicle.motors[index].position_m = positions[index];
        vehicle.motors[index].spin_direction = spin_directions[index];
        vehicle.motors[index].thrust_coeff = 2.0;
        vehicle.motors[index].torque_coeff = 0.5;
        vehicle.motors[index].max_speed_rad_s = 100.0;
        vehicle.motors[index].time_constant_s = 0.1;
    }
    return vehicle;
}

ofcsim::RigidBodyParameters test_parameters()
{
    return {
        .mass_kg = 2.0,
        .inertia_kgm2 = ofcsim::Mat3::Identity(),
        .inertia_inv_kgm2 = ofcsim::Mat3::Identity(),
    };
}

imu_processing_config_t test_imu_processing_config()
{
    imu_processing_config_t config{};
    config.acceleration_filter.type =
        ACCELERATION_FILTER_FIRST_ORDER_LOW_PASS;
    config.acceleration_filter.cutoff_hz = 50.0F;
    config.gyro_filter.type = GYRO_FILTER_FIRST_ORDER_LOW_PASS;
    config.gyro_filter.cutoff_hz = 50.0F;
    config.attitude_estimator.type = ATTITUDE_ESTIMATOR_COMPLEMENTARY;
    config.attitude_estimator.accelerometer_correction_time_constant_s =
        0.5F;
    config.maximum_gap_us = 10000U;
    config.acceleration_counts_per_g = 4096.0F;
    config.gyroscope_counts_per_dps = 16.384F;
    return config;
}

ofcsim::ImuModelConfig ideal_imu_config()
{
    ofcsim::ImuModelConfig config{};
    config.acceleration_bias_g.fill(0.0);
    config.gyroscope_bias_dps.fill(0.0);
    config.acceleration_noise_std_g = 0.0;
    config.gyroscope_noise_std_dps = 0.0;
    return config;
}

}  // namespace

TEST(Plant, DerivativeUsesCurrentRotorSpeedAndLagTarget)
{
    const ofcsim::VehicleConfig vehicle = test_vehicle();
    const ofcsim::Propulsion propulsion(vehicle);
    const ofcsim::RigidBodyParameters parameters = test_parameters();
    ofcsim::PlantState state{};
    state.rotor_speed_rad_s = {50.0, 50.0, 50.0, 50.0};

    const ofcsim::PlantDerivative result = ofcsim::derivative(
        state, {0.2, 0.2, 0.2, 0.2}, propulsion, vehicle, parameters);

    for (const double rate : result.rotor_speed_rad_s2) {
        EXPECT_DOUBLE_EQ(rate, -300.0);
    }
}

TEST(Plant, Rk4IntegratesRotorLag)
{
    const ofcsim::VehicleConfig vehicle = test_vehicle();
    const ofcsim::Propulsion propulsion(vehicle);
    const ofcsim::RigidBodyParameters parameters = test_parameters();
    const ofcsim::PlantState state{};

    const ofcsim::PlantState result = ofcsim::rk4_step(
        state,
        {0.5, 0.5, 0.5, 0.5},
        propulsion,
        vehicle,
        parameters,
        0.01);

    const double expected_speed =
        50.0 * (1.0 - std::exp(-0.01 / 0.1));
    for (const double speed : result.rotor_speed_rad_s) {
        EXPECT_NEAR(speed, expected_speed, 1.0e-5);
    }
}

TEST(Plant, PropulsionUsesActualRotorSpeedDuringIntegration)
{
    const ofcsim::VehicleConfig vehicle = test_vehicle();
    const ofcsim::Propulsion propulsion(vehicle);
    const ofcsim::RigidBodyParameters parameters = test_parameters();
    ofcsim::PlantState state{};
    state.rotor_speed_rad_s = {1.0, 1.0, 1.0, 1.0};

    const ofcsim::PlantDerivative result = ofcsim::derivative(
        state, {0.0, 0.0, 0.0, 0.0}, propulsion, vehicle, parameters);

    EXPECT_DOUBLE_EQ(result.rigid_body.velocity_mps2.z(),
                     ofcsim::kGravityNed.z() - 4.0);
}

TEST(Plant, SpecificForceAtGroundedRestIsNegativeOneG)
{
    const ofcsim::VehicleConfig vehicle = test_vehicle();
    const ofcsim::Propulsion propulsion(vehicle);
    const ofcsim::RigidBodyParameters parameters = test_parameters();
    const ofcsim::GroundContact contact;
    const ofcsim::PlantState state{};

    const ofcsim::Vec3 result = ofcsim::specific_force_body_mps2(
        state, propulsion, parameters, contact);

    EXPECT_NEAR(result.x(), 0.0, 1.0e-12);
    EXPECT_NEAR(result.y(), 0.0, 1.0e-12);
    EXPECT_NEAR(result.z(), -ofcsim::kGravityNed.z(), 1.0e-12);
}

TEST(Plant, SpecificForceInFreeFallIsZero)
{
    const ofcsim::VehicleConfig vehicle = test_vehicle();
    const ofcsim::Propulsion propulsion(vehicle);
    const ofcsim::RigidBodyParameters parameters = test_parameters();
    const ofcsim::GroundContact contact;
    ofcsim::PlantState state{};
    state.rigid_body.position_m.z() = -1.0;

    const ofcsim::Vec3 result = ofcsim::specific_force_body_mps2(
        state, propulsion, parameters, contact);

    EXPECT_TRUE(result.isApprox(ofcsim::Vec3::Zero(), 1.0e-12));
}

TEST(Plant, SpecificForceFromThrustExcludesGravity)
{
    const ofcsim::VehicleConfig vehicle = test_vehicle();
    const ofcsim::Propulsion propulsion(vehicle);
    const ofcsim::RigidBodyParameters parameters = test_parameters();
    const ofcsim::GroundContact contact;
    ofcsim::PlantState state{};
    state.rigid_body.position_m.z() = -1.0;
    state.rotor_speed_rad_s.fill(2.0);

    const ofcsim::Vec3 result = ofcsim::specific_force_body_mps2(
        state, propulsion, parameters, contact);

    EXPECT_NEAR(result.x(), 0.0, 1.0e-12);
    EXPECT_NEAR(result.y(), 0.0, 1.0e-12);
    EXPECT_NEAR(result.z(), -16.0, 1.0e-12);
}

TEST(Plant, SpecificForceUsesBodyFrameAttitude)
{
    const ofcsim::VehicleConfig vehicle = test_vehicle();
    const ofcsim::Propulsion propulsion(vehicle);
    const ofcsim::RigidBodyParameters parameters = test_parameters();
    const ofcsim::GroundContact contact;
    ofcsim::PlantState state{};
    state.rigid_body.attitude = ofcsim::Quat(
        Eigen::AngleAxisd(
            std::numbers::pi / 2.0,
            ofcsim::Vec3::UnitY()));

    const ofcsim::Vec3 result = ofcsim::specific_force_body_mps2(
        state, propulsion, parameters, contact);

    EXPECT_NEAR(result.x(), ofcsim::kGravityNed.z(), 1.0e-12);
    EXPECT_NEAR(result.y(), 0.0, 1.0e-12);
    EXPECT_NEAR(result.z(), 0.0, 1.0e-12);
}

TEST(PlantImuIntegration, GroundedRestProducesFirmwareSample)
{
    const ofcsim::VehicleConfig vehicle = test_vehicle();
    const ofcsim::Propulsion propulsion(vehicle);
    const ofcsim::RigidBodyParameters parameters = test_parameters();
    const ofcsim::GroundContact contact;
    const ofcsim::PlantState state{};
    ofcsim::ImuModel imu_model(42U, ideal_imu_config());

    const ofcsim::Vec3 specific_force =
        ofcsim::specific_force_body_mps2(
            state, propulsion, parameters, contact);
    const auto sample = imu_model.sample(
        specific_force,
        state.rigid_body.rate_rad_s,
        1000U);

    EXPECT_EQ(sample.acceleration_x, 0);
    EXPECT_EQ(sample.acceleration_y, 0);
    EXPECT_EQ(sample.acceleration_z, -4096);
    EXPECT_EQ(sample.gyroscope_x, 0);
    EXPECT_EQ(sample.gyroscope_y, 0);
    EXPECT_EQ(sample.gyroscope_z, 0);
    EXPECT_EQ(sample.acquired_at_us, 1000U);
    EXPECT_EQ(sample.sequence, 1U);
    EXPECT_TRUE(sample.valid);
}

TEST(PlantImuIntegration, StaticTiltMatchesFirmwareAccelerometerAttitude)
{
    const ofcsim::VehicleConfig vehicle = test_vehicle();
    const ofcsim::Propulsion propulsion(vehicle);
    const ofcsim::RigidBodyParameters parameters = test_parameters();
    const ofcsim::GroundContact contact;
    ofcsim::PlantState state{};
    state.rigid_body.attitude = ofcsim::quat_from_euler_zyx_deg(
        ofcsim::Vec3(10.0, -15.0, 0.0));
    ofcsim::ImuModel imu_model(42U, ideal_imu_config());

    const ofcsim::Vec3 specific_force =
        ofcsim::specific_force_body_mps2(
            state, propulsion, parameters, contact);
    const auto sample = imu_model.sample(
        specific_force,
        state.rigid_body.rate_rad_s,
        1000U);
    const float acceleration_g[3] = {
        static_cast<float>(sample.acceleration_x) / 4096.0F,
        static_cast<float>(sample.acceleration_y) / 4096.0F,
        static_cast<float>(sample.acceleration_z) / 4096.0F,
    };
    accelerometer_attitude_t attitude{};

    ASSERT_TRUE(accelerometer_attitude_calculate(
        acceleration_g,
        &attitude));
    EXPECT_NEAR(attitude.roll_degrees, 10.0F, 0.1F);
    EXPECT_NEAR(attitude.pitch_degrees, -15.0F, 0.1F);
}

TEST(PlantImuIntegration, StaticTiltConvergesThroughFirmwarePipeline)
{
    const ofcsim::VehicleConfig vehicle = test_vehicle();
    const ofcsim::Propulsion propulsion(vehicle);
    const ofcsim::RigidBodyParameters parameters = test_parameters();
    const ofcsim::GroundContact contact;
    ofcsim::PlantState state{};
    state.rigid_body.attitude = ofcsim::quat_from_euler_zyx_deg(
        ofcsim::Vec3(10.0, -15.0, 0.0));
    ofcsim::ImuModel imu_model(42U, ideal_imu_config());

    const ofcsim::Vec3 specific_force =
        ofcsim::specific_force_body_mps2(
            state, propulsion, parameters, contact);
    const imu_processing_config_t config = test_imu_processing_config();
    imu_processing_pipeline_t pipeline{};
    ASSERT_TRUE(imu_processing_pipeline_initialize(&pipeline, &config));
    const int32_t gyro_bias[3] = {0, 0, 0};

    for (std::uint64_t index = 0U; index < 2000U; ++index) {
        const auto sample = imu_model.sample(
            specific_force,
            state.rigid_body.rate_rad_s,
            (index + 1U) * 1000U);
        ASSERT_EQ(
            imu_processing_pipeline_process(
                &pipeline,
                &sample,
                IMU_FRESHNESS_FRESH,
                gyro_bias),
            IMU_PROCESSING_UPDATED);
    }

    attitude_snapshot_t attitude{};
    ASSERT_TRUE(imu_processing_pipeline_latest(&pipeline, &attitude));
    ASSERT_TRUE(attitude.valid);
    EXPECT_NEAR(attitude.roll_degrees, 10.0F, 0.5F);
    EXPECT_NEAR(attitude.pitch_degrees, -15.0F, 0.5F);
}

TEST(Plant, BelowHoverCommandRemainsOnGround)
{
    const ofcsim::VehicleConfig vehicle = test_vehicle();
    const ofcsim::Propulsion propulsion(vehicle);
    const ofcsim::RigidBodyParameters parameters = test_parameters();
    const ofcsim::GroundContact contact;
    ofcsim::PlantState state{};

    const double hover_speed = std::sqrt(
        vehicle.mass_kg * ofcsim::kGravityNed.z() /
        (4.0 * vehicle.motors[0].thrust_coeff));
    const double rotor_speed = 0.9 * hover_speed;
    state.rotor_speed_rad_s.fill(rotor_speed);
    const ofcsim::MotorArray command{
        rotor_speed / vehicle.motors[0].max_speed_rad_s,
        rotor_speed / vehicle.motors[1].max_speed_rad_s,
        rotor_speed / vehicle.motors[2].max_speed_rad_s,
        rotor_speed / vehicle.motors[3].max_speed_rad_s};

    for (int step = 0; step < 1000; ++step) {
        state = ofcsim::step(
            state, command, propulsion, vehicle, parameters, contact, 0.001);
    }

    EXPECT_DOUBLE_EQ(state.rigid_body.position_m.z(), 0.0);
    EXPECT_DOUBLE_EQ(state.rigid_body.velocity_mps.z(), 0.0);
}

TEST(Plant, AboveHoverCommandProducesLiftoff)
{
    const ofcsim::VehicleConfig vehicle = test_vehicle();
    const ofcsim::Propulsion propulsion(vehicle);
    const ofcsim::RigidBodyParameters parameters = test_parameters();
    const ofcsim::GroundContact contact;
    ofcsim::PlantState state{};

    const double hover_speed = std::sqrt(
        vehicle.mass_kg * ofcsim::kGravityNed.z() /
        (4.0 * vehicle.motors[0].thrust_coeff));
    const double rotor_speed = 1.1 * hover_speed;
    state.rotor_speed_rad_s.fill(rotor_speed);
    const ofcsim::MotorArray command{
        rotor_speed / vehicle.motors[0].max_speed_rad_s,
        rotor_speed / vehicle.motors[1].max_speed_rad_s,
        rotor_speed / vehicle.motors[2].max_speed_rad_s,
        rotor_speed / vehicle.motors[3].max_speed_rad_s};

    for (int step = 0; step < 1000; ++step) {
        state = ofcsim::step(
            state, command, propulsion, vehicle, parameters, contact, 0.001);
    }

    EXPECT_LT(state.rigid_body.position_m.z(), 0.0);
    EXPECT_LT(state.rigid_body.velocity_mps.z(), 0.0);
}
