#include "ofcsim/propulsion.hpp"

#include <cmath>

#include <gtest/gtest.h>

namespace {

ofcsim::VehicleConfig test_vehicle()
{
    ofcsim::VehicleConfig vehicle{};
    vehicle.mass_kg = 2.0;
    vehicle.inertia_kgm2 = ofcsim::Mat3::Identity();
    vehicle.inertia_inv_kgm2 = ofcsim::Mat3::Identity();

    vehicle.motors[0].position_m = ofcsim::Vec3(1.0, -1.0, 0.0);
    vehicle.motors[1].position_m = ofcsim::Vec3(-1.0, -1.0, 0.0);
    vehicle.motors[2].position_m = ofcsim::Vec3(1.0, 1.0, 0.0);
    vehicle.motors[3].position_m = ofcsim::Vec3(-1.0, 1.0, 0.0);

    vehicle.motors[0].spin_direction = 1;
    vehicle.motors[1].spin_direction = -1;
    vehicle.motors[2].spin_direction = -1;
    vehicle.motors[3].spin_direction = 1;

    for (ofcsim::MotorConfig& motor : vehicle.motors) {
        motor.thrust_coeff = 2.0;
        motor.torque_coeff = 0.5;
        motor.max_speed_rad_s = 100.0;
        motor.time_constant_s = 0.1;
    }
    return vehicle;
}

}  // namespace

TEST(Propulsion, ConvertsCommandToRotorSpeed)
{
    const ofcsim::Propulsion propulsion(test_vehicle());

    EXPECT_DOUBLE_EQ(propulsion.commanded_speed_rad_s(0, 0.5), 50.0);
    EXPECT_DOUBLE_EQ(propulsion.commanded_speed_rad_s(0, -1.0), 0.0);
    EXPECT_DOUBLE_EQ(propulsion.commanded_speed_rad_s(0, 2.0), 100.0);
}

TEST(Propulsion, ZeroRotorSpeedProducesZeroWrench)
{
    const ofcsim::Propulsion propulsion(test_vehicle());

    const ofcsim::BodyWrench result = propulsion.wrench({0.0, 0.0, 0.0, 0.0});

    EXPECT_TRUE(result.force_n.isApprox(ofcsim::Vec3::Zero()));
    EXPECT_TRUE(result.torque_nm.isApprox(ofcsim::Vec3::Zero()));
}

TEST(Propulsion, EqualRotorSpeedsProduceVerticalThrustOnly)
{
    const ofcsim::Propulsion propulsion(test_vehicle());

    const ofcsim::BodyWrench result = propulsion.wrench({1.0, 1.0, 1.0, 1.0});

    EXPECT_TRUE(result.force_n.isApprox(ofcsim::Vec3(0.0, 0.0, -8.0)));
    EXPECT_TRUE(result.torque_nm.isApprox(ofcsim::Vec3::Zero()));
}

TEST(Propulsion, SingleMotorProducesMomentFromOffsetThrust)
{
    const ofcsim::Propulsion propulsion(test_vehicle());

    const ofcsim::BodyWrench result = propulsion.wrench({1.0, 0.0, 0.0, 0.0});

    EXPECT_TRUE(result.force_n.isApprox(ofcsim::Vec3(0.0, 0.0, -2.0)));
    EXPECT_TRUE(result.torque_nm.isApprox(ofcsim::Vec3(2.0, 2.0, -0.5)));
}

TEST(Propulsion, RotorSpinDirectionProducesYawTorque)
{
    const ofcsim::Propulsion propulsion(test_vehicle());

    const ofcsim::BodyWrench result = propulsion.wrench({0.0, 1.0, 0.0, 0.0});

    EXPECT_DOUBLE_EQ(result.torque_nm.z(), 0.5);
}

TEST(Propulsion, HoverSpeedBalancesMeasuredVehicleMass)
{
    ofcsim::VehicleConfig vehicle = test_vehicle();
    vehicle.mass_kg = 0.668;
    vehicle.motors[0].position_m = ofcsim::Vec3(0.0725, -0.0975, 0.0);
    vehicle.motors[1].position_m = ofcsim::Vec3(-0.0725, -0.0975, 0.0);
    vehicle.motors[2].position_m = ofcsim::Vec3(0.0725, 0.0975, 0.0);
    vehicle.motors[3].position_m = ofcsim::Vec3(-0.0725, 0.0975, 0.0);
    for (ofcsim::MotorConfig& motor : vehicle.motors) {
        motor.thrust_coeff = 1.6e-6; // initial estimate
    }

    const ofcsim::Propulsion propulsion(vehicle);
    const double hover_speed = std::sqrt(
        vehicle.mass_kg * ofcsim::kGravityNed.z() /
        (4.0 * vehicle.motors[0].thrust_coeff));
    const ofcsim::BodyWrench result = propulsion.wrench({
        hover_speed, hover_speed, hover_speed, hover_speed});

    EXPECT_NEAR(result.force_n.z(),
                -vehicle.mass_kg * ofcsim::kGravityNed.z(),
                1.0e-12);
    EXPECT_TRUE(result.force_n.head<2>().isApprox(
        ofcsim::Vec3::Zero().head<2>(), 1.0e-12));
    EXPECT_TRUE(result.torque_nm.isApprox(
        ofcsim::Vec3::Zero(), 1.0e-12));
}
