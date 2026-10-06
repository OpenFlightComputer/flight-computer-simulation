#include "ofcsim/contact.hpp"
#include "ofcsim/plant.hpp"

#include <gtest/gtest.h>

namespace ofcsim {
namespace {

TEST(GroundContact, AirborneStateIsUnchanged)
{
    GroundContact contact;
    RigidBodyState state{};
    state.position_m = Vec3(1.0, 2.0, -0.1);
    state.velocity_mps = Vec3(3.0, 4.0, -5.0);
    state.rate_rad_s = Vec3(0.1, 0.2, 0.3);

    contact.enforce(state);

    EXPECT_TRUE(state.position_m.isApprox(Vec3(1.0, 2.0, -0.1)));
    EXPECT_TRUE(state.velocity_mps.isApprox(Vec3(3.0, 4.0, -5.0)));
    EXPECT_TRUE(state.rate_rad_s.isApprox(Vec3(0.1, 0.2, 0.3)));
    EXPECT_FALSE(contact.on_ground(state));
}

TEST(GroundContact, EnforceClampsPenetrationAndContactMotion)
{
    GroundContact contact;
    RigidBodyState state{};
    state.position_m = Vec3(1.0, 2.0, 0.2);
    state.velocity_mps = Vec3(3.0, 4.0, 5.0);
    state.rate_rad_s = Vec3(0.1, 0.2, 0.3);

    contact.enforce(state);

    EXPECT_DOUBLE_EQ(state.position_m.z(), 0.0);
    EXPECT_DOUBLE_EQ(state.velocity_mps.x(), 0.0);
    EXPECT_DOUBLE_EQ(state.velocity_mps.y(), 0.0);
    EXPECT_DOUBLE_EQ(state.velocity_mps.z(), 0.0);
    EXPECT_TRUE(state.rate_rad_s.isZero());
}

TEST(GroundContact, UpwardVelocityIsPreservedAtGround)
{
    GroundContact contact;
    RigidBodyState state{};
    state.position_m.z() = 0.0;
    state.velocity_mps = Vec3(0.0, 0.0, -1.0);

    contact.enforce(state);

    EXPECT_DOUBLE_EQ(state.position_m.z(), 0.0);
    EXPECT_DOUBLE_EQ(state.velocity_mps.z(), -1.0);
}

TEST(GroundContact, DownwardAccelerationIsCancelledAtGround)
{
    GroundContact contact;
    RigidBodyState state{};
    state.position_m.z() = 0.0;

    const Vec3 constrained = contact.constrain_acceleration(
        state, Vec3(2.0, -3.0, 4.0));

    EXPECT_TRUE(constrained.isApprox(Vec3::Zero()));
}

TEST(GroundContact, UpwardAccelerationCausesLiftoff)
{
    GroundContact contact;
    RigidBodyState state{};
    state.position_m.z() = 0.0;

    const Vec3 constrained = contact.constrain_acceleration(
        state, Vec3(2.0, -3.0, -4.0));

    EXPECT_TRUE(constrained.isApprox(Vec3(0.0, 0.0, -4.0)));
}

TEST(GroundContact, AirborneAccelerationIsUnchanged)
{
    GroundContact contact;
    RigidBodyState state{};
    state.position_m.z() = -0.1;
    const Vec3 acceleration(2.0, -3.0, 4.0);

    EXPECT_TRUE(contact.constrain_acceleration(state, acceleration)
                    .isApprox(acceleration));
}

TEST(GroundContact, PlantStepAppliesContactAfterIntegration)
{
    VehicleConfig vehicle{};
    vehicle.mass_kg = 1.0;
    vehicle.inertia_kgm2 = Mat3::Identity();
    vehicle.inertia_inv_kgm2 = Mat3::Identity();
    for (MotorConfig& motor : vehicle.motors) {
        motor.position_m = Vec3::Zero();
        motor.spin_direction = 1;
        motor.thrust_coeff = 1.0;
        motor.torque_coeff = 1.0;
        motor.max_speed_rad_s = 1.0;
        motor.time_constant_s = 1.0;
    }

    const Propulsion propulsion(vehicle);
    const RigidBodyParameters rigid_body_parameters{
        vehicle.mass_kg,
        vehicle.inertia_kgm2,
        vehicle.inertia_inv_kgm2};
    GroundContact contact;
    PlantState state{};
    state.rigid_body.position_m.z() = 0.0;

    const PlantState result = step(
        state,
        MotorArray{0.0, 0.0, 0.0, 0.0},
        propulsion,
        vehicle,
        rigid_body_parameters,
        contact,
        0.01);

    EXPECT_DOUBLE_EQ(result.rigid_body.position_m.z(), 0.0);
    EXPECT_DOUBLE_EQ(result.rigid_body.velocity_mps.z(), 0.0);
}

}  // namespace
}  // namespace ofcsim
