#include "ofcsim/rigid_body.hpp"

#include <gtest/gtest.h>

TEST(RigidBody, DerivativeCalculatesGravity)
{
    const ofcsim::RigidBodyState state{};
    const ofcsim::BodyWrench wrench{};
    const ofcsim::RigidBodyParameters parameters{
        .mass_kg = 1.0,
        .inertia_kgm2 = ofcsim::Mat3::Identity(),
        .inertia_inv_kgm2 = ofcsim::Mat3::Identity(),
    };

    const ofcsim::StateDerivative result =
        ofcsim::derivative(state, wrench, parameters);

    EXPECT_DOUBLE_EQ(result.position_mps.x(), 0.0);
    EXPECT_DOUBLE_EQ(result.position_mps.y(), 0.0);
    EXPECT_DOUBLE_EQ(result.position_mps.z(), 0.0);

    EXPECT_DOUBLE_EQ(result.velocity_mps2.x(), 0.0);
    EXPECT_DOUBLE_EQ(result.velocity_mps2.y(), 0.0);
    EXPECT_DOUBLE_EQ(result.velocity_mps2.z(), ofcsim::kGravityNed.z());

    EXPECT_TRUE(result.attitude_wxyz.isZero(1.0e-12));
    EXPECT_TRUE(result.rate_rad_s2.isZero(1.0e-12));
}

TEST(RigidBody, AdvanceAppliesDerivativeZeroAttitudeDerivative)
{
    ofcsim::RigidBodyState state{};
    state.position_m = ofcsim::Vec3(1.0, 2.0, 3.0);
    state.velocity_mps = ofcsim::Vec3(4.0, 5.0, 6.0);
    state.rate_rad_s = ofcsim::Vec3(0.1, 0.2, 0.3);

    ofcsim::StateDerivative derivative{};
    derivative.position_mps = ofcsim::Vec3(10.0, 20.0, 30.0);
    derivative.velocity_mps2 = ofcsim::Vec3(2.0, 3.0, 4.0);
    derivative.rate_rad_s2 = ofcsim::Vec3(1.0, 2.0, 3.0);

    const double dt_s = 0.1;

    const ofcsim::RigidBodyState result =
        ofcsim::advance(state, derivative, dt_s);

    EXPECT_NEAR(result.position_m.x(), 2.0, 1.0e-12);
    EXPECT_NEAR(result.position_m.y(), 4.0, 1.0e-12);
    EXPECT_NEAR(result.position_m.z(), 6.0, 1.0e-12);

    EXPECT_NEAR(result.velocity_mps.x(), 4.2, 1.0e-12);
    EXPECT_NEAR(result.velocity_mps.y(), 5.3, 1.0e-12);
    EXPECT_NEAR(result.velocity_mps.z(), 6.4, 1.0e-12);

    EXPECT_NEAR(result.rate_rad_s.x(), 0.2, 1.0e-12);
    EXPECT_NEAR(result.rate_rad_s.y(), 0.4, 1.0e-12);
    EXPECT_NEAR(result.rate_rad_s.z(), 0.6, 1.0e-12);

    EXPECT_TRUE(result.attitude.isApprox(
        ofcsim::Quat::Identity(), 1.0e-12));
}


TEST(RigidBody, AdvanceAppliesDerivativeQuaternionUpdate)
{
    ofcsim::RigidBodyState state{};
    state.position_m = ofcsim::Vec3(1.0, 2.0, 3.0);
    state.velocity_mps = ofcsim::Vec3(4.0, 5.0, 6.0);
    state.rate_rad_s = ofcsim::Vec3(0.1, 0.2, 0.3);

    ofcsim::StateDerivative derivative{};
    derivative.position_mps = ofcsim::Vec3(10.0, 20.0, 30.0);
    derivative.velocity_mps2 = ofcsim::Vec3(2.0, 3.0, 4.0);
    derivative.rate_rad_s2 = ofcsim::Vec3(1.0, 2.0, 3.0);
    derivative.attitude_wxyz = ofcsim::Vec4(0.0, 0.0, 0.0, 0.5);

    const double dt_s = 0.1;

    const ofcsim::RigidBodyState result =
        ofcsim::advance(state, derivative, dt_s);

    EXPECT_NEAR(result.position_m.x(), 2.0, 1.0e-12);
    EXPECT_NEAR(result.position_m.y(), 4.0, 1.0e-12);
    EXPECT_NEAR(result.position_m.z(), 6.0, 1.0e-12);

    EXPECT_NEAR(result.velocity_mps.x(), 4.2, 1.0e-12);
    EXPECT_NEAR(result.velocity_mps.y(), 5.3, 1.0e-12);
    EXPECT_NEAR(result.velocity_mps.z(), 6.4, 1.0e-12);

    EXPECT_NEAR(result.rate_rad_s.x(), 0.2, 1.0e-12);
    EXPECT_NEAR(result.rate_rad_s.y(), 0.4, 1.0e-12);
    EXPECT_NEAR(result.rate_rad_s.z(), 0.6, 1.0e-12);

    // Check the attitude.
    EXPECT_NEAR(result.attitude.w(), 0.9987523388778446, 1.0e-12);
    EXPECT_NEAR(result.attitude.x(), 0.0, 1.0e-12);
    EXPECT_NEAR(result.attitude.y(), 0.0, 1.0e-12);
    EXPECT_NEAR(result.attitude.z(), 0.0499376169438922, 1.0e-12);
    // Check that normalization worked.
    EXPECT_NEAR(result.attitude.norm(), 1.0, 1.0e-12);
}


TEST(RigidBody, Rk4FreeFallMatchesAnalyticSolution)
{
    ofcsim::RigidBodyState state{};
    state.position_m = ofcsim::Vec3::Zero();
    state.velocity_mps = ofcsim::Vec3::Zero();
    state.rate_rad_s = ofcsim::Vec3::Zero();
    state.attitude = ofcsim::Quat::Identity();

    ofcsim::BodyWrench wrench{};
    wrench.force_n = ofcsim::Vec3::Zero();
    wrench.torque_nm = ofcsim::Vec3::Zero();

    ofcsim::RigidBodyParameters parameters{};
    parameters.mass_kg = 1.0;
    parameters.inertia_kgm2 = ofcsim::Mat3::Identity();
    parameters.inertia_inv_kgm2 = ofcsim::Mat3::Identity();

    const double dt = 1.0;

    const ofcsim::RigidBodyState result =
        ofcsim::rk4_step(state, wrench, parameters, dt);

    EXPECT_NEAR(result.position_m.x(), 0.0, 1.0e-12);
    EXPECT_NEAR(result.position_m.y(), 0.0, 1.0e-12);
    EXPECT_NEAR(result.position_m.z(), 4.903325, 1.0e-12);

    EXPECT_NEAR(result.velocity_mps.x(), 0.0, 1.0e-12);
    EXPECT_NEAR(result.velocity_mps.y(), 0.0, 1.0e-12);
    EXPECT_NEAR(result.velocity_mps.z(), 9.80665, 1.0e-12);

    EXPECT_TRUE(result.attitude.isApprox(
        ofcsim::Quat::Identity(), 1.0e-12));
    EXPECT_TRUE(result.rate_rad_s.isApprox(
        ofcsim::Vec3::Zero(), 1.0e-12));
}
