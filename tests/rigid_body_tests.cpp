#include "ofcsim/rigid_body.hpp"

#include <cmath>
#include <cstddef>

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

    EXPECT_NEAR(result.attitude.w(), 0.9987523388778446, 1.0e-12);
    EXPECT_NEAR(result.attitude.x(), 0.0, 1.0e-12);
    EXPECT_NEAR(result.attitude.y(), 0.0, 1.0e-12);
    EXPECT_NEAR(result.attitude.z(), 0.0499376169438922, 1.0e-12);
    EXPECT_NEAR(result.attitude.norm(), 1.0, 1.0e-12);
}

TEST(RigidBody, Rk4FreeFallMatchesAnalyticSolution)
{
    ofcsim::RigidBodyState state{};
    ofcsim::BodyWrench wrench{};
    ofcsim::RigidBodyParameters parameters{};
    parameters.mass_kg = 1.0;
    parameters.inertia_kgm2 = ofcsim::Mat3::Identity();
    parameters.inertia_inv_kgm2 = ofcsim::Mat3::Identity();

    const double dt_s = 1.0;
    const ofcsim::RigidBodyState result =
        ofcsim::rk4_step(state, wrench, parameters, dt_s);

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

TEST(RigidBody, Rk4ConstantTorqueSpinUp)
{
    ofcsim::RigidBodyState state{};
    ofcsim::BodyWrench wrench{};
    wrench.torque_nm = ofcsim::Vec3(1.0, 0.0, 0.0);

    ofcsim::RigidBodyParameters parameters{};
    parameters.mass_kg = 1.0;
    parameters.inertia_kgm2 =
        ofcsim::Vec3(2.0, 3.0, 4.0).asDiagonal();
    parameters.inertia_inv_kgm2 = parameters.inertia_kgm2.inverse();

    const ofcsim::RigidBodyState result =
        ofcsim::rk4_step(state, wrench, parameters, 1.0);

    EXPECT_NEAR(result.rate_rad_s.x(), 0.5, 1.0e-12);
    EXPECT_NEAR(result.rate_rad_s.y(), 0.0, 1.0e-12);
    EXPECT_NEAR(result.rate_rad_s.z(), 0.0, 1.0e-12);
    EXPECT_NEAR(result.attitude.norm(), 1.0, 1.0e-12);
}

TEST(RigidBody, Rk4QuaternionNormalizationDuringRotation)
{
    ofcsim::RigidBodyState state{};
    state.rate_rad_s = ofcsim::Vec3(0.0, 0.0, 1.0);

    const ofcsim::BodyWrench wrench{};
    const ofcsim::RigidBodyParameters parameters{
        .mass_kg = 1.0,
        .inertia_kgm2 = ofcsim::Mat3::Identity(),
        .inertia_inv_kgm2 = ofcsim::Mat3::Identity(),
    };

    for (std::size_t i = 0; i < 1000; ++i) {
        state = ofcsim::rk4_step(state, wrench, parameters, 0.001);
        EXPECT_NEAR(state.attitude.norm(), 1.0, 1.0e-12);
    }
}

//Drone tumbeling freely. Angular rates change but rotational and workd frome angulare momentum stay constant.
TEST(RigidBody, Rk4TorqueFreeTumblingConservesInvariants)
{
    ofcsim::RigidBodyState state{};
    state.rate_rad_s = ofcsim::Vec3(0.4, 0.7, 1.0);

    const ofcsim::BodyWrench wrench{};
    ofcsim::RigidBodyParameters parameters{};
    parameters.mass_kg = 1.0;
    parameters.inertia_kgm2 = ofcsim::Vec3(2.0, 3.0, 4.0).asDiagonal();
    parameters.inertia_inv_kgm2 = parameters.inertia_kgm2.inverse();

    //Calculate rotational energy and initial momentum
    const auto rotational_energy =
        [&](const ofcsim::RigidBodyState& current_state) {
            return 0.5 * current_state.rate_rad_s.dot(
                parameters.inertia_kgm2 * current_state.rate_rad_s);
        };

    const double initial_energy = rotational_energy(state);
    const ofcsim::Vec3 initial_momentum = state.attitude * (parameters.inertia_kgm2 * state.rate_rad_s);

    //Run for 10 secs
    for (std::size_t i = 0; i < 10000; ++i) {
        state = ofcsim::rk4_step(state, wrench, parameters, 0.001);
        EXPECT_NEAR(state.attitude.norm(), 1.0, 1.0e-12);
    }

    //Calculate energies now
    const double final_energy = rotational_energy(state);
    const ofcsim::Vec3 final_momentum = state.attitude * (parameters.inertia_kgm2 * state.rate_rad_s);

    //Compare that both stay nearly the same
    EXPECT_NEAR(final_energy, initial_energy, 1.0e-6);
    EXPECT_TRUE(final_momentum.isApprox(initial_momentum, 1.0e-6));
}


// Same tumbling case as Rk4TorqueFreeTumblingConservesInvariants, calculated two ways.
//RK4 produces less artifical energy drift than Euler
TEST(RigidBody, Rk4HasLowerEnergyDriftThanEuler)
{
    ofcsim::RigidBodyState rk4_state{};
    rk4_state.rate_rad_s = ofcsim::Vec3(0.4, 0.7, 1.0);
    ofcsim::RigidBodyState euler_state = rk4_state;

    const ofcsim::BodyWrench wrench{};
    ofcsim::RigidBodyParameters parameters{};
    parameters.mass_kg = 1.0;
    parameters.inertia_kgm2 = ofcsim::Vec3(2.0, 3.0, 4.0).asDiagonal();
    parameters.inertia_inv_kgm2 = parameters.inertia_kgm2.inverse();

    //Calculate rotational energy before
    const auto rotational_energy =
        [&](const ofcsim::RigidBodyState& current_state) {
            return 0.5 * current_state.rate_rad_s.dot(
                parameters.inertia_kgm2 * current_state.rate_rad_s);
        };
    const double initial_energy = rotational_energy(rk4_state);


    //Euler step
    const auto euler_step =
        [&](const ofcsim::RigidBodyState& current_state) {
            return ofcsim::advance(
                current_state,
                ofcsim::derivative(current_state, wrench, parameters),
                0.001);
        };


    for (std::size_t i = 0; i < 10000; ++i) {
        //Calculate using RK4
        rk4_state = ofcsim::rk4_step(
            rk4_state, wrench, parameters, 0.001);
        //Calculate using euler
        euler_state = euler_step(euler_state);
    }
    //Calculate Drift
    const double rk4_energy_drift = std::abs(rotational_energy(rk4_state) - initial_energy);
    const double euler_energy_drift = std::abs(rotational_energy(euler_state) - initial_energy);
    //Check that Euler drift is highers
    EXPECT_LT(rk4_energy_drift, euler_energy_drift / 100.0);
}

TEST(RigidBody, EulerZyxIdentityReturnsZero)
{
    const ofcsim::Quat attitude(1.0, 0.0, 0.0, 0.0);

    const ofcsim::Vec3 result = ofcsim::euler_zyx_deg(attitude);

    EXPECT_TRUE(result.isApprox(ofcsim::Vec3::Zero(), 1.0e-9));
}

TEST(RigidBody, EulerZyxExtractsPureRoll)
{
    const ofcsim::Quat attitude(0.9659258263, 0.2588190451, 0.0, 0.0);

    const ofcsim::Vec3 result = ofcsim::euler_zyx_deg(attitude);

    EXPECT_TRUE(result.isApprox(ofcsim::Vec3(30.0, 0.0, 0.0), 1.0e-9));
}

TEST(RigidBody, EulerZyxExtractsPurePitch)
{
    const ofcsim::Quat attitude(0.9848077530, 0.0, 0.1736481777, 0.0);

    const ofcsim::Vec3 result = ofcsim::euler_zyx_deg(attitude);

    EXPECT_TRUE(result.isApprox(ofcsim::Vec3(0.0, 20.0, 0.0), 1.0e-9));
}

TEST(RigidBody, EulerZyxExtractsPureYaw)
{
    const ofcsim::Quat attitude(0.9238795325, 0.0, 0.0, 0.3826834324);
    const ofcsim::Vec3 result = ofcsim::euler_zyx_deg(attitude);

    EXPECT_TRUE(result.isApprox(ofcsim::Vec3(0.0, 0.0, 45.0), 1.0e-9));
}
