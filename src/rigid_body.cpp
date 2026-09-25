#include "ofcsim/rigid_body.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace ofcsim {

StateDerivative derivative(
    const RigidBodyState& state,
    const BodyWrench& wrench,
    const RigidBodyParameters& parameters)
{
    StateDerivative result{};

    result.position_mps = state.velocity_mps;

    // The motor force is in the body frame. Rotate it into the world frame
    // so it can be combined with gravity.
    const Vec3 force_world = state.attitude * wrench.force_n;
    result.velocity_mps2 =
        force_world / parameters.mass_kg + kGravityNed;

    // Represent body angular velocity as a pure quaternion for the
    // quaternion kinematics equation.
    const Quat angular_rate_quaternion(
        0.0,
        state.rate_rad_s.x(),
        state.rate_rad_s.y(),
        state.rate_rad_s.z());
    const Quat quaternion_rate =
        state.attitude * angular_rate_quaternion;
    result.attitude_wxyz = 0.5 * Vec4(
        quaternion_rate.w(),
        quaternion_rate.x(),
        quaternion_rate.y(),
        quaternion_rate.z());

    // Apply Euler's rigid-body rotational equation. The inertia is currently
    // supplied directly by the caller.
    const Vec3 angular_momentum =
        parameters.inertia_kgm2 * state.rate_rad_s;
    const Vec3 gyroscopic_term =
        state.rate_rad_s.cross(angular_momentum);
    result.rate_rad_s2 = parameters.inertia_inv_kgm2 *
        (wrench.torque_nm - gyroscopic_term);

    return result;
}

RigidBodyState advance(
    const RigidBodyState& state,
    const StateDerivative& derivative,
    double dt_s)
{
    RigidBodyState result = state;

    result.position_m += dt_s * derivative.position_mps;
    result.velocity_mps += dt_s * derivative.velocity_mps2;
    result.rate_rad_s += dt_s * derivative.rate_rad_s2;

    // Eigen stores coeffs() as x, y, z, w, while this derivative uses w, x, y, z.
    const Vec4 attitude_wxyz =
        Vec4(
            state.attitude.w(),
            state.attitude.x(),
            state.attitude.y(),
            state.attitude.z())
        + dt_s * derivative.attitude_wxyz;

    result.attitude = Quat(
        attitude_wxyz[0],
        attitude_wxyz[1],
        attitude_wxyz[2],
        attitude_wxyz[3]);

    result.attitude.normalize();

    return result;
}

RigidBodyState rk4_step(
    const RigidBodyState& state,
    const BodyWrench& wrench,
    const RigidBodyParameters& parameters,
    double dt_s)
{
    const auto derivative_function =
        [&](const RigidBodyState& current_state) {
            return derivative(current_state, wrench, parameters);
        };

    const StateDerivative k1 = derivative_function(state);
    const StateDerivative k2 =
        derivative_function(advance(state, k1, dt_s / 2.0));
    const StateDerivative k3 =
        derivative_function(advance(state, k2, dt_s / 2.0));
    const StateDerivative k4 =
        derivative_function(advance(state, k3, dt_s));

    const StateDerivative average =
        (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0;

    return advance(state, average, dt_s);
}

Vec3 euler_zyx_deg(const Quat& attitude)
{
    const double w = attitude.w();
    const double x = attitude.x();
    const double y = attitude.y();
    const double z = attitude.z();

    const double roll = std::atan2(
        2.0 * (w * x + y * z),
        1.0 - 2.0 * (x * x + y * y));
    const double pitch = std::asin(std::clamp(
        2.0 * (w * y - z * x), -1.0, 1.0));
    const double yaw = std::atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z));

    return Vec3(roll, pitch, yaw) *
        (180.0 / std::numbers::pi);
}

}  // namespace ofcsim
