#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace ofcsim {

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
using Quat = Eigen::Quaterniond;
using Vec4 = Eigen::Vector4d;

// World frame: NED coordinates (x north, y east, z down), metres and seconds.
// Body frame: FRD coordinates (x front, y right, z down).
inline const Vec3 kGravityNed{0.0, 0.0, 9.80665};

struct RigidBodyState {
    Vec3 position_m = Vec3::Zero();
    Vec3 velocity_mps = Vec3::Zero();
    Quat attitude = Quat::Identity();  // body -> world rotation
    Vec3 rate_rad_s = Vec3::Zero();    // body p, q, r in rad/s
};

struct StateDerivative {
    Vec3 position_mps = Vec3::Zero();
    Vec3 velocity_mps2 = Vec3::Zero();
    Vec4 attitude_wxyz = Vec4::Zero();
    Vec3 rate_rad_s2 = Vec3::Zero();
};

struct BodyWrench {
    Vec3 force_n = Vec3::Zero();   // total force in the body frame
    Vec3 torque_nm = Vec3::Zero(); // total torque about the centre of mass
};

struct RigidBodyParameters {
    double mass_kg;
    Mat3 inertia_kgm2;
    Mat3 inertia_inv_kgm2;
};

[[nodiscard]] StateDerivative derivative(
    const RigidBodyState& state,
    const BodyWrench& wrench,
    const RigidBodyParameters& parameters);

[[nodiscard]] RigidBodyState advance(
    const RigidBodyState& state,
    const StateDerivative& derivative,
    double dt_s);

[[nodiscard]] RigidBodyState rk4_step(
    const RigidBodyState& state,
    const BodyWrench& wrench,
    const RigidBodyParameters& parameters,
    double dt_s);

// Returns aerospace ZYX Euler angles as {roll, pitch, yaw} in degrees.
[[nodiscard]] Vec3 euler_zyx_deg(const Quat& attitude);

// Constructs a body-to-world quaternion from aerospace ZYX angles in degrees.
[[nodiscard]] Quat quat_from_euler_zyx_deg(const Vec3& angles_deg);

inline StateDerivative operator+(
      const StateDerivative& lhs,
      const StateDerivative& rhs)
  {
      StateDerivative result{};
      result.position_mps = lhs.position_mps + rhs.position_mps;
      result.velocity_mps2 = lhs.velocity_mps2 + rhs.velocity_mps2;
      result.attitude_wxyz = lhs.attitude_wxyz + rhs.attitude_wxyz;
      result.rate_rad_s2 = lhs.rate_rad_s2 + rhs.rate_rad_s2;
      return result;
  }


  inline StateDerivative operator*(
      const StateDerivative& value,
      double scale)
  {
      StateDerivative result{};
      result.position_mps = scale * value.position_mps;
      result.velocity_mps2 = scale * value.velocity_mps2;
      result.attitude_wxyz = scale * value.attitude_wxyz;
      result.rate_rad_s2 = scale * value.rate_rad_s2;
      return result;
  }

  inline StateDerivative operator*(
      double scale,
      const StateDerivative& value)
  {
      return value * scale;
  }

  inline StateDerivative operator/(
      const StateDerivative& value,
      double divisor)
  {
      return value * (1.0 / divisor);
  }


}  // namespace ofcsim
