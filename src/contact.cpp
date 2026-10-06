#include "ofcsim/contact.hpp"

namespace ofcsim {

bool GroundContact::on_ground(const RigidBodyState& state) const
{
    constexpr double kContactToleranceM = 1.0e-9;
    return state.position_m.z() >= ground_z_m_ - kContactToleranceM;
}

void GroundContact::enforce(RigidBodyState& state) const
{
    if (state.position_m.z() < ground_z_m_) {
        return;
    }

    state.position_m.z() = ground_z_m_;
    if (state.velocity_mps.z() > 0.0) {
        state.velocity_mps.z() = 0.0;
    }

    state.velocity_mps.x() = 0.0;
    state.velocity_mps.y() = 0.0;
    state.rate_rad_s.setZero();
}

Vec3 GroundContact::constrain_acceleration(
    const RigidBodyState& state,
    Vec3 acceleration_world) const
{
    if (!on_ground(state)) {
        return acceleration_world;
    }

    if (acceleration_world.z() > 0.0) {
        acceleration_world.z() = 0.0;
    }
    acceleration_world.x() = 0.0;
    acceleration_world.y() = 0.0;
    return acceleration_world;
}

}  // namespace ofcsim
