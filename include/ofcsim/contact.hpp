#pragma once

#include "ofcsim/rigid_body.hpp"

namespace ofcsim {

// Simple v1 ground constraint. The ground is a horizontal plane in the NED
// world frame; positive z points into the ground.
class GroundContact {
public:
    explicit GroundContact(double ground_z_m = 0.0)
        : ground_z_m_(ground_z_m)
    {
    }

    [[nodiscard]] bool on_ground(const RigidBodyState& state) const;

    // Clamp penetration and remove motion that cannot occur on the static
    // ground plane. This v1 model has no bounce, sliding, or tipping.
    void enforce(RigidBodyState& state) const;

    // Apply the ground's normal/friction constraints to world acceleration.
    [[nodiscard]] Vec3 constrain_acceleration(
        const RigidBodyState& state,
        Vec3 acceleration_world) const;

private:
    double ground_z_m_;
};

}  // namespace ofcsim
