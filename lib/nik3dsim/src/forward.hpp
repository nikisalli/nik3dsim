#pragma once

#include "types.hpp"
#include "math.hpp"

namespace nik3dsim {
    // RigidBody operations
    void rigidbody_init(RigidBody* body, BodyType type, Vec3 size, float density, Vec3 pos, Vec3 angles);
    void rigidbody_integrate(RigidBody* body, float dt, Vec3 gravity);

    // Simulator operations
    void simulator_init(RigidBodySimulator* sim, Vec3 gravity, float timeStepSize, int numPosIters);
    void simulator_add_body(RigidBodySimulator* sim, RigidBody* body);
    void simulator_simulate(RigidBodySimulator* sim);
    void simulator_destroy(RigidBodySimulator* sim);

    // Utility functions
    void print_simulation_state(RigidBodySimulator* sim);

}  // namespace nik3dsim