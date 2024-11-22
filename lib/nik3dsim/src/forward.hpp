#pragma once
#include "types.hpp"
#include "math.hpp"

namespace nik3dsim {
    // RigidBody operations
    void rigidbody_init(RigidBody* body, BodyType type, niknum size[3], niknum density, niknum pos[3], niknum angles[3]);
    void rigidbody_integrate(RigidBody* body, niknum dt, niknum gravity[3]);

    // Simulator operations
    void simulator_init(RigidBodySimulator* sim, niknum gravity[3], niknum timeStepSize, int numPosIters);
    void simulator_simulate(RigidBodySimulator* sim);
    void simulator_destroy(RigidBodySimulator* sim);

    // Utility functions
    void print_simulation_state(RigidBodySimulator* sim);
} // namespace nik3dsim