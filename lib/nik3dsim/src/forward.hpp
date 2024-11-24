#pragma once
#include "types.hpp"
#include "math.hpp"

namespace nik3dsim {
    // RigidBody operations
    void rigidbody_init(RigidBodyModel* bm, RigidBodyData* bd, BodyType type, niknum size[3], niknum density, niknum pos[3], niknum angles[3]);
    void rigidbody_integrate(RigidBodyModel* bm, RigidBodyData* bd, niknum dt, niknum gravity[3]);

    // Simulator operations
    void simulator_init(nikModel* m, niknum gravity[3], niknum timeStepSize, int numPosIters);
    void simulator_simulate(nikModel* m, nikData* d);
    void simulator_destroy(nikModel* m, nikData* d);

    // Utility functions
    void print_simulation_state(nikModel* m, nikData* d);
} // namespace nik3dsim