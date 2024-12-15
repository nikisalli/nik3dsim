#pragma once
#include "types.hpp"
#include "math.hpp"

namespace nik3dsim {
    // RigidBody operations
    void add_rigidbody(nikModel* m, BodyType type, niknum size[3], niknum density, niknum pos[3], niknum angles[3], niknum compliance, niknum friction, unsigned int contype, unsigned int conaffinity);
    void add_staticbody(nikModel* m, BodyType type, niknum size[3], niknum pos[3], niknum angles[3], niknum compliance, niknum friction, unsigned int contype, unsigned int conaffinity);
    void add_distance_constraint(nikModel* m, unsigned int b0, unsigned int b1, niknum r0[3], niknum r1[3], niknum compliance, niknum distance);
    void add_hinge_constraint(nikModel* m, unsigned int b0, unsigned int b1, niknum a0[3], niknum a1[3], niknum compliance);

    // Simulator operations
    void simulator_init(nikModel* m, niknum gravity[3], niknum timeStepSize, int numPosIters, niknum damping);
    void data_init(nikModel* m, nikData* d);
    void simulator_step(nikModel* m, nikData* d);
    void simulator_destroy(nikModel* m, nikData* d);

    // Utility functions
    void print_simulation_state(nikModel* m, nikData* d);
} // namespace nik3dsim