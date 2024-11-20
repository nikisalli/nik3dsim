#pragma once

#include <stdlib.h>

#include "math.hpp"

namespace nik3dsim {
    // Body type enum
    typedef enum {
        BODY_SPHERE,
        BODY_BOX
    } BodyType;

    // RigidBody struct
    typedef struct RigidBody {
        BodyType type;
        niknum size[3];
        
        niknum pos[3];
        niknum rot[4];
        niknum vel[3];
        niknum omega[3];

        niknum prevPos[3];
        niknum prevRot[4];
        
        niknum invMass;
        niknum invInertia[3];
    } RigidBody;

    typedef struct PositionalConstraint {
        size_t b0;
        size_t b1;
        
        niknum r0[3];  // Attachment point in body0's local space
        niknum r1[3];  // Attachment point in body1's local space
        
        niknum compliance;     // α (inverse stiffness)
        niknum lambda;         // Lagrange multiplier
        
    } PositionalConstraint;

    // Simulator struct
    typedef struct {
        niknum gravity[3];
        niknum dt;
        niknum damping;
        
        RigidBody rigidBodies[100];
        size_t rigidBodyCount;
        
        PositionalConstraint positionalConstraints[100];
        size_t positionalConstraintCount;

        size_t posIters;
    } RigidBodySimulator;
}
