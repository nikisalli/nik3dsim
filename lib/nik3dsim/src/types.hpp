#pragma once

#include <stdlib.h>

#include "math.hpp"

namespace nik3dsim {
    // Body type enum
    typedef enum {
        BODY_SPHERE,
        BODY_BOX
    } BodyType;

    // Forward declarations for struct dependencies
    struct RigidBody;
    struct DistanceConstraint;

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

    // Distance Constraint struct
    typedef struct DistanceConstraint {
        size_t body0;
        size_t body1;
        
        niknum localPos0[3];
        niknum localPos1[3];
        
        niknum distance;
    } DistanceConstraint;

    typedef struct HingeConstraint {
        size_t body0;
        size_t body1;

        niknum local_pos0[3];  // Local attachment point on body0 
        niknum local_pos1[3];  // Local attachment point on body1
        niknum local_axis0[3]; // Local hinge axis on body0
        niknum local_axis1[3]; // Local hinge axis on body1
    } HingeConstraint;

    // Simulator struct
    typedef struct {
        niknum gravity[3];
        niknum dt;
        
        RigidBody rigidBodies[100];
        size_t rigidBodyCount;
        
        DistanceConstraint distanceConstraints[100];
        size_t distanceConstraintCount;
        HingeConstraint hingeConstraints[100];
        size_t hingeConstraintCount;

        size_t posIters;
    } RigidBodySimulator;
}
