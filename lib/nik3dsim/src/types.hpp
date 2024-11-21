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
        niknum invRot[4];
        niknum vel[3];
        niknum omega[3];

        niknum prevPos[3];
        niknum prevRot[4];
        
        niknum invMass;
        niknum invInertia[3];
    } RigidBody;

    typedef struct DistanceConstraint {
        size_t b0;
        size_t b1;
        
        niknum r0[3];  // Attachment point in body0's local space
        niknum r1[3];  // Attachment point in body1's local space
        
        niknum compliance;     // α (inverse stiffness)
        niknum distance;
        
    } DistanceConstraint;

    typedef struct HingeConstraint {
        size_t b0;
        size_t b1;
        
        niknum r0[3];  // Attachment point in body0's local space
        niknum r1[3];  // Attachment point in body1's local space
        niknum a0[3];  // Axis in body0's local space
        niknum a1[3];  // Axis in body1's local space
        
        niknum compliance;     // α (inverse stiffness)
        niknum distance;
        
    } HingeConstraint;

    // Simulator struct
    typedef struct {
        niknum gravity[3];
        niknum dt;
        niknum damping;
        
        RigidBody rigidBodies[100];
        size_t rigidBodyCount;
        
        DistanceConstraint positionalConstraints[100];
        size_t positionalConstraintCount;
        HingeConstraint hingeConstraints[100];
        size_t hingeConstraintCount;

        size_t posIters;
    } RigidBodySimulator;
}
