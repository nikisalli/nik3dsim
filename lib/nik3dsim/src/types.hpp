#pragma once

#include <cstddef>
#include <cstdint>
#include <stdlib.h>

#include "math.hpp"

namespace nik3dsim {
    // Body type enum
    typedef enum {
        BODY_SPHERE,
        BODY_BOX,
        BODY_AXIS_ALIGNED_BOX,
        BODY_CAPSULE,
        BODY_PLANE
    } BodyType;

    typedef struct {
        uint32_t b0, b1;
        niknum pos0[3];
        niknum pos1[3];
        niknum depth;
        bool is_static;
    } Contact;

    typedef struct {
        BodyType type;
        niknum size[3];
        
        uint32_t contype, conaffinity;

        niknum invMass;
        niknum invInertia[3];

        niknum contactCompliance;
        niknum frictionCoef;
    } RigidBodyModel;

    typedef struct {
        niknum pos[3];
        niknum rot[4];
        niknum vel[3];
        niknum omega[3];

        niknum prevPos[3];
        niknum prevRot[4];
    } RigidBodyData;

    typedef struct {
        BodyType type;
        niknum size[3];  // for planes it is the normal

        uint32_t contype, conaffinity;

        niknum pos[3];
        niknum rot[4];

        niknum contactCompliance;
        niknum frictionCoef;
    } StaticBodyModel;

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
        
        niknum a0[3];  // Axis in body0's local space
        niknum a1[3];  // Axis in body1's local space
        
        niknum compliance;     // α (inverse stiffness)

    } HingeConstraint;

    typedef struct {
        niknum gravity[3];
        niknum dt;
        niknum damping;

        RigidBodyModel bodies[10000];
        size_t rigidBodyCount;
        StaticBodyModel staticBodies[10000];
        size_t staticBodyCount;

        DistanceConstraint positionalConstraints[10000];
        size_t positionalConstraintCount;
        HingeConstraint hingeConstraints[10000];
        size_t hingeConstraintCount;

        size_t posIters;
    } nikModel;

    typedef struct {
        RigidBodyData bodies[10000];
        Contact contacts[10000];
        size_t contactCount;
    } nikData;
}
