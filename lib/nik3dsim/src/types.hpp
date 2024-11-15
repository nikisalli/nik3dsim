#pragma once

#include <stdlib.h>

#include "math.hpp"

namespace nik3dsim {
    // Body type enum
    typedef enum {
        BODY_SPHERE,
        BODY_BOX
    } BodyType;

    typedef enum {
        STATIC_PLANE,
    } StaticBodyType;

    // Forward declarations for struct dependencies
    struct RigidBody;
    struct DistanceConstraint;

    // RigidBody struct
    typedef struct RigidBody {
        BodyType type;
        Vec3 size;
        float damping;
        
        Vec3 pos;
        Quat rot;
        Vec3 vel;
        Vec3 omega;

        Vec3 prevPos;
        Quat prevRot;
        
        float invMass;
        Vec3 invInertia;
    } RigidBody;

    // Distance Constraint struct
    typedef struct DistanceConstraint {
        size_t body0;
        size_t body1;
        
        Vec3 localPos0;
        Vec3 localPos1;
        
        float distance;
        float compliance;
        Vec3 corr;
    } DistanceConstraint;

    // Simulator struct
    typedef struct {
        Vec3 gravity;
        float dt;
        
        RigidBody rigidBodies[100];
        int rigidBodyCount;
        
        DistanceConstraint constraints[100];
        int constraintCount;

        int posIters;
    } RigidBodySimulator;
}
