#include <stdio.h>
#include <math.h>
#include <stdbool.h>

#include "forward.hpp"

namespace nik3dsim {
    // RigidBody functions remain the same up to integrate
    void rigidbody_init(RigidBody* body, BodyType type, Vec3 size, float density, Vec3 pos, Vec3 angles) {
        body->type = type;
        body->size = size;
        
        body->pos = pos;
        // Initialize rotation from Euler angles
        float cx = cosf(angles.x * 0.5f);
        float cy = cosf(angles.y * 0.5f);
        float cz = cosf(angles.z * 0.5f);
        float sx = sinf(angles.x * 0.5f);
        float sy = sinf(angles.y * 0.5f);
        float sz = sinf(angles.z * 0.5f);
        body->rot = quat_create(
            sx * cy * cz - cx * sy * sz,
            cx * sy * cz + sx * cy * sz,
            cx * cy * sz - sx * sy * cz,
            cx * cy * cz + sx * sy * sz
        );
        
        body->vel = vec3_create(0, 0, 0);
        body->omega = vec3_create(0, 0, 0);

        float Ix, Iy, Iz;
        
        if (density > 0.0f) {
            float mass;
            switch(type) {
                case BODY_BOX:
                    mass = density * size.x * size.y * size.z;
                    body->invMass = 1.0f / mass;
                    Ix = mass / 12.0f * (size.y * size.y + size.z * size.z);
                    Iy = mass / 12.0f * (size.x * size.x + size.z * size.z);
                    Iz = mass / 12.0f * (size.x * size.x + size.y * size.y);
                    body->invInertia = vec3_create(1.0f / Ix, 1.0f / Iy, 1.0f / Iz);
                    break;
                    
                case BODY_SPHERE:
                    mass = 4.0f/3.0f * M_PI * size.x * size.x * size.x * density;
                    body->invMass = 1.0f / mass;
                    float I = 2.0f/5.0f * mass * size.x * size.x;
                    body->invInertia = vec3_create(1.0f / I, 1.0f / I, 1.0f / I);
                    break;
            }
        } else {
            body->invMass = 0.0f;
            body->invInertia = vec3_create(0, 0, 0);
        }
    }

    void rigidbody_integrate(RigidBody* body, float dt, Vec3 gravity) {
        // Store previous state
        body->prevPos = body->pos;
        body->prevRot = body->rot;
        
        // Update linear state
        Vec3 fext = vec3_scale(gravity, body->invMass); // External force (just gravity)
        body->vel = vec3_add(body->vel, vec3_scale(fext, dt));
        body->pos = vec3_add(body->pos, vec3_scale(body->vel, dt));
        
        // Update angular state
        Vec3 torque = vec3_create(0, 0, 0); // External torque (none for now)
        Vec3 I_omega = vec3_create(
            body->invInertia.x != 0 ? body->omega.x / body->invInertia.x : 0,
            body->invInertia.y != 0 ? body->omega.y / body->invInertia.y : 0,
            body->invInertia.z != 0 ? body->omega.z / body->invInertia.z : 0
        );
        Vec3 cross = vec3_cross(body->omega, I_omega);
        Vec3 effective_torque = vec3_sub(torque, cross);
        
        // Update angular velocity
        Vec3 angular_accel = vec3_create(
            effective_torque.x * body->invInertia.x,
            effective_torque.y * body->invInertia.y,
            effective_torque.z * body->invInertia.z
        );
        body->omega = vec3_add(body->omega, vec3_scale(angular_accel, dt));
        
        // Update rotation quaternion
        Quat omega_quat = quat_create(
            body->omega.x,
            body->omega.y,
            body->omega.z,
            0.0f
        );
        Quat q_dot = quat_multiply(omega_quat, body->rot);
        body->rot.x += 0.5f * dt * q_dot.x;
        body->rot.y += 0.5f * dt * q_dot.y;
        body->rot.z += 0.5f * dt * q_dot.z;
        body->rot.w += 0.5f * dt * q_dot.w;
        body->rot = quat_normalize(body->rot);
    }

    void simulator_init(RigidBodySimulator* sim, Vec3 gravity, float timeStepSize, int numPosIters) {
        sim->gravity = gravity;
        sim->dt = timeStepSize;
        sim->posIters = numPosIters;
        sim->rigidBodyCount = 0;
        sim->constraintCount = 0;
    }

    void simulator_simulate(RigidBodySimulator* sim) {        
        // TODO: Implement CollectCollisionPairs()
        
        // Integrate bodies
        for (int i = 0; i < sim->rigidBodyCount; i++) {
            rigidbody_integrate(&sim->rigidBodies[i], sim->dt, sim->gravity);
        }
        
        // Solve position constraints
        for (int iter = 0; iter < sim->posIters; iter++) {
            // TODO: Implement constraint solving
        }
        
        // Update velocities
        for (int i = 0; i < sim->rigidBodyCount; i++) {
            RigidBody* body = &sim->rigidBodies[i];
            
            // Update linear velocity from position change
            Vec3 deltaPos = vec3_sub(body->pos, body->prevPos);
            body->vel = vec3_scale(deltaPos, 1.0f / sim->dt);
            
            // Update angular velocity from quaternion change
            Quat deltaQ = quat_multiply(body->rot, quat_conjugate(body->prevRot));
            float sign = copysignf(1.0f, deltaQ.w);  // Built-in branchless sign function
            body->omega = vec3_scale(vec3_create(deltaQ.x, deltaQ.y, deltaQ.z), (2.0f / sim->dt) * sign);
            
            // Apply damping
            body->vel = vec3_scale(body->vel, fmaxf(1.0f - body->damping * sim->dt, 0.0f));
            body->omega = vec3_scale(body->omega, fmaxf(1.0f - body->damping * sim->dt, 0.0f));
        }
    }

    // Modified print function to use enum
    void print_simulation_state(RigidBodySimulator* sim) {
        // printf("Simulation state:\n");
        // printf("Number of bodies: %d\n", sim->rigidBodyCount);
        // printf("Number of constraints: %d\n", sim->constraintCount);
        
        for (int i = 0; i < sim->rigidBodyCount; i++) {
            RigidBody* body = &sim->rigidBodies[i];
            printf(" body %d (%s):", i, body->type == BODY_BOX ? "box" : "sphere");
            printf(" pos: %.2f, %.2f, %.2f", body->pos.x, body->pos.y, body->pos.z);
            printf(" vel: %.2f, %.2f, %.2f", body->vel.x, body->vel.y, body->vel.z);
            printf(" rot: %.2f, %.2f, %.2f, %.2f", body->rot.x, body->rot.y, body->rot.z, body->rot.w);
            printf(" omega: %.2f, %.2f, %.2f", body->omega.x, body->omega.y, body->omega.z);
            printf("\n");
        }
        printf("----------------------------------------------------\n");
    }

    void print_simulation_state_parse(RigidBodySimulator* sim) {
        printf("FRAME_START %f\n", sim->dt);
        
        // Print each body's state in a parseable format
        for (int i = 0; i < sim->rigidBodyCount; i++) {
            RigidBody* body = &sim->rigidBodies[i];
            
            // Print body type and dimensions first
            printf("BODY_SHAPE %d %.6f %.6f %.6f\n",
                body->type,
                body->size.x, body->size.y, body->size.z);
            
            // Print state (position, orientation, velocities)
            printf("BODY_STATE %d %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n",
                i,
                body->pos.x, body->pos.y, body->pos.z,
                body->rot.x, body->rot.y, body->rot.z, body->rot.w,
                body->vel.x, body->vel.y, body->vel.z,
                body->omega.x, body->omega.y, body->omega.z);
        }
        
        printf("FRAME_END\n");
        fflush(stdout);
    }
}