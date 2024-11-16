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
        Vec3 fext = vec3_scale(gravity, body->invMass);
        body->vel = vec3_add(body->vel, vec3_scale(fext, dt));
        body->pos = vec3_add(body->pos, vec3_scale(body->vel, dt));
        
        // Angular motion in body space
        Vec3 I_omega = vec3_create(
            body->omega.x / body->invInertia.x,
            body->omega.y / body->invInertia.y,
            body->omega.z / body->invInertia.z
        );
        
        // Compute cross product in body space
        Vec3 cross_term = vec3_cross(body->omega, I_omega);
        Vec3 torque = vec3_create(0, 0, 0);  // No external torque for now
        
        // Update angular velocity (in body space)
        Vec3 angular_accel = vec3_create(
            body->invInertia.x * (torque.x - cross_term.x),
            body->invInertia.y * (torque.y - cross_term.y),
            body->invInertia.z * (torque.z - cross_term.z)
        );
        body->omega = vec3_add(body->omega, vec3_scale(angular_accel, dt));
        
        // Update rotation using exponential map
        float omega_len = vec3_length(body->omega);
        float omega_dt = omega_len * dt;
        
        if (omega_dt > 1e-6f) {
            float s = sinf(omega_dt * 0.5f);
            // Create incremental rotation quaternion
            Quat dq = quat_create(
                body->omega.x * s / omega_len,
                body->omega.y * s / omega_len,
                body->omega.z * s / omega_len,
                cosf(omega_dt * 0.5f)
            );
            
            // Right multiply for body-space update
            body->rot = quat_multiply(body->rot, dq);
            
            // Normalize quaternion (using your implementation)
            body->rot = quat_normalize(body->rot);
        }
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
            // Vec3 deltaPos = vec3_sub(body->pos, body->prevPos);
            // body->vel = vec3_scale(deltaPos, 1.0f / sim->dt);
            
            // Update angular velocity from quaternion change
            // Quat deltaQ = quat_multiply(body->rot, quat_conjugate(body->prevRot));
            // float sign = copysignf(1.0f, deltaQ.w);  // Built-in branchless sign function
            // body->omega = vec3_scale(vec3_create(deltaQ.x, deltaQ.y, deltaQ.z), (2.0f / sim->dt) * sign);

            // Apply damping
            // body->vel = vec3_scale(body->vel, fmaxf(1.0f - body->damping * sim->dt, 0.0f));
            // body->omega = vec3_scale(body->omega, fmaxf(1.0f - body->damping * sim->dt, 0.0f));
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
}