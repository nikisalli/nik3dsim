#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include "forward.hpp"
#include "math.hpp"

namespace nik3dsim {
    void rigidbody_init(RigidBody* body, BodyType type, niknum size[3], niknum density, niknum pos[3], niknum angles[3]) {
        body->type = type;
        for(int i = 0; i < 3; i++) {
            body->size[i] = size[i];
            body->pos[i] = pos[i];
            body->vel[i] = 0.0f;
            body->omega[i] = 0.0f;
        }
        
        // Initialize rotation from Euler angles
        niknum cx = cosf(angles[0] * 0.5f);
        niknum cy = cosf(angles[1] * 0.5f);
        niknum cz = cosf(angles[2] * 0.5f);
        niknum sx = sinf(angles[0] * 0.5f);
        niknum sy = sinf(angles[1] * 0.5f);
        niknum sz = sinf(angles[2] * 0.5f);
        
        body->rot[0] = sx * cy * cz - cx * sy * sz;  // x
        body->rot[1] = cx * sy * cz + sx * cy * sz;  // y
        body->rot[2] = cx * cy * sz - sx * sy * cz;  // z
        body->rot[3] = cx * cy * cz + sx * sy * sz;  // w

        niknum Ix, Iy, Iz;
        
        if (density > 0.0f) {
            niknum mass;
            switch(type) {
                case BODY_BOX:
                    mass = density * size[0] * size[1] * size[2];
                    body->invMass = 1.0f / mass;
                    Ix = mass / 12.0f * (size[1] * size[1] + size[2] * size[2]);
                    Iy = mass / 12.0f * (size[0] * size[0] + size[2] * size[2]);
                    Iz = mass / 12.0f * (size[0] * size[0] + size[1] * size[1]);
                    body->invInertia[0] = 1.0f / Ix;
                    body->invInertia[1] = 1.0f / Iy;
                    body->invInertia[2] = 1.0f / Iz;
                    break;
                    
                case BODY_SPHERE:
                    mass = 4.0f/3.0f * M_PI * size[0] * size[0] * size[0] * density;
                    body->invMass = 1.0f / mass;
                    niknum I = 2.0f/5.0f * mass * size[0] * size[0];
                    body->invInertia[0] = 1.0f / I;
                    body->invInertia[1] = 1.0f / I;
                    body->invInertia[2] = 1.0f / I;
                    break;
            }
        } else {
            body->invMass = 0.0f;
            body->invInertia[0] = 0.0f;
            body->invInertia[1] = 0.0f;
            body->invInertia[2] = 0.0f;
        }
    }

    void rigidbody_integrate(RigidBody* body, niknum dt, niknum gravity[3]) {
        // Store previous state
        for(int i = 0; i < 3; i++) {
            body->prevPos[i] = body->pos[i];
        }
        for(int i = 0; i < 4; i++) {
            body->prevRot[i] = body->rot[i];
        }
        
        // Update linear state
        niknum fext[3];
        vec3_scale(fext, gravity, body->invMass > 0.0f);
        
        niknum dv[3], dx[3];
        vec3_scale(dv, fext, dt);
        vec3_add(body->vel, body->vel, dv);
        
        vec3_scale(dx, body->vel, dt);
        vec3_add(body->pos, body->pos, dx);
        
        // Angular motion in body space
        niknum I_omega[3];
        I_omega[0] = body->invInertia[0] == 0.0f ? 0.0f : body->omega[0] / body->invInertia[0];
        I_omega[1] = body->invInertia[1] == 0.0f ? 0.0f : body->omega[1] / body->invInertia[1];
        I_omega[2] = body->invInertia[2] == 0.0f ? 0.0f : body->omega[2] / body->invInertia[2];
        
        // Compute cross product in body space
        niknum cross_term[3], torque[3] = {0, 0, 0};  // No external torque for now
        vec3_cross(cross_term, body->omega, I_omega);
        
        // Update angular velocity (in body space)
        niknum angular_accel[3];
        angular_accel[0] = body->invInertia[0] * (torque[0] - cross_term[0]);
        angular_accel[1] = body->invInertia[1] * (torque[1] - cross_term[1]);
        angular_accel[2] = body->invInertia[2] * (torque[2] - cross_term[2]);
        
        niknum dw[3];
        vec3_scale(dw, angular_accel, dt);
        vec3_add(body->omega, body->omega, dw);
        
        // Update rotation using exponential map
        niknum omega_len = vec3_length(body->omega);
        niknum omega_dt = omega_len * dt;
        
        if (omega_dt > 1e-6f) {
            niknum s = sinf(omega_dt * 0.5f);
            niknum inv_omega_len = s / omega_len;
            niknum c = cosf(omega_dt * 0.5f);
            
            // Create incremental rotation quaternion
            niknum dq[4];
            dq[0] = body->omega[0] * inv_omega_len;  // x
            dq[1] = body->omega[1] * inv_omega_len;  // y
            dq[2] = body->omega[2] * inv_omega_len;  // z
            dq[3] = c;                               // w
            
            // Right multiply for body-space update
            niknum new_rot[4];
            quat_multiply(new_rot, body->rot, dq);
            
            // Normalize quaternion
            quat_normalize(body->rot, new_rot);
        }
    }

    void solve_hinge_gauss_seidel_sor(RigidBody* body0, RigidBody* body1, HingeConstraint* constraint, niknum dt, niknum omega) {
        // TODO
    }

    void simulator_init(RigidBodySimulator* sim, niknum gravity[3], niknum timeStepSize, int numPosIters) {
        vec3_copy(sim->gravity, gravity);
        sim->dt = timeStepSize;
        sim->posIters = numPosIters;
        sim->rigidBodyCount = 0;
        sim->distanceConstraintCount = 0;
        sim->hingeConstraintCount = 0;
    }

    void simulator_simulate(RigidBodySimulator* sim) {        
        // TODO: Implement CollectCollisionPairs()
        
        // Integrate bodies
        for (int i = 0; i < sim->rigidBodyCount; i++) {
            rigidbody_integrate(&sim->rigidBodies[i], sim->dt, sim->gravity);
        }
        
        // Solve position constraints
        for (int iter = 0; iter < sim->posIters; iter++) {
            for (int i = 0; i < sim->distanceConstraintCount; i++) {
                // TODO: Implement distance constraints
            }

            for (int i = 0; i < sim->hingeConstraintCount; i++) {
                // TODO: Implement hinge constraints
            }
        }
        
        // Update velocities
        for (int i = 0; i < sim->rigidBodyCount; i++) {
            RigidBody* body = &sim->rigidBodies[i];
            
            // Update linear velocity from position change
            // niknum deltaPos[3];
            // vec3_sub(deltaPos, body->pos, body->prevPos);
            // vec3_scale(body->vel, deltaPos, 1.0f / sim->dt);
            
            // Update angular velocity from quaternion change
            // niknum deltaQ[4];
            // niknum conj[4];
            // quat_conjugate(conj, body->prevRot);
            // quat_multiply(deltaQ, body->rot, conj);
            // niknum sign = copysignf(1.0f, deltaQ[3]);  // Built-in branchless sign function
            // vec3_scale(body->omega, (niknum[3]){deltaQ[0], deltaQ[1], deltaQ[2]}, (2.0f / sim->dt) * sign);

            // Apply damping
            // vec3_scale(body->vel, body->vel, fmaxf(1.0f - body->damping * sim->dt, 0.0f));
            // vec3_scale(body->omega, body->omega, fmaxf(1.0f - body->damping * sim->dt, 0.0f));
        }
    }

    void print_simulation_state(RigidBodySimulator* sim) {
        for (int i = 0; i < sim->rigidBodyCount; i++) {
            RigidBody* body = &sim->rigidBodies[i];
            printf(" body %d (%s):", i, body->type == BODY_BOX ? "box" : "sphere");
            printf(" pos: %.2f, %.2f, %.2f", body->pos[0], body->pos[1], body->pos[2]);
            printf(" vel: %.2f, %.2f, %.2f", body->vel[0], body->vel[1], body->vel[2]);
            printf(" rot: %.2f, %.2f, %.2f, %.2f", body->rot[0], body->rot[1], body->rot[2], body->rot[3]);
            printf(" omega: %.2f, %.2f, %.2f", body->omega[0], body->omega[1], body->omega[2]);
            printf("\n");
        }
    }
} // namespace nik3dsim