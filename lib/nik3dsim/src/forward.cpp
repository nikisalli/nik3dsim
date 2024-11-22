#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <sys/types.h>
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
        vec3_copy(body->prevPos, body->pos);
        vec4_copy(body->prevRot, body->rot);

        // Update linear state
        niknum fext[3];
        vec3_scl(fext, gravity, body->invMass > 0.0f);
        
        vec3_addscl(body->vel, body->vel, fext, dt);
        vec3_addscl(body->pos, body->pos, body->vel, dt);
        
        // Angular motion in body space
        niknum I_omega[3];
        I_omega[0] = (body->omega[0] / (body->invInertia[0] + (body->invInertia[0] == 0.0f))) * (body->invInertia[0] != 0.0f);
        I_omega[1] = (body->omega[1] / (body->invInertia[1] + (body->invInertia[1] == 0.0f))) * (body->invInertia[1] != 0.0f);
        I_omega[2] = (body->omega[2] / (body->invInertia[2] + (body->invInertia[2] == 0.0f))) * (body->invInertia[2] != 0.0f); 
        
        // Compute cross product in body space
        niknum cross_term[3], torque[3] = {0, 0, 0};  // No external torque for now
        vec3_cross(cross_term, body->omega, I_omega);
        
        // Update angular velocity (in body space)
        niknum angular_accel[3];
        angular_accel[0] = body->invInertia[0] * (torque[0] - cross_term[0]);
        angular_accel[1] = body->invInertia[1] * (torque[1] - cross_term[1]);
        angular_accel[2] = body->invInertia[2] * (torque[2] - cross_term[2]);
        
        niknum dw[3];
        vec3_scl(dw, angular_accel, dt);
        vec3_add(body->omega, body->omega, dw);
        
        // Update rotation using exponential map
        niknum omega_len = vec3_length(body->omega);
        niknum omega_dt = omega_len * dt;
        
        niknum s = sinf(omega_dt * 0.5f);
        niknum mask = omega_len > 1e-6f;
        niknum inv_omega_len = s * mask / (omega_len + !mask);
        niknum c = cosf(omega_dt * 0.5f);
        
        // Create incremental rotation quaternion
        niknum dq[4];
        dq[0] = body->omega[0] * inv_omega_len;  // x
        dq[1] = body->omega[1] * inv_omega_len;  // y
        dq[2] = body->omega[2] * inv_omega_len;  // z
        dq[3] = c;                               // w
        
        // Right multiply for body-space update
        niknum new_rot[4];
        quat_mul(new_rot, body->rot, dq);
        
        // Normalize quaternion
        quat_normalize(body->rot, new_rot);
    }

    inline void local2world(RigidBody* body, niknum res[3], niknum local[3]) {
        vec3_copy(res, local);
        vec3_quat_rotate(res, body->rot, res);
        vec3_add(res, res, body->pos);
    }

    void solve_positional_constraint(RigidBody* b0, RigidBody* b1, DistanceConstraint* constraint, niknum dt) {
        niknum worldpos0[3], worldpos1[3], a0[3], a1[3], tmp[4];

        local2world(b0, worldpos0, constraint->r0);
        local2world(b1, worldpos1, constraint->r1);

        niknum n[3];
        vec3_sub(n, worldpos1, worldpos0);
        niknum c = vec3_normalize(n, n) - constraint->distance;
        
        // Compute inverse masses
        vec3_sub(a0, worldpos0, b0->pos);
        vec3_cross(tmp, a0, n);
        vec3_quat_rotate(a0, b0->invRot, tmp);
        vec3_sub(a1, worldpos1, b1->pos);
        vec3_cross(tmp, a1, n);
        vec3_quat_rotate(a1, b1->invRot, tmp);

        niknum w = a0[0] * a0[0] * b0->invInertia[0] + 
                   a0[1] * a0[1] * b0->invInertia[1] +
                   a0[2] * a0[2] * b0->invInertia[2] + b0->invMass +
                   a1[0] * a1[0] * b1->invInertia[0] +
                   a1[1] * a1[1] * b1->invInertia[1] +
                   a1[2] * a1[2] * b1->invInertia[2] + b1->invMass;
        
        niknum alpha = constraint->compliance / dt / dt;
        niknum lambda = -c / (w + alpha);

        // Update body0
        vec3_scl(n, n, -lambda);
        niknum dom[3], drot[4];
        // Update position
        vec3_addscl(b0->pos, b0->pos, n, b0->invMass);
        // Compute angular velocity
        vec3_copy(dom, n);
        vec3_sub(dom, worldpos0, b0->pos);
        vec3_cross(tmp, dom, n);
        vec3_quat_rotate(dom, b0->invRot, tmp);
        vec3_mul(dom, dom, b0->invInertia);
        vec3_quat_rotate(dom, b0->rot, dom);
        vec4_zero(drot);
        vec3_copy(drot, dom);
        // Update rotation
        quat_mul(tmp, drot, b0->rot);
        vec4_addscl(b0->rot, b0->rot, tmp, 0.5f);
        quat_normalize(b0->rot, b0->rot);

        // Update body1
        // Update position
        vec3_scl(n, n, -1.0f);
        // Compute angular velocity
        vec3_addscl(b1->pos, b1->pos, n, b1->invMass);
        vec3_copy(dom, n);
        vec3_sub(dom, worldpos1, b1->pos);
        vec3_cross(tmp, dom, n);
        vec3_quat_rotate(dom, b1->invRot, tmp);
        vec3_mul(dom, dom, b1->invInertia);
        vec3_quat_rotate(dom, b1->rot, dom);
        vec4_zero(drot);
        vec3_copy(drot, dom);
        // Update rotation
        quat_mul(tmp, drot, b1->rot);
        vec4_addscl(b1->rot, b1->rot, tmp, 0.5f);
        quat_normalize(b1->rot, b1->rot);
    }

    void solve_hinge_constraint(RigidBody* b0, RigidBody* b1, HingeConstraint* constraint, niknum dt) {
        // Compute rotation axis
        niknum n[3], a0[3], a1[3], tmp[4];
        vec3_quat_rotate(a0, b0->rot, constraint->a0);
        vec3_quat_rotate(a1, b1->rot, constraint->a1);
        vec3_cross(n, a0, a1);
        niknum c = vec3_normalize(n, n);


        // Compute generalized inverse masses (angular part only for hinge)
        niknum w = n[0] * n[0] * b0->invInertia[0] +
                   n[1] * n[1] * b0->invInertia[1] + 
                   n[2] * n[2] * b0->invInertia[2] +
                   n[0] * n[0] * b1->invInertia[0] +
                   n[1] * n[1] * b1->invInertia[1] + 
                   n[2] * n[2] * b1->invInertia[2];
        
        // Compute correction magnitude (without lambda accumulation)
        niknum alpha = constraint->compliance / (dt * dt);
        niknum lambda = -c / (w + alpha);

        // Compute angular impulse (like positional impulse p in position solver)
        niknum ang_impulse[3];
        vec3_scl(ang_impulse, n, lambda);

        // For body 0
        niknum dom0[3], drot0[4];
        // Apply inverse inertia to get angular velocity change (like I⁻¹(r × p))
        dom0[0] = -ang_impulse[0] * b0->invInertia[0];
        dom0[1] = -ang_impulse[1] * b0->invInertia[1];
        dom0[2] = -ang_impulse[2] * b0->invInertia[2];
        // Create quaternion from angular velocity (like [I⁻¹(r × p), 0])
        vec4_zero(drot0);
        vec3_copy(drot0, dom0);
        // Update rotation (like q1 += 0.5 * [I⁻¹(r × p), 0] * q1)
        quat_mul(tmp, drot0, b0->rot);
        vec4_addscl(b0->rot, b0->rot, tmp, 0.5f);
        quat_normalize(b0->rot, b0->rot);

        // For body 1 (negative correction)
        niknum dom1[3], drot1[4];
        dom1[0] = ang_impulse[0] * b1->invInertia[0];
        dom1[1] = ang_impulse[1] * b1->invInertia[1];
        dom1[2] = ang_impulse[2] * b1->invInertia[2];
        vec4_zero(drot1);
        vec3_copy(drot1, dom1);
        quat_mul(tmp, drot1, b1->rot);
        vec4_addscl(b1->rot, b1->rot, tmp, 0.5f);
        quat_normalize(b1->rot, b1->rot);
    }

    void simulator_init(RigidBodySimulator* sim, niknum gravity[3], niknum timeStepSize, int numPosIters) {
        vec3_copy(sim->gravity, gravity);
        sim->dt = timeStepSize;
        sim->posIters = numPosIters;
        sim->rigidBodyCount = 0;
        sim->positionalConstraintCount = 0;
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
            for (int i = 0; i < sim->positionalConstraintCount; i++) {
                DistanceConstraint* constraint = &sim->positionalConstraints[i];
                RigidBody* b0 = &sim->rigidBodies[constraint->b0];
                RigidBody* b1 = &sim->rigidBodies[constraint->b1];
                solve_positional_constraint(b0, b1, constraint, sim->dt);
            }

            for (int i = 0; i < sim->hingeConstraintCount; i++) {
                HingeConstraint* constraint = &sim->hingeConstraints[i];
                RigidBody* b0 = &sim->rigidBodies[constraint->b0];
                RigidBody* b1 = &sim->rigidBodies[constraint->b1];
                solve_hinge_constraint(b0, b1, constraint, sim->dt);
            }
        }
        
        // Update velocities
        for (int i = 0; i < sim->rigidBodyCount; i++) {
            RigidBody* body = &sim->rigidBodies[i];
            
            // Update linear velocity from position change
            niknum dp[3];
            vec3_sub(dp, body->pos, body->prevPos);
            vec3_scl(body->vel, dp, 1.0f / sim->dt);
            
            // Update angular velocity from quaternion change
            niknum dq[4], tmp[4];
            quat_conj(dq, body->prevRot);
            quat_mul(tmp, dq, body->rot);
            niknum axis[3] = {tmp[0], tmp[1], tmp[2]};
            niknum angle = vec3_normalize(axis, axis);
            niknum speed = 2 * atan2(angle, tmp[3]);
            speed -= (speed > nikpi) * 2 * nikpi;
            vec3_scl(body->omega, axis, speed / sim->dt);

            // Apply damping
            vec3_scl(body->vel, body->vel, fmaxf(1.0f - sim->damping * sim->dt, 0.0f));
            vec3_scl(body->omega, body->omega, fmaxf(1.0f - sim->damping * sim->dt, 0.0f));
        }
    }

    void print_simulation_state(RigidBodySimulator* sim) {
        for (int i = 0; i < sim->rigidBodyCount; i++) {
            RigidBody* body = &sim->rigidBodies[i];
            printf("body %d (%s):", i, body->type == BODY_BOX ? "box" : "sphere");
            printf(" pos: %.2f, %.2f, %.2f", body->pos[0], body->pos[1], body->pos[2]);
            printf(" vel: %.2f, %.2f, %.2f", body->vel[0], body->vel[1], body->vel[2]);
            printf(" rot: %.2f, %.2f, %.2f, %.2f", body->rot[0], body->rot[1], body->rot[2], body->rot[3]);
            printf(" omega: %.2f, %.2f, %.2f", body->omega[0], body->omega[1], body->omega[2]);
            printf("\n");
        }
    }
} // namespace nik3dsim