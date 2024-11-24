#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <sys/types.h>
#include "forward.hpp"
#include "math.hpp"
#include "types.hpp"

namespace nik3dsim {
    void rigidbody_init(RigidBodyModel* bm, RigidBodyData* bd, BodyType type, niknum size[3], niknum density, niknum pos[3], niknum angles[3]) {
        bm->type = type;
        for(int i = 0; i < 3; i++) {
            bm->size[i] = size[i];
            bd->pos[i] = pos[i];
            bd->vel[i] = 0.0f;
            bd->omega[i] = 0.0f;
        }
        
        // Initialize rotation from Euler angles
        niknum cx = cosf(angles[0] * 0.5f);
        niknum cy = cosf(angles[1] * 0.5f);
        niknum cz = cosf(angles[2] * 0.5f);
        niknum sx = sinf(angles[0] * 0.5f);
        niknum sy = sinf(angles[1] * 0.5f);
        niknum sz = sinf(angles[2] * 0.5f);
        
        bd->rot[0] = sx * cy * cz - cx * sy * sz;  // x
        bd->rot[1] = cx * sy * cz + sx * cy * sz;  // y
        bd->rot[2] = cx * cy * sz - sx * sy * cz;  // z
        bd->rot[3] = cx * cy * cz + sx * sy * sz;  // w

        niknum Ix, Iy, Iz;
        
        if (density > 0.0f) {
            niknum mass;
            switch(type) {
                case BODY_BOX:
                    mass = density * size[0] * size[1] * size[2];
                    bm->invMass = 1.0f / mass;
                    Ix = mass / 12.0f * (size[1] * size[1] + size[2] * size[2]);
                    Iy = mass / 12.0f * (size[0] * size[0] + size[2] * size[2]);
                    Iz = mass / 12.0f * (size[0] * size[0] + size[1] * size[1]);
                    bm->invInertia[0] = 1.0f / Ix;
                    bm->invInertia[1] = 1.0f / Iy;
                    bm->invInertia[2] = 1.0f / Iz;
                    break;
                    
                case BODY_SPHERE:
                    mass = 4.0f/3.0f * M_PI * size[0] * size[0] * size[0] * density;
                    bm->invMass = 1.0f / mass;
                    Ix = 2.0f/5.0f * mass * size[0] * size[0];
                    bm->invInertia[0] = 1.0f / Ix;
                    bm->invInertia[1] = 1.0f / Ix;
                    bm->invInertia[2] = 1.0f / Ix;
                    break;
                default:
                    printf("Unknown body type: %d\n", type);
                    exit(1);
                    break;
            }
        } else {
            bm->invMass = 0.0f;
            bm->invInertia[0] = 0.0f;
            bm->invInertia[1] = 0.0f;
            bm->invInertia[2] = 0.0f;
        }
    }

    void rigidbody_integrate(RigidBodyModel* bm, RigidBodyData* bd, niknum dt, niknum gravity[3]) {
        // Store previous state
        vec3_copy(bd->prevPos, bd->pos);
        vec4_copy(bd->prevRot, bd->rot);

        // Update linear state
        niknum fext[3];
        vec3_scl(fext, gravity, bm->invMass > 0.0f);
        
        vec3_addscl(bd->vel, bd->vel, fext, dt);
        vec3_addscl(bd->pos, bd->pos, bd->vel, dt);
        
        // Angular motion in body space
        niknum I_omega[3];
        I_omega[0] = (bd->omega[0] / (bm->invInertia[0] + (bm->invInertia[0] == 0.0f))) * (bm->invInertia[0] != 0.0f);
        I_omega[1] = (bd->omega[1] / (bm->invInertia[1] + (bm->invInertia[1] == 0.0f))) * (bm->invInertia[1] != 0.0f);
        I_omega[2] = (bd->omega[2] / (bm->invInertia[2] + (bm->invInertia[2] == 0.0f))) * (bm->invInertia[2] != 0.0f); 
        
        // Compute cross product in body space
        niknum cross_term[3], torque[3] = {0, 0, 0};  // No external torque for now
        vec3_cross(cross_term, bd->omega, I_omega);
        
        // Update angular velocity (in body space)
        niknum angular_accel[3];
        angular_accel[0] = bm->invInertia[0] * (torque[0] - cross_term[0]);
        angular_accel[1] = bm->invInertia[1] * (torque[1] - cross_term[1]);
        angular_accel[2] = bm->invInertia[2] * (torque[2] - cross_term[2]);
        
        niknum dw[3];
        vec3_scl(dw, angular_accel, dt);
        vec3_add(bd->omega, bd->omega, dw);
        
        // Update rotation using exponential map
        niknum omega_len = vec3_length(bd->omega);
        niknum omega_dt = omega_len * dt;
        
        niknum s = sinf(omega_dt * 0.5f);
        niknum mask = omega_len > 1e-6f;
        niknum inv_omega_len = s * mask / (omega_len + !mask);
        niknum c = cosf(omega_dt * 0.5f);
        
        // Create incremental rotation quaternion
        niknum dq[4];
        dq[0] = bd->omega[0] * inv_omega_len;  // x
        dq[1] = bd->omega[1] * inv_omega_len;  // y
        dq[2] = bd->omega[2] * inv_omega_len;  // z
        dq[3] = c;                               // w
        
        // Right multiply for body-space update
        niknum new_rot[4];
        quat_mul(new_rot, bd->rot, dq);
        
        // Normalize quaternion
        quat_normalize(bd->rot, new_rot);
    }

    void solve_positional_constraint(RigidBodyModel* bm0, RigidBodyModel* bm1, RigidBodyData* bd0, RigidBodyData* bd1, DistanceConstraint* constraint, niknum dt) {
        niknum worldpos0[3], worldpos1[3], a0[3], a1[3], tmp[4];

        vec3_copy(worldpos0, constraint->r0);
        vec3_quat_rotate(worldpos0, bd0->rot, worldpos0);
        vec3_add(worldpos0, worldpos0, bd0->pos);
        vec3_copy(worldpos1, constraint->r1);
        vec3_quat_rotate(worldpos1, bd1->rot, worldpos1);
        vec3_add(worldpos1, worldpos1, bd1->pos);

        niknum n[3];
        vec3_sub(n, worldpos1, worldpos0);
        niknum c = vec3_normalize(n, n) - constraint->distance;
        
        // Compute inverse masses
        vec3_sub(a0, worldpos0, bd0->pos);
        vec3_cross(tmp, a0, n);
        vec3_quat_rotate(a0, bd0->invRot, tmp);
        vec3_sub(a1, worldpos1, bd1->pos);
        vec3_cross(tmp, a1, n);
        vec3_quat_rotate(a1, bd1->invRot, tmp);

        niknum w = a0[0] * a0[0] * bm0->invInertia[0] + 
                   a0[1] * a0[1] * bm0->invInertia[1] +
                   a0[2] * a0[2] * bm0->invInertia[2] + bm0->invMass +
                   a1[0] * a1[0] * bm1->invInertia[0] +
                   a1[1] * a1[1] * bm1->invInertia[1] +
                   a1[2] * a1[2] * bm1->invInertia[2] + bm1->invMass;
        
        niknum alpha = constraint->compliance / dt / dt;
        niknum lambda = -c / (w + alpha);

        // Update body0
        vec3_scl(n, n, -lambda);
        niknum dom[3], drot[4];
        // Update position
        vec3_addscl(bd0->pos, bd0->pos, n, bm0->invMass);
        // Compute angular velocity
        vec3_copy(dom, n);
        vec3_sub(dom, worldpos0, bd0->pos);
        vec3_cross(tmp, dom, n);
        vec3_quat_rotate(dom, bd0->invRot, tmp);
        vec3_mul(dom, dom, bm0->invInertia);
        vec3_quat_rotate(dom, bd0->rot, dom);
        vec4_zero(drot);
        vec3_copy(drot, dom);
        // Update rotation
        quat_mul(tmp, drot, bd0->rot);
        vec4_addscl(bd0->rot, bd0->rot, tmp, 0.5f);
        quat_normalize(bd0->rot, bd0->rot);

        // Update body1
        // Update position
        vec3_scl(n, n, -1.0f);
        // Compute angular velocity
        vec3_addscl(bd1->pos, bd1->pos, n, bm1->invMass);
        vec3_copy(dom, n);
        vec3_sub(dom, worldpos1, bd1->pos);
        vec3_cross(tmp, dom, n);
        vec3_quat_rotate(dom, bd1->invRot, tmp);
        vec3_mul(dom, dom, bm1->invInertia);
        vec3_quat_rotate(dom, bd1->rot, dom);
        vec4_zero(drot);
        vec3_copy(drot, dom);
        // Update rotation
        quat_mul(tmp, drot, bd1->rot);
        vec4_addscl(bd1->rot, bd1->rot, tmp, 0.5f);
        quat_normalize(bd1->rot, bd1->rot);
    }

    void solve_hinge_constraint(RigidBodyModel* bm0, RigidBodyModel* bm1, RigidBodyData* bd0, RigidBodyData* bd1, HingeConstraint* constraint, niknum dt) {
        // Compute rotation axis
        niknum n[3], a0[3], a1[3], tmp[4];
        vec3_quat_rotate(a0, bd0->rot, constraint->a0);
        vec3_quat_rotate(a1, bd1->rot, constraint->a1);
        vec3_cross(n, a0, a1);
        niknum c = vec3_normalize(n, n);


        // Compute generalized inverse masses (angular part only for hinge)
        niknum w = n[0] * n[0] * bm0->invInertia[0] +
                   n[1] * n[1] * bm0->invInertia[1] + 
                   n[2] * n[2] * bm0->invInertia[2] +
                   n[0] * n[0] * bm1->invInertia[0] +
                   n[1] * n[1] * bm1->invInertia[1] + 
                   n[2] * n[2] * bm1->invInertia[2];
        
        // Compute correction magnitude (without lambda accumulation)
        niknum alpha = constraint->compliance / (dt * dt);
        niknum lambda = -c / (w + alpha);

        // Compute angular impulse (like positional impulse p in position solver)
        niknum ang_impulse[3];
        vec3_scl(ang_impulse, n, lambda);

        // For body 0
        niknum dom0[3], drot0[4];
        // Apply inverse inertia to get angular velocity change (like I⁻¹(r × p))
        dom0[0] = -ang_impulse[0] * bm0->invInertia[0];
        dom0[1] = -ang_impulse[1] * bm0->invInertia[1];
        dom0[2] = -ang_impulse[2] * bm0->invInertia[2];
        // Create quaternion from angular velocity (like [I⁻¹(r × p), 0])
        vec4_zero(drot0);
        vec3_copy(drot0, dom0);
        // Update rotation (like q1 += 0.5 * [I⁻¹(r × p), 0] * q1)
        quat_mul(tmp, drot0, bd0->rot);
        vec4_addscl(bd0->rot, bd0->rot, tmp, 0.5f);
        quat_normalize(bd0->rot, bd0->rot);

        // For body 1 (negative correction)
        niknum dom1[3], drot1[4];
        dom1[0] = ang_impulse[0] * bm1->invInertia[0];
        dom1[1] = ang_impulse[1] * bm1->invInertia[1];
        dom1[2] = ang_impulse[2] * bm1->invInertia[2];
        vec4_zero(drot1);
        vec3_copy(drot1, dom1);
        quat_mul(tmp, drot1, bd1->rot);
        vec4_addscl(bd1->rot, bd1->rot, tmp, 0.5f);
        quat_normalize(bd1->rot, bd1->rot);
    }

    void simulator_init(nikModel* m, niknum gravity[3], niknum timeStepSize, int numPosIters) {
        vec3_copy(m->gravity, gravity);
        m->dt = timeStepSize;
        m->posIters = numPosIters;
        m->rigidBodyCount = 0;
        m->positionalConstraintCount = 0;
        m->hingeConstraintCount = 0;
    }

    void simulator_simulate(nikModel* m, nikData* d) {
        // TODO: Implement CollectCollisionPairs()
        
        // Integrate bodies
        for (int i = 0; i < m->rigidBodyCount; i++) {
            rigidbody_integrate(&m->rigidBodies[i], &d->rigidBodies[i], m->dt, m->gravity);
        }
        
        // Solve position constraints
        for (int iter = 0; iter < m->posIters; iter++) {
            for (int i = 0; i < m->positionalConstraintCount; i++) {
                DistanceConstraint* constraint = &m->positionalConstraints[i];
                RigidBodyModel* bm0 = &m->rigidBodies[constraint->b0];
                RigidBodyModel* bm1 = &m->rigidBodies[constraint->b1];
                RigidBodyData* bd0 = &d->rigidBodies[constraint->b0];
                RigidBodyData* bd1 = &d->rigidBodies[constraint->b1];
                solve_positional_constraint(bm0, bm1, bd0, bd1, constraint, m->dt);
            }

            for (int i = 0; i < m->hingeConstraintCount; i++) {
                HingeConstraint* constraint = &m->hingeConstraints[i];
                RigidBodyModel* bm0 = &m->rigidBodies[constraint->b0];
                RigidBodyModel* bm1 = &m->rigidBodies[constraint->b1];
                RigidBodyData* bd0 = &d->rigidBodies[constraint->b0];
                RigidBodyData* bd1 = &d->rigidBodies[constraint->b1];
                solve_hinge_constraint(bm0, bm1, bd0, bd1, constraint, m->dt);
            }
        }
        
        // Update velocities
        for (int i = 0; i < m->rigidBodyCount; i++) {
            RigidBodyData* bd = &d->rigidBodies[i];
            
            // Update linear velocity from position change
            niknum dp[3];
            vec3_sub(dp, bd->pos, bd->prevPos);
            vec3_scl(bd->vel, dp, 1.0f / m->dt);
            
            // Update angular velocity from quaternion change
            niknum dq[4], tmp[4];
            quat_conj(dq, bd->prevRot);
            quat_mul(tmp, dq, bd->rot);
            niknum axis[3] = {tmp[0], tmp[1], tmp[2]};
            niknum angle = vec3_normalize(axis, axis);
            niknum speed = 2 * atan2(angle, tmp[3]);
            speed -= (speed > nikpi) * 2 * nikpi;
            vec3_scl(bd->omega, axis, speed / m->dt);

            // Apply damping
            vec3_scl(bd->vel, bd->vel, fmaxf(1.0f - m->damping * m->dt, 0.0f));
            vec3_scl(bd->omega, bd->omega, fmaxf(1.0f - m->damping * m->dt, 0.0f));
        }
    }

    void print_simulation_state(nikModel* m, nikData* d) {
        for (int i = 0; i < m->rigidBodyCount; i++) {
            RigidBodyModel* bm = &m->rigidBodies[i];
            RigidBodyData* bd = &d->rigidBodies[i];
            printf("body %d (%s):", i, bm->type == BODY_BOX ? "box" : "sphere");
            printf(" pos: %.2f, %.2f, %.2f", bd->pos[0], bd->pos[1], bd->pos[2]);
            printf(" vel: %.2f, %.2f, %.2f", bd->vel[0], bd->vel[1], bd->vel[2]);
            printf(" rot: %.2f, %.2f, %.2f, %.2f", bd->rot[0], bd->rot[1], bd->rot[2], bd->rot[3]);
            printf(" omega: %.2f, %.2f, %.2f", bd->omega[0], bd->omega[1], bd->omega[2]);
            printf("\n");
        }
    }
} // namespace nik3dsim