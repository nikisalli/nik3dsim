#include <SDL_opengl.h>
#include <cmath>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <sys/types.h>
#include "forward.hpp"
#include "colliders.hpp"
#include "math.hpp"
#include "types.hpp"

namespace nik3dsim {
    void rigidbody_init(RigidBodyModel* bm, RigidBodyData* bd, BodyType type, niknum size[3], niknum density, niknum pos[3], niknum angles[3]) {
        bm->type = type;
        vec3_copy(bm->size, size);
        vec3_copy(bd->pos, pos);
        vec3_copy(bd->prevPos, pos);
        vec3_zero(bd->vel);
        vec3_zero(bd->omega);

        bm->contactCompliance = 0.001f;
        bm->frictionCoef = 0.0f;
        
        // Initialize rotation from Euler angles
        euler2quat(bd->rot, angles);
        vec4_copy(bd->prevRot, bd->rot);

        niknum Ix, Iy, Iz;
        niknum m_cy, m_hs;
        
        if (density > 0.0f) {
            niknum mass;
            switch(type) {
                case BODY_BOX:
                    mass = density * size[0] * size[1] * size[2] * 8.0f;
                    bm->invMass = 1.0f / mass;
                    Ix = (mass / 12.0f) * (size[1] * size[1] * 4.0f + size[2] * size[2] * 4.0f);
                    Iy = (mass / 12.0f) * (size[0] * size[0] * 4.0f + size[2] * size[2] * 4.0f);
                    Iz = (mass / 12.0f) * (size[0] * size[0] * 4.0f + size[1] * size[1] * 4.0f);
                    bm->invInertia[0] = 1.0f / Ix;
                    bm->invInertia[1] = 1.0f / Iy;
                    bm->invInertia[2] = 1.0f / Iz;
                    printf("mass: %f, Ix: %f, Iy: %f, Iz: %f invInertia: %f, %f, %f\n", mass, Ix, Iy, Iz, bm->invInertia[0], bm->invInertia[1], bm->invInertia[2]);
                    break;
                case BODY_SPHERE:
                    mass = 4.0f/3.0f * M_PI * size[0] * size[0] * size[0] * density;
                    bm->invMass = 1.0f / mass;
                    Ix = 2.0f/5.0f * mass * size[0] * size[0];
                    bm->invInertia[0] = 1.0f / Ix;
                    bm->invInertia[1] = 1.0f / Ix;
                    bm->invInertia[2] = 1.0f / Ix;
                    break;
                case BODY_CAPSULE:  // oriented along z axis
                    // https://www.gamedev.net/tutorials/programming/math-and-physics/capsule-inertia-tensor-r3856/
                    m_cy = size[1] * size[0] * size[0] * M_PI * density;
                    m_hs = (2.0f/3.0f) * M_PI * size[0] * size[0] * size[0] * density;
                    mass = m_cy + 2.0f * m_hs;
                    bm->invMass = 1.0f / mass;
                    printf("m_cy: %f, m_hs: %f mass: %f invMass: %f\n", m_cy, m_hs, mass, bm->invMass);
                    Ix = m_cy * (size[0] * size[0] / 12.0 + size[1] * size[1] / 4.0) + 
                        2 * m_hs * (2 * size[0] * size[0] / 5.0 + size[1] * size[1] / 2.0 + 3 * size[1] * size[0] / 8.0);
                    Iy = m_cy * (size[0] * size[0] / 2.0) + 2 * m_hs * (2 * size[0] * size[0] / 5.0);
                    bm->invInertia[0] = 1.0f / Ix;
                    bm->invInertia[1] = 1.0f / Ix;
                    bm->invInertia[2] = 1.0f / Iy;
                    printf("Ix: %f, Iy: %f invInertia: %f, %f, %f\n", Ix, Iy, bm->invInertia[0], bm->invInertia[1], bm->invInertia[2]);
                    break;
                default:
                    printf("Unknown body type: %d\n", type);
                    exit(1);
                    break;
            }
        } else {
            bm->invMass = 0.0f;
            vec3_zero(bm->invInertia);
        }
    }

    void static_init(StaticBodyModel* bm, BodyType type, niknum size[3], niknum pos[3], niknum angles[3]) {
        bm->type = type;
        vec3_copy(bm->size, size);
        vec3_copy(bm->pos, pos);
        euler2quat(bm->rot, angles);
        bm->contactCompliance = 0.001f;
        bm->frictionCoef = 0.0f;
    }

    void simulator_init(nikModel* m, niknum gravity[3], niknum timeStepSize, int numPosIters) {
        vec3_copy(m->gravity, gravity);
        m->dt = timeStepSize;
        m->posIters = numPosIters;
        m->rigidBodyCount = 0;
        m->positionalConstraintCount = 0;
        m->hingeConstraintCount = 0;
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
        niknum worldpos0[3], worldpos1[3], a0[3], a1[3], tmp[4], rot0[9], rot1[9];

        quat2rotmat(bd0->rot, rot0);
        quat2rotmat(bd1->rot, rot1);

        vec3_copy(worldpos0, constraint->r0);
        // vec3_quat_rotate(a0, bd0->rot, worldpos0);
        vec3_matmul(a0, rot0, worldpos0);
        vec3_add(worldpos0, a0, bd0->pos);
        vec3_copy(worldpos1, constraint->r1);
        // vec3_quat_rotate(a1, bd1->rot, worldpos1);
        vec3_matmul(a1, rot1, worldpos1);
        vec3_add(worldpos1, a1, bd1->pos);

        niknum n[3];
        vec3_sub(n, worldpos1, worldpos0);
        niknum c = vec3_normalize(n, n) - constraint->distance;
        
        // Compute inverse masses
        vec3_sub(a0, worldpos0, bd0->pos);
        vec3_cross(tmp, a0, n);
        // vec3_quat_rotate(a0, bd0->invRot, tmp);
        vec3_matmul_t(a0, rot0, tmp);
        vec3_sub(a1, worldpos1, bd1->pos);
        vec3_cross(tmp, a1, n);
        // vec3_quat_rotate(a1, bd1->invRot, tmp);
        vec3_matmul_t(a1, rot1, tmp);

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
        // vec3_quat_rotate(dom, bd0->invRot, tmp);
        vec3_matmul_t(dom, rot0, tmp);
        vec3_mul(tmp, dom, bm0->invInertia);
        // vec3_quat_rotate(dom, bd0->rot, tmp);
        vec3_matmul(dom, rot0, tmp);
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
        // vec3_quat_rotate(dom, bd1->invRot, tmp);
        vec3_matmul_t(dom, rot1, tmp);
        vec3_mul(tmp, dom, bm1->invInertia);
        // vec3_quat_rotate(dom, bd1->rot, tmp);
        vec3_matmul(dom, rot1, tmp);
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

    void solve_contact_rigid_rigid(Contact* contact, RigidBodyModel* bm0, RigidBodyModel* bm1, RigidBodyData* bd0, RigidBodyData* bd1, niknum dt) {
        
    }

    void solve_contact_rigid_static(Contact* contact, RigidBodyModel* bm0, StaticBodyModel* bm1, RigidBodyData* bd0, niknum dt) {
        niknum r0[3], p0[3], tmp[4], n[3], dp[3], rot0[9];
        niknum r_world[3];

        quat2rotmat(bd0->rot, rot0);
        
        // Transform contact point to current position
        quat_conj(tmp, bd0->prevRot);
        vec3_sub(p0, contact->pos0, bd0->prevPos);
        vec3_quat_rotate(r0, tmp, p0);
        // vec3_quat_rotate(p0, bd0->rot, r0);
        vec3_matmul(p0, rot0, r0);
        vec3_add(p0, p0, bd0->pos);
        
        // Calculate penetration
        vec3_sub(dp, p0, contact->pos1);
        vec3_sub(n, contact->pos1, contact->pos0);
        vec3_normalize(n, n);
        niknum depth = vec3_dot(dp, n);
        
        // Calculate r vector from center of mass to contact point
        vec3_sub(r0, contact->pos0, bd0->pos);
        
        // Calculate r × n (in world space)
        niknum r_cross_n[3];
        vec3_cross(r_cross_n, r0, n);
        
        // Transform r × n to body space, apply inverse inertia, transform back
        niknum temp[3];
        // vec3_quat_rotate(temp, bd0->invRot, r_cross_n);
        vec3_matmul_t(temp, rot0, r_cross_n);
        vec3_mul(tmp, temp, bm0->invInertia);
        // vec3_quat_rotate(temp, bd0->rot, tmp);
        vec3_matmul(temp, rot0, tmp);
        
        // Calculate effective mass: 1/(1/m + (r × n)·I⁻¹·(r × n))
        niknum angular_term = vec3_dot(r_cross_n, temp);
        niknum eff_mass = bm0->invMass + angular_term;
        
        // Calculate impulse magnitude
        niknum alpha = (bm0->contactCompliance + bm1->contactCompliance) / (dt * dt * 2.0);
        niknum lamn = fabs(-depth / (eff_mass + alpha));
        
        // Apply linear impulse
        vec3_addscl(bd0->pos, bd0->pos, n, lamn * bm0->invMass);
        
        // Calculate and apply angular impulse
        niknum drot[4], dom[3];
        vec3_scl(temp, r_cross_n, lamn); // r × (j·n)
        // vec3_quat_rotate(dom, bd0->invRot, temp);
        vec3_matmul_t(dom, rot0, temp);
        vec3_mul(temp, dom, bm0->invInertia);
        // vec3_quat_rotate(dom, bd0->rot, temp);
        vec3_matmul(dom, rot0, temp);
        vec4_zero(drot);
        vec3_copy(drot, dom);
        // Update rotation using infinitesimal rotation approximation
        quat_mul(tmp, drot, bd0->rot);
        vec4_addscl(bd0->rot, bd0->rot, tmp, 0.5f);
        quat_normalize(bd0->rot, bd0->rot);

        // Static friction with proper distribution between linear and angular motion
        vec3_sub(dp, p0, contact->pos0);
        vec3_scl(tmp, n, vec3_dot(dp, n));
        vec3_sub(dp, dp, tmp);
        
        niknum lamt = vec3_length(dp);
        niknum friction = fmaxf(bm0->frictionCoef, bm1->frictionCoef);

        // Apply static friction only
        // niknum applyfriction = lamt < friction * lamn;
        // if (!applyfriction) return;

        // Apply dynamic friction with same coefficient as static friction (approximation)
        lamt = fminf(lamt, friction * lamn);

        // Calculate friction direction unit vector
        niknum t[3];
        vec3_normalize(t, dp);
        vec3_scl(t, t, -1.0f);
        
        // Calculate r × t for angular contribution
        niknum r_cross_t[3];
        vec3_cross(r_cross_t, r0, t);
        
        // Transform to body space, apply inverse inertia, transform back
        vec3_matmul_t(temp, rot0, r_cross_t);
        vec3_mul(tmp, temp, bm0->invInertia);
        vec3_matmul(temp, rot0, tmp);
        
        // Calculate effective mass for friction
        niknum angular_term_t = vec3_dot(r_cross_t, temp);
        niknum eff_mass_t = bm0->invMass + angular_term_t;
        
        // Calculate friction impulse magnitude
        niknum lamt_max = friction * lamn;
        niknum lambda_t = lamt / (eff_mass_t + 1e-7f);
        lambda_t = fminf(lambda_t, lamt_max);
        
        // Apply linear component
        vec3_addscl(bd0->pos, bd0->pos, t, lambda_t * bm0->invMass);
        
        // Apply angular component
        vec3_cross(temp, r0, t);
        vec3_scl(temp, temp, lambda_t);
        vec3_matmul_t(dom, rot0, temp);
        vec3_mul(temp, dom, bm0->invInertia);
        vec3_matmul(dom, rot0, temp);
        vec4_zero(drot);
        vec3_copy(drot, dom);
        quat_mul(tmp, drot, bd0->rot);
        vec4_addscl(bd0->rot, bd0->rot, tmp, 0.5f);
        quat_normalize(bd0->rot, bd0->rot);
    }

    void simulator_step(nikModel* m, nikData* d) {        
        // Collide rigid bodies
        d->contactCount = 0;
        for (int i = 0; i < m->rigidBodyCount; i++) {
            RigidBodyModel* bm0 = &m->bodies[i];
            RigidBodyData* bd0 = &d->bodies[i];
            for (int j = i + 1; j < m->rigidBodyCount; j++) {
                RigidBodyModel* bm1 = &m->bodies[j];
                RigidBodyData* bd1 = &d->bodies[j];
                if ((bm0->contype & bm1->conaffinity) || (bm1->contype & bm0->conaffinity)) {
                    
                }
            }

            for (int j = 0; j < m->staticBodyCount; j++) {
                StaticBodyModel* bm1 = &m->staticBodies[j];
                if ((bm0->contype & bm1->conaffinity) || (bm1->contype & bm0->conaffinity)) {
                    Contact contacts[4];
                    int numcon = collide_rigid_static(contacts,bm0, bm1, bd0);

                    for (int k = 0; k < numcon; k++) {
                        if (contacts[k].depth < 0.0f) {
                        // if (true) {
                            contacts[k].b0 = i;
                            contacts[k].b1 = j;
                            contacts[k].is_static = true;
                            d->contacts[d->contactCount++] = contacts[k];
                        }
                    }
                }
            }
        }

        // Integrate bodies
        for (int i = 0; i < m->rigidBodyCount; i++) {
            rigidbody_integrate(&m->bodies[i], &d->bodies[i], m->dt, m->gravity);
        }
        
        // Solve position constraints
        for (int iter = 0; iter < m->posIters; iter++) {
            for (int i = 0; i < m->positionalConstraintCount; i++) {
                DistanceConstraint* c = &m->positionalConstraints[i];
                solve_positional_constraint(&m->bodies[c->b0], &m->bodies[c->b1], &d->bodies[c->b0], &d->bodies[c->b1], c, m->dt);
            }

            for (int i = 0; i < m->hingeConstraintCount; i++) {
                HingeConstraint* c = &m->hingeConstraints[i];
                solve_hinge_constraint(&m->bodies[c->b0], &m->bodies[c->b1], &d->bodies[c->b0], &d->bodies[c->b1], c, m->dt);
            }

            for (int i = 0; i < d->contactCount; i++) {
                Contact* con = &d->contacts[i];
                if (con->is_static) {
                    solve_contact_rigid_static(con, &m->bodies[con->b0], &m->staticBodies[con->b1], &d->bodies[con->b0], m->dt);
                } else {
                    solve_contact_rigid_rigid(con, &m->bodies[con->b0], &m->bodies[con->b1], &d->bodies[con->b0], &d->bodies[con->b1], m->dt);
                }
            }
        }
        
        // Update velocities
        for (int i = 0; i < m->rigidBodyCount; i++) {
            RigidBodyData* bd = &d->bodies[i];
            
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
            speed -= (speed > M_PI) * 2 * M_PI;
            vec3_scl(bd->omega, axis, speed / m->dt);

            // Apply damping
            vec3_scl(bd->vel, bd->vel, fmaxf(1.0f - m->damping * m->dt, 0.0f));
            vec3_scl(bd->omega, bd->omega, fmaxf(1.0f - m->damping * m->dt, 0.0f));
        }
    }

    void print_simulation_state(nikModel* m, nikData* d) {
        for (int i = 0; i < m->rigidBodyCount; i++) {
            RigidBodyModel* bm = &m->bodies[i];
            RigidBodyData* bd = &d->bodies[i];
            printf("body %d (%s):", i, bm->type == BODY_BOX ? "box" : "sphere");
            printf(" pos: %.2f, %.2f, %.2f", bd->pos[0], bd->pos[1], bd->pos[2]);
            printf(" vel: %.2f, %.2f, %.2f", bd->vel[0], bd->vel[1], bd->vel[2]);
            printf(" rot: %.2f, %.2f, %.2f, %.2f", bd->rot[0], bd->rot[1], bd->rot[2], bd->rot[3]);
            printf(" omega: %.2f, %.2f, %.2f", bd->omega[0], bd->omega[1], bd->omega[2]);
            printf("\n");
        }
    }
} // namespace nik3dsim