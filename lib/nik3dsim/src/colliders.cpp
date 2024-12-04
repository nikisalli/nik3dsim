#include <cmath>
#include "colliders.hpp"
#include "math.hpp"
#include "types.hpp"

namespace nik3dsim {
    static void segment_box_query(niknum result[4], const niknum s[3], const niknum e[3], const niknum b[3]) {
        // Initialize result with zeros
        vec4_zero(result);
        niknum d[3];
        vec3_sub(d, e, s);
        
        // Handle reflection within line_box_query
        bool reflected[3] = {false, false, false};
        niknum modified_o[3], modified_d[3];
        for(int i = 0; i < 3; i++) {
            reflected[i] = d[i] < 0;
            modified_o[i] = reflected[i] ? -s[i] : s[i];
            modified_d[i] = reflected[i] ? -d[i] : d[i];
        }
        
        niknum PmE[3], PpE[3];
        vec3_sub(PmE, modified_o, b);
        vec3_add(PpE, modified_o, b);
        
        // Determine indices for face query
        int indices[3];
        if(modified_d[1] * PmE[0] >= modified_d[0] * PmE[1]) {
            if(modified_d[2] * PmE[0] >= modified_d[0] * PmE[2]) {
                indices[0] = 0; indices[1] = 1; indices[2] = 2;
            } else {
                indices[0] = 2; indices[1] = 0; indices[2] = 1;
            }
        } else {
            if(modified_d[2] * PmE[1] >= modified_d[1] * PmE[2]) {
                indices[0] = 1; indices[1] = 2; indices[2] = 0;
            } else {
                indices[0] = 2; indices[1] = 0; indices[2] = 1;
            }
        }
        
        // Reorder vectors according to index array
        niknum bi[3] = {b[indices[0]], b[indices[1]], b[indices[2]]};
        niknum oi[3] = {modified_o[indices[0]], modified_o[indices[1]], modified_o[indices[2]]};
        niknum di[3] = {modified_d[indices[0]], modified_d[indices[1]], modified_d[indices[2]]};
        niknum PmEi[3] = {PmE[indices[0]], PmE[indices[1]], PmE[indices[2]]};
        niknum PpEi[3] = {PpE[indices[0]], PpE[indices[1]], PpE[indices[2]]};
        
        // Face query computation
        niknum c[4];
        if(di[0] * PpEi[1] >= di[1] * PmEi[0]) {
            if(di[0] * PpEi[2] >= di[2] * PmEi[0]) {
                c[0] = bi[0];
                c[1] = oi[1] - di[1] * PmEi[0] / di[0];
                c[2] = oi[2] - di[2] * PmEi[0] / di[0];
                c[3] = -PmEi[0] / di[0];
            } else {
                niknum len_sqr = di[0] * di[0] + di[2] * di[2];
                niknum tmp = len_sqr * PpEi[1] - di[1] * (di[0] * PmEi[0] + di[2] * PpEi[2]);
                if(tmp <= 2.0f * len_sqr * bi[1]) {
                    niknum t = tmp / len_sqr;
                    len_sqr += di[1] * di[1];
                    niknum tmp_val = PpEi[1] - t;
                    niknum delta = di[0] * PmEi[0] + di[1] * tmp_val + di[2] * PpEi[2];
                    c[0] = bi[0];
                    c[1] = t - bi[1];
                    c[2] = -bi[2];
                    c[3] = -delta / len_sqr;
                } else {
                    len_sqr += di[1] * di[1];
                    niknum delta = di[0] * PmEi[0] + di[1] * PmEi[1] + di[2] * PpEi[2];
                    c[0] = bi[0];
                    c[1] = bi[1];
                    c[2] = -bi[2];
                    c[3] = -delta / len_sqr;
                }
            }
        } else {
            if(di[0] * PpEi[2] >= di[2] * PmEi[0]) {
                niknum len_sqr = di[0] * di[0] + di[1] * di[1];
                niknum tmp = len_sqr * PpEi[2] - di[2] * (di[0] * PmEi[0] + di[1] * PpEi[1]);
                if(tmp <= 2.0f * len_sqr * bi[2]) {
                    niknum t = tmp / len_sqr;
                    len_sqr += di[2] * di[2];
                    niknum tmp_val = PpEi[2] - t;
                    niknum delta = di[0] * PmEi[0] + di[1] * PpEi[1] + di[2] * tmp_val;
                    c[0] = bi[0];
                    c[1] = -bi[1];
                    c[2] = t - bi[2];
                    c[3] = -delta / len_sqr;
                } else {
                    len_sqr += di[2] * di[2];
                    niknum delta = di[0] * PmEi[0] + di[1] * PpEi[1] + di[2] * PmEi[2];
                    c[0] = bi[0];
                    c[1] = -bi[1];
                    c[2] = bi[2];
                    c[3] = -delta / len_sqr;
                }
            } else {
                niknum len_sqr = di[0] * di[0] + di[2] * di[2];
                niknum tmp = len_sqr * PpEi[1] - di[1] * (di[0] * PmEi[0] + di[2] * PpEi[2]);
                if(tmp >= 0.0f) {
                    if(tmp <= 2.0f * len_sqr * bi[1]) {
                        niknum t = tmp / len_sqr;
                        len_sqr += di[1] * di[1];
                        niknum tmp_val = PpEi[1] - t;
                        niknum delta = di[0] * PmEi[0] + di[1] * tmp_val + di[2] * PpEi[2];
                        c[0] = bi[0];
                        c[1] = t - bi[1];
                        c[2] = -bi[2];
                        c[3] = -delta / len_sqr;
                    } else {
                        len_sqr += di[1] * di[1];
                        niknum delta = di[0] * PmEi[0] + di[1] * PmEi[1] + di[2] * PpEi[2];
                        c[0] = bi[0];
                        c[1] = bi[1];
                        c[2] = -bi[2];
                        c[3] = -delta / len_sqr;
                    }
                } else {
                    len_sqr = di[0] * di[0] + di[1] * di[1];
                    tmp = len_sqr * PpEi[2] - di[2] * (di[0] * PmEi[0] + di[1] * PpEi[1]);
                    if(tmp >= 0.0f) {
                        if(tmp <= 2.0f * len_sqr * bi[2]) {
                            niknum t = tmp / len_sqr;
                            len_sqr += di[2] * di[2];
                            niknum tmp_val = PpEi[2] - t;
                            niknum delta = di[0] * PmEi[0] + di[1] * PpEi[1] + di[2] * tmp_val;
                            c[0] = bi[0];
                            c[1] = -bi[1];
                            c[2] = t - bi[2];
                            c[3] = -delta / len_sqr;
                        } else {
                            len_sqr += di[2] * di[2];
                            niknum delta = di[0] * PmEi[0] + di[1] * PpEi[1] + di[2] * PmEi[2];
                            c[0] = bi[0];
                            c[1] = -bi[1];
                            c[2] = bi[2];
                            c[3] = -delta / len_sqr;
                        }
                    } else {
                        len_sqr += di[2] * di[2];
                        niknum delta = di[0] * PmEi[0] + di[1] * PpEi[1] + di[2] * PpEi[2];
                        c[0] = bi[0];
                        c[1] = -bi[1];
                        c[2] = -bi[2];
                        c[3] = -delta / len_sqr;
                    }
                }
            }
        }
        
        // Map back to original ordering
        int map_indices[3] = {0, 0, 0};
        map_indices[indices[0]] = 0;
        map_indices[indices[1]] = 1;
        map_indices[indices[2]] = 2;
        
        niknum mapped_result[4];
        mapped_result[0] = c[map_indices[0]];
        mapped_result[1] = c[map_indices[1]];
        mapped_result[2] = c[map_indices[2]];
        mapped_result[3] = c[3];
        
        // Un-reflect the result
        for(int i = 0; i < 3; i++) {
            result[i] = reflected[i] ? -mapped_result[i] : mapped_result[i];
        }
        result[3] = mapped_result[3];
        
        // Handle parameter out of range
        if(!(result[3] >= 0.0f && result[3] <= 1.0f)) {
            niknum parameter = result[3] < 0.0f ? 0.0f : 1.0f;
            niknum point[3];
            vec3_addscl(point, s, d, parameter);
            
            // Point-box query
            for(int i = 0; i < 3; i++) {
                if(point[i] < -b[i]) result[i] = -b[i];
                else if(point[i] > b[i]) result[i] = b[i];
                else result[i] = point[i];
            }
            result[3] = parameter;
        }
    }

    int collide_sphere_plane(Contact contacts[], const niknum spos[3], const niknum ssize[3], const niknum ppos[3], const niknum prot[4]) {
        niknum tmp[3] = {0, 0, 1}, planeNormal[3];
        vec3_quat_rotate(planeNormal, prot, tmp);
        niknum sphereToPlane[3];
        vec3_sub(sphereToPlane, spos, ppos);
        niknum distance = vec3_dot(planeNormal, sphereToPlane);
        contacts[0].depth = fabs(distance) - ssize[0];
        // vec3_scl(contact.n, planeNormal, mult);
        vec3_addscl(contacts[0].pos0, spos, planeNormal, -ssize[0] * copysignf(1.0f, distance));
        vec3_addscl(contacts[0].pos1, spos, planeNormal, -distance);
        return 1;
    }

    int collide_capsule_plane(Contact contacts[], const niknum cpos[3], const niknum crot[4], const niknum csize[3], const niknum ppos[3], const niknum prot[4]) {
        niknum tmp[3] = {0, 0, 1}, planeNormal[3], capsuleDir[3];
        vec3_quat_rotate(planeNormal, prot, tmp);
        vec3_quat_rotate(capsuleDir, crot, tmp);
        niknum p1[3], p2[3], toP1[3], toP2[3];
        vec3_addscl(p1, cpos, capsuleDir, csize[1]/2);
        vec3_addscl(p2, cpos, capsuleDir, -csize[1]/2);
        vec3_sub(toP1, p1, ppos);
        vec3_sub(toP2, p2, ppos);
        niknum d1 = vec3_dot(planeNormal, toP1);
        niknum d2 = vec3_dot(planeNormal, toP2);
        niknum useFirst = fabs(d1) < fabs(d2);
        niknum distance = useFirst ? d1 : d2;
        niknum invert = (d1 < 0 || useFirst) && (d2 < 0 || !useFirst);
        vec3_addscl(contacts[0].pos0, useFirst ? p1 : p2, planeNormal, -csize[0] * (invert ? -1 : 1));
        vec3_addscl(contacts[0].pos1, useFirst ? p1 : p2, planeNormal, -distance);
        contacts[0].depth = fabs(distance) - csize[0];
        return 1;
    }

    int collide_capsule_box(Contact contacts[], const niknum cpos[3], const niknum crot[4], const niknum csize[3], const niknum bpos[3], const niknum brot[4], const niknum bsize[3]) {
        niknum capsuleDir[3], tmp[3] = {0, 0, 1};  // Default capsule direction
        vec3_quat_rotate(capsuleDir, crot, tmp);  // Rotate to world space
        niknum segStart[3], segEnd[3], relStart[3], relEnd[3];
        vec3_addscl(segStart, cpos, capsuleDir, csize[1]/2);
        vec3_addscl(segEnd, cpos, capsuleDir, -csize[1]/2);
        niknum temp[3], rot[9];
        quat2rotmat(brot, rot);
        vec3_sub(temp, segStart, bpos);  // Translate to box origin
        // vec3_quat_rotate(relStart, ibrot, temp);  // Rotate to box space
        vec3_matmul_t(relStart, rot, temp);
        vec3_sub(temp, segEnd, bpos);  // Translate to box origin
        // vec3_quat_rotate(relEnd, ibrot, temp);  // Rotate to box space
        vec3_matmul_t(relEnd, rot, temp);
        niknum query_result[4];
        segment_box_query(query_result, relStart, relEnd, bsize);
        niknum d[3];
        vec3_sub(d, relEnd, relStart);
        niknum segmentPoint[3];
        vec3_addscl(segmentPoint, relStart, d, query_result[3]);
        // vec3_quat_rotate(temp, brot, query_result);
        vec3_matmul(temp, rot, query_result);
        vec3_add(contacts[0].pos0, temp, bpos);
        // vec3_quat_rotate(temp, brot, segmentPoint);
        vec3_matmul(temp, rot, segmentPoint);
        vec3_add(contacts[0].pos1, temp, bpos);
        niknum normal[3];
        vec3_sub(normal, contacts[0].pos1, contacts[0].pos0);
        niknum dist = vec3_normalize(normal, normal);
        // Add size to pos0 directed as normal
        vec3_addscl(contacts[0].pos0, contacts[0].pos0, normal, csize[0]);
        // vec3_scl(contact.n, normal, -1.0);
        // vec3_copy(contact.pos, contact.pos0);
        contacts[0].depth = dist - csize[0];  // Subtract capsule radius
        return 1;
    }

    int collide_capsule_aabb(Contact contacts[], const niknum cpos[3], const niknum crot[4], const niknum csize[3], const niknum bpos[3], const niknum bsize[3]) {
        niknum capsuleDir[3], tmp[3] = {0, 0, 1};  // Default capsule direction
        vec3_quat_rotate(capsuleDir, crot, tmp);  // Rotate to world space
        niknum segStart[3], segEnd[3], relStart[3], relEnd[3];
        vec3_addscl(segStart, cpos, capsuleDir, csize[1]/2);
        vec3_addscl(segEnd, cpos, capsuleDir, -csize[1]/2);
        vec3_sub(relStart, segStart, bpos);
        vec3_sub(relEnd, segEnd, bpos);
        niknum query_result[4];
        segment_box_query(query_result, relStart, relEnd, bsize);
        niknum d[3];
        vec3_sub(d, relEnd, relStart);
        niknum segmentPoint[3];
        vec3_addscl(segmentPoint, relStart, d, query_result[3]);
        vec3_add(contacts[0].pos0, query_result, bpos);
        vec3_add(contacts[0].pos1, segmentPoint, bpos);
        niknum normal[3];
        vec3_sub(normal, contacts[0].pos0, contacts[0].pos1);
        niknum dist = vec3_normalize(normal, normal);
        vec3_addscl(contacts[0].pos1, contacts[0].pos1, normal, csize[0]);
        // vec3_scl(contact.n, normal, -1.0);
        // vec3_copy(contact.pos, contact.pos0);
        contacts[0].depth = dist - csize[0];  // Subtract capsule radius
        return 1;
    }

    int collide_box_plane(Contact contacts[], const niknum bpos[3], const niknum brot[4], const niknum bsize[3], const niknum ppos[3], const niknum prot[4]) {
        // Get plane normal in world space (plane's local up vector rotated by orientation)
        niknum tmp[3] = {0, 0, 1}, planeNormal[3];
        vec3_quat_rotate(planeNormal, prot, tmp);
        
        // Transform box vertices to world space
        niknum vertices[8][3];
        niknum localVerts[8][3] = {
            {-bsize[0], -bsize[1], -bsize[2]},
            { bsize[0], -bsize[1], -bsize[2]},
            { bsize[0],  bsize[1], -bsize[2]},
            {-bsize[0],  bsize[1], -bsize[2]},
            {-bsize[0], -bsize[1],  bsize[2]},
            { bsize[0], -bsize[1],  bsize[2]},
            { bsize[0],  bsize[1],  bsize[2]},
            {-bsize[0],  bsize[1],  bsize[2]}
        };
        
        // Transform vertices to world space
        niknum cache[9];
        quat2rotmat(brot, cache);
        for(int i = 0; i < 8; i++) {
            niknum rotated[3];
            // vec3_quat_rotate(rotated, brot, localVerts[i]);
            vec3_matmul(rotated, cache, localVerts[i]);
            vec3_add(vertices[i], rotated, bpos);
        }
        
        // Calculate center distance
        niknum boxToPlane[3];
        vec3_sub(boxToPlane, bpos, ppos);
        niknum centerDist = vec3_dot(planeNormal, boxToPlane);
        
        // Calculate distances from all vertices to plane
        niknum distances[8];
        int numContacts = 0;
        
        // Find penetrating vertices
        for(int i = 0; i < 8; i++) {
            niknum toPlane[3];
            vec3_sub(toPlane, vertices[i], ppos);
            distances[i] = vec3_dot(planeNormal, toPlane);
            
            bool isPenetrating = centerDist > 0 ? distances[i] < 0 : distances[i] > 0;

            if(isPenetrating) {
                vec3_copy(contacts[numContacts].pos0, vertices[i]);
                vec3_copy(contacts[numContacts].pos1, vertices[i]);
                vec3_addscl(contacts[numContacts].pos1, contacts[numContacts].pos1, planeNormal, -distances[i]);
                contacts[numContacts].depth = centerDist > 0 ? distances[i] : -distances[i];
                numContacts++;
            }
        }
        
        return numContacts;
    }

    int collide_rigid_rigid(Contact* contacts, const RigidBodyModel* bm0, const RigidBodyModel* bm1, const RigidBodyData* bd0, const RigidBodyData* bd1) {
        // sort by type
        BodyType t0 = bm0->type < bm1->type ? bm0->type : bm1->type;
        BodyType t1 = bm0->type < bm1->type ? bm1->type : bm0->type;

        Contact contact;

        return 0;
    }

    int collide_rigid_static(Contact* contacts, const RigidBodyModel* bm0, const StaticBodyModel* bm1, const RigidBodyData* bd0) {
        bool swap = bm0->type > bm1->type;
        niknum pos0[3], pos1[3], size0[3], size1[3], rot0[4], rot1[4];
        BodyType t0, t1;
        int numcon;

        t0 = swap ? bm1->type : bm0->type; t1 = swap ? bm0->type : bm1->type;
        vec3_copy(pos0, swap ? bm1->pos : bd0->pos);
        vec3_copy(pos1, swap ? bd0->pos : bm1->pos);
        vec3_copy(size0, swap ? bm1->size : bm0->size);
        vec3_copy(size1, swap ? bm0->size : bm1->size);
        vec4_copy(rot0, swap ? bm1->rot : bd0->rot);
        vec4_copy(rot1, swap ? bd0->rot : bm1->rot);

        if (t0 == BODY_SPHERE && t1 == BODY_PLANE) numcon = collide_sphere_plane(contacts, pos0, size0, pos1, rot1);
        else if (t0 == BODY_CAPSULE && t1 == BODY_PLANE) numcon = collide_capsule_plane(contacts, pos0, rot0, size0, pos1, rot1);
        else if (t0 == BODY_BOX && t1 == BODY_CAPSULE) numcon = collide_capsule_box(contacts, pos1, rot1, size1, pos0, rot0, size0);
        else if (t0 == BODY_AXIS_ALIGNED_BOX && t1 == BODY_CAPSULE) numcon = collide_capsule_aabb(contacts, pos1, rot1, size1, pos0, size0);
        else if (t0 == BODY_BOX && t1 == BODY_PLANE) numcon = collide_box_plane(contacts, pos0, rot0, size0, pos1, rot1);

        if (swap) {
            for (int i = 0; i < numcon; i++) vec3_swap(contacts[i].pos0, contacts[i].pos1);
        }

        return numcon;
    }
}