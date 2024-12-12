#include <cmath>
#include <cstdio>
#include "colliders.hpp"
#include "math.hpp"
#include "types.hpp"

namespace nik3dsim {
    // Collision function table (using unified signature)
    typedef int (*CollisionFunc)(Contact[], const niknum[3], const niknum[4], const niknum[3], const niknum[3], const niknum[4], const niknum[3]);
    static CollisionFunc collisionTable[5][5] = {
        //                    Sphere                 Box                     Axis-Aligned Box         Capsule                    Plane
        /* Sphere    */  {collide_sphere_sphere, collide_sphere_box,   nullptr,         nullptr,                  collide_sphere_plane   },
        /* Box       */  {nullptr,               nullptr,              nullptr,         collide_capsule_box,      collide_box_plane      },
        /* AABB      */  {nullptr,               nullptr,              nullptr,         collide_capsule_aabb,     nullptr                },
        /* Capsule   */  {nullptr,               nullptr,              nullptr,         nullptr,                  collide_capsule_plane  },
        /* Plane     */  {nullptr,               nullptr,              nullptr,         nullptr,                  nullptr                }
    };

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

    // Returns the distance between two spheres
    int collide_sphere_sphere(Contact contacts[], const niknum pos0[3], const niknum rot0[4], const niknum size0[3], const niknum pos1[3], const niknum rot1[4], const niknum size1[3]) {
        niknum n[3];
        vec3_sub(n, pos0, pos1);
        niknum dist = vec3_normalize(n, n);
        contacts[0].depth = dist - size0[0] - size1[0];
        vec3_addscl(contacts[0].pos0, pos0, n, -size0[0]);
        vec3_addscl(contacts[0].pos1, pos1, n, size1[0]);
        return 1;
    }

    int collide_sphere_plane(Contact contacts[], const niknum pos0[3], const niknum rot0[4], const niknum size0[3], const niknum pos1[3], const niknum rot1[4], const niknum size1[3]) {
        niknum tmp[3] = { 0, 0, 1 }, planeNormal[3];
        vec3_quat_rotate(planeNormal, rot1, tmp);
        niknum sphereToPlane[3];
        vec3_sub(sphereToPlane, pos0, pos1);
        niknum distance = vec3_dot(planeNormal, sphereToPlane);
        contacts[0].depth = fabs(distance) - size0[0];
        vec3_addscl(contacts[0].pos0, pos0, planeNormal, -size0[0] * copysignf(1.0f, distance));
        vec3_addscl(contacts[0].pos1, pos0, planeNormal, -distance);
        return 1;
    }

    int collide_capsule_plane(Contact contacts[], const niknum pos0[3], const niknum rot0[4], const niknum size0[3], const niknum pos1[3], const niknum rot1[4], const niknum size1[3]) {
        niknum tmp[3] = { 0, 0, 1 }, planeNormal[3], capsuleDir[3];
        vec3_quat_rotate(planeNormal, rot1, tmp);
        vec3_quat_rotate(capsuleDir, rot0, tmp);
        niknum p1[3], p2[3], toP1[3], toP2[3];
        vec3_addscl(p1, pos0, capsuleDir, size0[1] / 2);
        vec3_addscl(p2, pos0, capsuleDir, -size0[1] / 2);
        vec3_sub(toP1, p1, pos1);
        vec3_sub(toP2, p2, pos1);
        niknum d1 = vec3_dot(planeNormal, toP1);
        niknum d2 = vec3_dot(planeNormal, toP2);

        int numContacts = (fabs(d1) < size0[0]) + (fabs(d2) < size0[0]);
        niknum depths[2] = { fabs(d1) - size0[0], fabs(d2) - size0[0] };
        niknum* points[2] = { p1, p2 };
        niknum distances[2] = { d1, d2 };

        bool first = depths[0] > depths[1];
        bool second = !first;
        contacts[0].depth = depths[first];
        contacts[1].depth = depths[second];
        vec3_addscl(contacts[0].pos0, points[first], planeNormal, -size0[0] * copysignf(1.0f, distances[first]));
        vec3_addscl(contacts[0].pos1, points[first], planeNormal, -distances[first]);
        vec3_addscl(contacts[1].pos0, points[second], planeNormal, -size0[0] * copysignf(1.0f, distances[second]));
        vec3_addscl(contacts[1].pos1, points[second], planeNormal, -distances[second]);
        return 1 + (depths[0] < 0.0f && depths[1] < 0.0f);
    }

    int collide_capsule_box(Contact contacts[], const niknum pos0[3], const niknum rot0[4], const niknum size0[3], const niknum pos1[3], const niknum rot1[4], const niknum size1[3]) {                
        niknum capsuleDir[3], tmp[3] = { 0, 0, 1 };  // Default capsule direction
        vec3_quat_rotate(capsuleDir, rot1, tmp);  // Rotate to world space
        niknum segStart[3], segEnd[3], relStart[3], relEnd[3];
        vec3_addscl(segStart, pos1, capsuleDir, size1[1] / 2);
        vec3_addscl(segEnd, pos1, capsuleDir, -size1[1] / 2);
        niknum temp[3], rot[9];
        quat2rotmat(rot0, rot);
        vec3_sub(temp, segStart, pos0);  // Translate to box origin
        vec3_matmul_t(relStart, rot, temp);
        vec3_sub(temp, segEnd, pos0);  // Translate to box origin
        vec3_matmul_t(relEnd, rot, temp);
        niknum query_result[4];
        segment_box_query(query_result, relStart, relEnd, size0);
        niknum d[3];
        vec3_sub(d, relEnd, relStart);
        niknum segmentPoint[3];
        vec3_addscl(segmentPoint, relStart, d, query_result[3]);
        vec3_matmul(temp, rot, query_result);
        vec3_add(contacts[0].pos0, temp, pos0);
        vec3_matmul(temp, rot, segmentPoint);
        vec3_add(contacts[0].pos1, temp, pos0);
        niknum normal[3];
        vec3_sub(normal, contacts[0].pos0, contacts[0].pos1);
        niknum dist = vec3_normalize(normal, normal);
        // Add size to pos0 directed as normal
        vec3_addscl(contacts[0].pos1, contacts[0].pos1, normal, size1[0]);
        contacts[0].depth = dist - size1[0];  // Subtract capsule radius
        return 1;
    }


    int collide_capsule_aabb(Contact contacts[], const niknum pos0[3], const niknum rot0[4], const niknum size0[3], const niknum pos1[3], const niknum rot1[4], const niknum size1[3]) {
        niknum capsuleDir[3], tmp[3] = { 0, 0, 1 };  // Default capsule direction
        vec3_quat_rotate(capsuleDir, rot1, tmp);  // Rotate to world space
        niknum segStart[3], segEnd[3], relStart[3], relEnd[3];
        vec3_addscl(segStart, pos1, capsuleDir, size1[1] / 2);
        vec3_addscl(segEnd, pos1, capsuleDir, -size1[1] / 2);
        vec3_sub(relStart, segStart, pos0);
        vec3_sub(relEnd, segEnd, pos0);
        niknum query_result[4];
        segment_box_query(query_result, relStart, relEnd, size0);
        niknum d[3];
        vec3_sub(d, relEnd, relStart);
        niknum segmentPoint[3];
        vec3_addscl(segmentPoint, relStart, d, query_result[3]);
        vec3_add(contacts[0].pos0, query_result, pos0);
        vec3_add(contacts[0].pos1, segmentPoint, pos0);
        niknum normal[3];
        vec3_sub(normal, contacts[0].pos0, contacts[0].pos1);
        niknum dist = vec3_normalize(normal, normal);
        vec3_addscl(contacts[0].pos1, contacts[0].pos1, normal, size1[0]);
        contacts[0].depth = dist - size1[0];  // Subtract capsule radius
        return 1;
    }

    int collide_sphere_box(Contact contacts[], const niknum pos0[3], const niknum rot0[4], const niknum size0[3], const niknum pos1[3], const niknum rot1[4], const niknum size1[3]) {
        // Transform sphere position to box local space
        niknum localSpherePos[3], temp[3];
        niknum rot[9];
        quat2rotmat(rot1, rot);
        vec3_sub(temp, pos0, pos1);  // Translate to box origin
        vec3_matmul_t(localSpherePos, rot, temp);  // Rotate to box space

        // Find closest point on box to sphere center
        niknum closestPoint[3];
        for (int i = 0; i < 3; i++) {
            if (localSpherePos[i] < -size1[i]) {
                closestPoint[i] = -size1[i];
            }
            else if (localSpherePos[i] > size1[i]) {
                closestPoint[i] = size1[i];
            }
            else {
                closestPoint[i] = localSpherePos[i];
            }
        }

        // Transform closest point back to world space
        niknum worldClosestPoint[3];
        vec3_matmul(temp, rot, closestPoint);
        vec3_add(worldClosestPoint, temp, pos1);

        // Calculate normal and distance
        niknum normal[3];
        vec3_sub(normal, pos0, worldClosestPoint);
        niknum dist = vec3_normalize(normal, normal);

        // Set contact points
        vec3_copy(contacts[0].pos1, worldClosestPoint);  // Point on box
        vec3_addscl(contacts[0].pos0, pos0, normal, -size0[0]);  // Point on sphere surface
        contacts[0].depth = dist - size0[0];  // Penetration depth (negative if penetrating)

        return 1;
    }

    int collide_box_plane(Contact contacts[], const niknum pos0[3], const niknum rot0[4], const niknum size0[3], const niknum pos1[3], const niknum rot1[4], const niknum size1[3]) {
        // Get plane normal in world space (plane's local up vector rotated by orientation)
        niknum tmp[3] = { 0, 0, 1 }, planeNormal[3];
        vec3_quat_rotate(planeNormal, rot1, tmp);

        // Transform box vertices to world space
        niknum vertices[8][3];
        niknum localVerts[8][3] = {
            {-size0[0], -size0[1], -size0[2]},
            { size0[0], -size0[1], -size0[2]},
            { size0[0],  size0[1], -size0[2]},
            {-size0[0],  size0[1], -size0[2]},
            {-size0[0], -size0[1],  size0[2]},
            { size0[0], -size0[1],  size0[2]},
            { size0[0],  size0[1],  size0[2]},
            {-size0[0],  size0[1],  size0[2]}
        };

        // Transform vertices to world space
        niknum cache[9];
        quat2rotmat(rot0, cache);
        for (int i = 0; i < 8; i++) {
            niknum rotated[3];
            vec3_matmul(rotated, cache, localVerts[i]);
            vec3_add(vertices[i], rotated, pos0);
        }

        // Calculate center distance
        niknum boxToPlane[3];
        vec3_sub(boxToPlane, pos0, pos1);
        niknum centerDist = vec3_dot(planeNormal, boxToPlane);

        // Calculate distances from all vertices to plane
        niknum distances[8];
        int numContacts = 0;

        // Find minimum distance and penetrating vertices
        niknum minDist = INFINITY;
        int minDistIndex = 0;

        for (int i = 0; i < 8; i++) {
            niknum toPlane[3];
            vec3_sub(toPlane, vertices[i], pos1);
            distances[i] = vec3_dot(planeNormal, toPlane);

            bool isPenetrating = centerDist > 0 ? distances[i] < 0 : distances[i] > 0;

            if (isPenetrating) {
                vec3_copy(contacts[numContacts].pos0, vertices[i]);
                vec3_copy(contacts[numContacts].pos1, vertices[i]);
                vec3_addscl(contacts[numContacts].pos1, contacts[numContacts].pos1, planeNormal, -distances[i]);
                contacts[numContacts].depth = centerDist > 0 ? distances[i] : -distances[i];
                numContacts++;
            }

            // Track minimum absolute distance
            if (fabs(distances[i]) < fabs(minDist)) {
                minDist = distances[i];
                minDistIndex = i;
            }
        }

        // If no penetration, return the contact with minimum distance
        if (numContacts == 0) {
            vec3_copy(contacts[0].pos0, vertices[minDistIndex]);
            vec3_copy(contacts[0].pos1, vertices[minDistIndex]);
            vec3_addscl(contacts[0].pos1, contacts[0].pos1, planeNormal, -distances[minDistIndex]);
            contacts[0].depth = distances[minDistIndex];
            return 1;
        }

        return numContacts;
    }

    // Returns all the penetrating contacts if there are any or the shortest distance if there are none
    int collide_rigid_rigid(Contact* contacts, const RigidBodyModel* bm0, const RigidBodyModel* bm1, const RigidBodyData* bd0, const RigidBodyData* bd1) {
        // sort by type
        BodyType t0 = bm0->type < bm1->type ? bm0->type : bm1->type;
        BodyType t1 = bm0->type < bm1->type ? bm1->type : bm0->type;

        Contact contact;

        return 0;
    }

    // Returns all the penetrating contacts if there are any or the shortest distance if there are none
    int collide_rigid_static(Contact* contacts, const RigidBodyModel* bm0, const StaticBodyModel* bm1, const RigidBodyData* bd0) {
        bool swap = bm0->type > bm1->type;
        BodyType t0 = swap ? bm1->type : bm0->type;
        BodyType t1 = swap ? bm0->type : bm1->type;

        // Look up collision function in table
        CollisionFunc collisionFunc = collisionTable[t0][t1];

        if (!collisionFunc) return 0;

        // Call the appropriate collision function
        int numContacts = collisionFunc(contacts,
            swap ? bm1->pos : bd0->pos,
            swap ? bm1->rot : bd0->rot,
            swap ? bm1->size : bm0->size,
            swap ? bd0->pos : bm1->pos,
            swap ? bd0->rot : bm1->rot,
            swap ? bm0->size : bm1->size);

        // Swap contact points if necessary
        if (swap) {
            for (int i = 0; i < numContacts; i++) vec3_swap(contacts[i].pos0, contacts[i].pos1);
        }

        return numContacts;
    }
}