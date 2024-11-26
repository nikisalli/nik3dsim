#include <cmath>
#include "colliders.hpp"
#include "math.hpp"
#include "types.hpp"

namespace nik3dsim {
    Contact collide_sphere_plane(const niknum spos[3], const niknum ssize[3], const niknum ppos[3], const niknum prot[4], const niknum iprot[4]) {
        Contact contact = {0};
        niknum tmp[3] = {0, 0, 1}, planeNormal[3];
        vec3_quat_rotate(planeNormal, prot, tmp);
        niknum sphereToPlane[3];
        vec3_sub(sphereToPlane, spos, ppos);
        niknum distance = vec3_dot(planeNormal, sphereToPlane);
        contact.depth = fabs(distance) - ssize[0];
        niknum mult = copysignf(1.0f, distance);
        vec3_scl(contact.n, planeNormal, mult);
        vec3_addscl(contact.pos, spos, planeNormal, -distance);
        return contact;
    }

    Contact collide_capsule_plane(const niknum cpos[3], const niknum crot[4], const niknum csize[3], const niknum ppos[3], const niknum prot[4], const niknum iprot[4]) {
        Contact contact = {0};
        niknum tmp[3] = {0, 0, 1}, planeNormal[3], capsuleDir[3] = {0, 0, 1};
        vec3_quat_rotate(planeNormal, prot, tmp);
        vec3_quat_rotate(capsuleDir, crot, capsuleDir);
        niknum p1[3], p2[3], toP1[3], toP2[3];
        vec3_addscl(p1, cpos, capsuleDir, csize[1]/2);
        vec3_addscl(p2, cpos, capsuleDir, -csize[1]/2);
        vec3_sub(toP1, p1, ppos);
        vec3_sub(toP2, p2, ppos);
        niknum d1 = vec3_dot(planeNormal, toP1);
        niknum d2 = vec3_dot(planeNormal, toP2);
        niknum useFirst = fabs(d1) < fabs(d2);
        niknum distance = useFirst * d1 + (!useFirst) * d2;
        vec3_scl(contact.n, planeNormal, copysignf(1.0f, distance));
        vec3_addscl(contact.pos, useFirst ? p1 : p2, planeNormal, -distance);
        contact.depth = fabs(distance) - csize[0];
        return contact;
    }

    Contact collide_rigid_rigid(const RigidBodyModel* bm0, const RigidBodyModel* bm1, const RigidBodyData* bd0, const RigidBodyData* bd1) {
        // sort by type
        BodyType t0 = bm0->type < bm1->type ? bm0->type : bm1->type;
        BodyType t1 = bm0->type < bm1->type ? bm1->type : bm0->type;

        Contact contact;

        return contact;
    }

    Contact collide_rigid_static(const RigidBodyModel* bm0, const StaticBodyModel* bm1, const RigidBodyData* bd0) {
        // sort by type
        bool swap = bm0->type < bm1->type;
        niknum pos0[3], pos1[3], size0[3], size1[3], rot0[4], rot1[4], irot0[4], irot1[4];
        BodyType t0, t1;
        Contact contact;

        t0 = swap ? bm0->type : bm1->type; t1 = swap ? bm1->type : bm0->type;
        vec3_copy(pos0, swap ? bd0->pos : bm1->pos);
        vec3_copy(pos1, swap ? bm1->pos : bd0->pos);
        vec3_copy(size0, swap ? bm0->size : bm1->size);
        vec3_copy(size1, swap ? bm1->size : bm0->size);
        vec4_copy(rot0, swap ? bd0->rot : bm1->rot);
        vec4_copy(rot1, swap ? bm1->rot : bd0->rot);
        vec4_copy(irot0, swap ? bd0->invRot : bm1->invRot);
        vec4_copy(irot1, swap ? bm1->invRot : bd0->invRot);

        if (t0 == BODY_SPHERE && t1 == BODY_PLANE) contact = collide_sphere_plane(pos0, size0, pos1, rot1, irot1);
        else if (t0 == BODY_CAPSULE && t1 == BODY_PLANE) contact = collide_capsule_plane(pos0, rot0, size0, pos1, rot1, irot1);

        vec3_scl(contact.n, contact.n, 2.0 * swap - 1.0); // swap normal to point from contact to body0
        
        return contact;
    }
}