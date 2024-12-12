#pragma once
#include "types.hpp"

namespace nik3dsim {
    int collide_rigid_rigid(Contact contacts[], const RigidBodyModel* bm0, const RigidBodyModel* bm1, const RigidBodyData* bd0, const RigidBodyData* bd1);
    int collide_rigid_static(Contact contacts[], const RigidBodyModel* bm0, const StaticBodyModel* bm1, const RigidBodyData* bd0);

    // Specialized collision detection functions (unified signature)
    int collide_sphere_sphere(Contact contacts[], const niknum pos0[3], const niknum rot0[4], const niknum size0[3], const niknum pos1[3], const niknum rot1[4], const niknum size1[3]);
    int collide_sphere_plane(Contact contacts[], const niknum pos0[3], const niknum rot0[4], const niknum size0[3], const niknum pos1[3], const niknum rot1[4], const niknum size1[3]);
    int collide_capsule_plane(Contact contacts[], const niknum pos0[3], const niknum rot0[4], const niknum size0[3], const niknum pos1[3], const niknum rot1[4], const niknum size1[3]);
    int collide_capsule_box(Contact contacts[], const niknum pos0[3], const niknum rot0[4], const niknum size0[3], const niknum pos1[3], const niknum rot1[4], const niknum size1[3]);
    int collide_sphere_box(Contact contacts[], const niknum pos0[3], const niknum rot0[4], const niknum size0[3], const niknum pos1[3], const niknum rot1[4], const niknum size1[3]);
    int collide_capsule_aabb(Contact contacts[], const niknum pos0[3], const niknum rot0[4], const niknum size0[3], const niknum pos1[3], const niknum rot1[4], const niknum size1[3]);
    int collide_box_plane(Contact contacts[], const niknum pos0[3], const niknum rot0[4], const niknum size0[3], const niknum pos1[3], const niknum rot1[4], const niknum size1[3]);
} // namespace nik3dsim