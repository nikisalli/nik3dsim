#pragma once
#include "types.hpp"

namespace nik3dsim {
    int collide_rigid_rigid(Contact contacts[], const RigidBodyModel* bm0, const RigidBodyModel* bm1, const RigidBodyData* bd0, const RigidBodyData* bd1);
    int collide_rigid_static(Contact contacts[], const RigidBodyModel* bm0, const StaticBodyModel* bm1, const RigidBodyData* bd0);
    int collide_capsule_box(Contact contacts[], const niknum cpos[3], const niknum crot[4], const niknum csize[3], const niknum bpos[3], const niknum brot[4], const niknum ibrot[4], const niknum bsize[3]);
    int collide_capsule_aabb(Contact contacts[], const niknum cpos[3], const niknum crot[4], const niknum csize[3], const niknum bpos[3], const niknum bsize[3]);
    int collide_box_plane(Contact contacts[], const niknum bpos[3], const niknum brot[4], const niknum bsize[3], const niknum ppos[3], const niknum prot[4], const niknum iprot[4]);
} // namespace nik3dsim