#pragma once
#include "types.hpp"

namespace nik3dsim {
    Contact collide_rigid_rigid(const RigidBodyModel* bm0, const RigidBodyModel* bm1, const RigidBodyData* bd0, const RigidBodyData* bd1);
    Contact collide_rigid_static(const RigidBodyModel* bm0, const StaticBodyModel* bm1, const RigidBodyData* bd0);
    Contact collide_capsule_box(const niknum cpos[3], const niknum crot[4], const niknum csize[3], const niknum bpos[3], const niknum brot[4], const niknum ibrot[4], const niknum bsize[3]);
} // namespace nik3dsim