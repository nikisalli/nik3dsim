#pragma once
#include "types.hpp"

namespace nik3dsim {
    Contact collide_rigid_rigid(const RigidBodyModel* bm0, const RigidBodyModel* bm1, const RigidBodyData* bd0, const RigidBodyData* bd1);
    Contact collide_rigid_static(const RigidBodyModel* bm0, const StaticBodyModel* bm1, const RigidBodyData* bd0);
    
} // namespace nik3dsim