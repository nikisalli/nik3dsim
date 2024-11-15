#pragma once

namespace nik3dsim {
    // Vec3 struct
    typedef struct {
        float x;
        float y;
        float z;
    } Vec3;

    // Quat struct
    typedef struct {
        float x;
        float y;
        float z;
        float w;
    } Quat;

    // Vec3 operations
    Vec3 vec3_create(float x, float y, float z);
    Vec3 vec3_add(Vec3 a, Vec3 b);
    Vec3 vec3_sub(Vec3 a, Vec3 b);
    Vec3 vec3_scale(Vec3 v, float s);
    Vec3 vec3_cross(Vec3 a, Vec3 b);
    float vec3_length(Vec3 v);
    Vec3 vec3_normalize(Vec3 v);

    // Quat operations
    Quat quat_create(float x, float y, float z, float w);
    Quat quat_multiply(Quat a, Quat b);
    Quat quat_invert(Quat q);
    Quat quat_normalize(Quat q);
    Quat quat_conjugate(Quat q);

}  // namespace nik3dsim