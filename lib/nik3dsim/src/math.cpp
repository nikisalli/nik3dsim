#include <math.h>

#include "math.hpp"

namespace nik3dsim {
    // Vec3 operations
    Vec3 vec3_create(float x, float y, float z) {
        Vec3 v = {x, y, z};
        return v;
    }

    Vec3 vec3_add(Vec3 a, Vec3 b) {
        return vec3_create(a.x + b.x, a.y + b.y, a.z + b.z);
    }

    Vec3 vec3_sub(Vec3 a, Vec3 b) {
        return vec3_create(a.x - b.x, a.y - b.y, a.z - b.z);
    }

    Vec3 vec3_scale(Vec3 v, float s) {
        return vec3_create(v.x * s, v.y * s, v.z * s);
    }

    Vec3 vec3_cross(Vec3 a, Vec3 b) {
        return vec3_create(
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x
        );
    }

    float vec3_length(Vec3 v) {
        return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
    }

    Vec3 vec3_normalize(Vec3 v) {
        float len = vec3_length(v);
        if (len > 0) {
            return vec3_scale(v, 1.0f / len);
        }
        return v;
    }

    // Quat operations
    Quat quat_create(float x, float y, float z, float w) {
        Quat q = {x, y, z, w};
        return q;
    }

    Quat quat_multiply(Quat a, Quat b) {
        return quat_create(
            a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
            a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
        );
    }

    Quat quat_invert(Quat q) {
        float norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
        if (norm > 0) {
            float invNorm = 1.0f / norm;
            return quat_create(-q.x * invNorm, -q.y * invNorm, -q.z * invNorm, q.w * invNorm);
        }
        return q;
    }

    Quat quat_normalize(Quat q) {
        float norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
        if (norm > 0) {
            float invNorm = 1.0f / norm;
            return quat_create(q.x * invNorm, q.y * invNorm, q.z * invNorm, q.w * invNorm);
        }
        return q;
    }

    Quat quat_conjugate(Quat q) {
        return quat_create(-q.x, -q.y, -q.z, q.w);
    }
}