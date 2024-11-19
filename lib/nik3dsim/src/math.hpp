#pragma once

#define niknum float

namespace nik3dsim {
    // Vec3 operations
    void vec3_copy(niknum res[3], niknum v[3]);
    void vec3_zero(niknum res[3]);
    void vec3_add(niknum res[3], niknum a[3], niknum b[3]);
    void vec3_sub(niknum res[3], niknum a[3], niknum b[3]);
    void vec3_scale(niknum res[3], niknum a[3], niknum s);
    niknum vec3_dot(niknum a[3], niknum b[3]);
    void vec3_cross(niknum res[3], niknum a[3], niknum b[3]);
    niknum vec3_length(niknum v[3]);
    void vec3_normalize(niknum res[3], niknum v[3]);
    void vec3_quat_rotate(niknum res[3], niknum q[4], niknum v[3]);
    void vec3_mult(niknum res[3], niknum a[3], niknum b[3]);

    // Quat operations
    void quat_multiply(niknum res[4], niknum a[4], niknum b[4]);
    void quat_normalize(niknum res[4], niknum q[4]);
    void quat_conjugate(niknum res[4], niknum q[4]);

}  // namespace nik3dsim