#pragma once

#define niknum float
#define nikpi 3.1415926535897932384626433832795

namespace nik3dsim {
    // Vec3 operations
    void vec3_copy(niknum res[3], niknum v[3]);
    void vec3_zero(niknum res[3]);
    void vec3_add(niknum res[3], niknum a[3], niknum b[3]);
    void vec3_addto(niknum res[3], niknum b[3]);
    void vec3_addscl(niknum res[3], niknum a[3], niknum b[3], niknum s);
    void vec3_sub(niknum res[3], niknum a[3], niknum b[3]);
    void vec3_subto(niknum res[3], niknum b[3]);
    void vec3_scl(niknum res[3], niknum a[3], niknum s);
    niknum vec3_dot(niknum a[3], niknum b[3]);
    void vec3_cross(niknum res[3], niknum a[3], niknum b[3]);
    niknum vec3_length(niknum v[3]);
    niknum vec3_normalize(niknum res[3], niknum v[3]);
    void vec3_quat_rotate(niknum res[3], niknum q[4], niknum v[3]);
    void vec3_mul(niknum res[3], niknum a[3], niknum b[3]);

    // Vec4 operations
    void vec4_copy(niknum res[4], niknum v[4]);
    void vec4_zero(niknum res[4]);
    void vec4_add(niknum res[4], niknum a[4], niknum b[4]);
    void vec4_addscl(niknum res[4], niknum a[4], niknum b[4], niknum s);
    void vec4_sub(niknum res[4], niknum a[4], niknum b[4]);
    void vec4_scl(niknum res[4], niknum a[4], niknum s);
    niknum vec4_normalize(niknum res[4], niknum v[4]);
    void vec4_mul(niknum res[4], niknum a[4], niknum b[4]);

    // Quat operations
    void quat_mul(niknum res[4], niknum a[4], niknum b[4]);
    void quat_normalize(niknum res[4], niknum q[4]);
    void quat_conj(niknum res[4], niknum q[4]);

}  // namespace nik3dsim