#pragma once

namespace nik3dsim {
    // Vec3 struct
    typedef struct {
        float x;
        float y;
        float z;
    } Vec3;

    // Vec4 struct
    typedef struct {
        float x;
        float y;
        float z;
        float w;
    } Vec4;

    // Quat struct
    typedef struct {
        float x;
        float y;
        float z;
        float w;
    } Quat;

    // Mat3 struct
    typedef struct {
        float m[9];
    } Mat3;

    // Vec3 operations
    Vec3 vec3_create(float x, float y, float z);
    Vec3 vec3_add(Vec3 a, Vec3 b);
    Vec3 vec3_sub(Vec3 a, Vec3 b);
    Vec3 vec3_scale(Vec3 v, float s);
    float vec3_dot(Vec3 a, Vec3 b);
    Vec3 vec3_cross(Vec3 a, Vec3 b);
    float vec3_length(Vec3 v);
    Vec3 vec3_normalize(Vec3 v);
    Vec3 vec3_quat_rotate(Quat q, Vec3 v);

    // Vec4 operations
    Vec4 vec4_create(float x, float y, float z, float w);
    Vec4 vec4_add(Vec4 a, Vec4 b);
    Vec4 vec4_sub(Vec4 a, Vec4 b);
    Vec4 vec4_scale(Vec4 v, float s);
    float vec4_dot(Vec4 a, Vec4 b);
    Vec4 vec4_cross(Vec4 a, Vec4 b);
    float vec4_length(Vec4 v);
    Vec4 vec4_normalize(Vec4 v);

    // Quat operations
    Quat quat_create(float x, float y, float z, float w);
    Quat quat_multiply(Quat a, Quat b);
    Quat quat_invert(Quat q);
    Quat quat_normalize(Quat q);
    Quat quat_conjugate(Quat q);

    // Forward declarations
    Mat3 mat3_create(float m00, float m01, float m02, float m10, float m11, float m12, float m20, float m21, float m22);
    Mat3 mat3_identity();
    Mat3 mat3_scale(Mat3 m, float s);
    Mat3 mat3_add(Mat3 a, Mat3 b);
    Mat3 mat3_multiply(Mat3 a, Mat3 b);
    Vec3 mat3_multiply_vec3(Mat3 m, Vec3 v);
    Mat3 mat3_transpose(Mat3 m);
    Mat3 quat_to_mat3(Quat q);
    Quat mat3_to_quat(Mat3 m);
    Mat3 mat3_skew_symmetric(Vec3 v);

}  // namespace nik3dsim