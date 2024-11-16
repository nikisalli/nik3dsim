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

    float vec3_dot(Vec3 a, Vec3 b) {
        return a.x * b.x + a.y * b.y + a.z * b.z;
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

    Vec3 vec3_quat_rotate(Quat q, Vec3 v) {
        // Using the formula: v' = q * v * q^(-1)
        // Optimized version that doesn't construct intermediate quaternions
        
        float x2 = q.x * 2.0f;
        float y2 = q.y * 2.0f;
        float z2 = q.z * 2.0f;
        float xx2 = q.x * x2;
        float xy2 = q.x * y2;
        float xz2 = q.x * z2;
        float yy2 = q.y * y2;
        float yz2 = q.y * z2;
        float zz2 = q.z * z2;
        float wx2 = q.w * x2;
        float wy2 = q.w * y2;
        float wz2 = q.w * z2;

        return vec3_create(
            v.x * (1.0f - yy2 - zz2) + v.y * (xy2 - wz2) + v.z * (xz2 + wy2),
            v.x * (xy2 + wz2) + v.y * (1.0f - xx2 - zz2) + v.z * (yz2 - wx2),
            v.x * (xz2 - wy2) + v.y * (yz2 + wx2) + v.z * (1.0f - xx2 - yy2)
        );
    }

    // Vec4 operations
    Vec4 vec4_create(float x, float y, float z, float w) {
        Vec4 v = {x, y, z, w};
        return v;
    }

    Vec4 vec4_add(Vec4 a, Vec4 b) {
        return vec4_create(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w);
    }

    Vec4 vec4_sub(Vec4 a, Vec4 b) {
        return vec4_create(a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w);
    }

    Vec4 vec4_scale(Vec4 v, float s) {
        return vec4_create(v.x * s, v.y * s, v.z * s, v.w * s);
    }

    float vec4_dot(Vec4 a, Vec4 b) {
        return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
    }

    Vec4 vec4_cross(Vec4 a, Vec4 b) {
        return vec4_create(
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x,
            0.0f
        );
    }

    float vec4_length(Vec4 v) {
        return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z + v.w * v.w);
    }

    Vec4 vec4_normalize(Vec4 v) {
        float len = vec4_length(v);
        if (len > 0) {
            return vec4_scale(v, 1.0f / len);
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

    // Mat3 operations
    Mat3 mat3_create(
        float m00, float m01, float m02,  // first row
        float m10, float m11, float m12,  // second row
        float m20, float m21, float m22   // third row
    ) {
        Mat3 m;
        // Store in column major order
        m.m[0] = m00; m.m[3] = m01; m.m[6] = m02;
        m.m[1] = m10; m.m[4] = m11; m.m[7] = m12;
        m.m[2] = m20; m.m[5] = m21; m.m[8] = m22;
        return m;
    }

    Mat3 mat3_identity() {
        return mat3_create(
            1, 0, 0,
            0, 1, 0,
            0, 0, 1
        );
    }

    Mat3 mat3_scale(Mat3 m, float s) {
        Mat3 result;
        for (int i = 0; i < 9; i++) {
            result.m[i] = m.m[i] * s;
        }
        return result;
    }

    Mat3 mat3_add(Mat3 a, Mat3 b) {
        Mat3 result;
        for (int i = 0; i < 9; i++) {
            result.m[i] = a.m[i] + b.m[i];
        }
        return result;
    }

    Mat3 mat3_multiply(Mat3 a, Mat3 b) {
        Mat3 result;
        
        // Unroll the loops for better performance
        result.m[0] = a.m[0] * b.m[0] + a.m[3] * b.m[1] + a.m[6] * b.m[2];
        result.m[1] = a.m[1] * b.m[0] + a.m[4] * b.m[1] + a.m[7] * b.m[2];
        result.m[2] = a.m[2] * b.m[0] + a.m[5] * b.m[1] + a.m[8] * b.m[2];
        
        result.m[3] = a.m[0] * b.m[3] + a.m[3] * b.m[4] + a.m[6] * b.m[5];
        result.m[4] = a.m[1] * b.m[3] + a.m[4] * b.m[4] + a.m[7] * b.m[5];
        result.m[5] = a.m[2] * b.m[3] + a.m[5] * b.m[4] + a.m[8] * b.m[5];
        
        result.m[6] = a.m[0] * b.m[6] + a.m[3] * b.m[7] + a.m[6] * b.m[8];
        result.m[7] = a.m[1] * b.m[6] + a.m[4] * b.m[7] + a.m[7] * b.m[8];
        result.m[8] = a.m[2] * b.m[6] + a.m[5] * b.m[7] + a.m[8] * b.m[8];
        
        return result;
    }

    Vec3 mat3_multiply_vec3(Mat3 m, Vec3 v) {
        return vec3_create(
            m.m[0] * v.x + m.m[3] * v.y + m.m[6] * v.z,
            m.m[1] * v.x + m.m[4] * v.y + m.m[7] * v.z,
            m.m[2] * v.x + m.m[5] * v.y + m.m[8] * v.z
        );
    }

    Mat3 mat3_transpose(Mat3 m) {
        return mat3_create(
            m.m[0], m.m[1], m.m[2],
            m.m[3], m.m[4], m.m[5],
            m.m[6], m.m[7], m.m[8]
        );
    }

    Mat3 quat_to_mat3(Quat q) {
        float x2 = q.x * 2.0f;
        float y2 = q.y * 2.0f;
        float z2 = q.z * 2.0f;
        float xx = q.x * x2;
        float xy = q.x * y2;
        float xz = q.x * z2;
        float yy = q.y * y2;
        float yz = q.y * z2;
        float zz = q.z * z2;
        float wx = q.w * x2;
        float wy = q.w * y2;
        float wz = q.w * z2;

        return mat3_create(
            1.0f - (yy + zz), xy - wz, xz + wy,
            xy + wz, 1.0f - (xx + zz), yz - wx,
            xz - wy, yz + wx, 1.0f - (xx + yy)
        );
    }

    Quat mat3_to_quat(Mat3 m) {
        float trace = m.m[0] + m.m[4] + m.m[8];
        Quat q;

        if (trace > 0.0f) {
            float s = sqrtf(trace + 1.0f);
            q.w = s * 0.5f;
            s = 0.5f / s;
            q.x = (m.m[5] - m.m[7]) * s;
            q.y = (m.m[6] - m.m[2]) * s;
            q.z = (m.m[1] - m.m[3]) * s;
        } else {
            // Find largest diagonal element
            int i = 0;
            if (m.m[4] > m.m[0]) i = 1;
            if (m.m[8] > m.m[i*3+i]) i = 2;

            static const int NEXT[3] = {1, 2, 0};
            int j = NEXT[i];
            int k = NEXT[j];

            float s = sqrtf(m.m[i*3+i] - m.m[j*3+j] - m.m[k*3+k] + 1.0f);
            float* qv[3] = {&q.x, &q.y, &q.z};
            *qv[i] = s * 0.5f;
            s = 0.5f / s;
            q.w = (m.m[k*3+j] - m.m[j*3+k]) * s;
            *qv[j] = (m.m[j*3+i] + m.m[i*3+j]) * s;
            *qv[k] = (m.m[k*3+i] + m.m[i*3+k]) * s;
        }

        return quat_normalize(q);
    }

    Mat3 mat3_skew_symmetric(Vec3 v) {
        return mat3_create(
            0, -v.z, v.y,
            v.z, 0, -v.x,
            -v.y, v.x, 0
        );
    }
}