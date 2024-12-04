#pragma once

#include <math.h>

#define niknum float

namespace nik3dsim {
    // Vec3 operations
    inline void vec3_copy(niknum res[3], const niknum v[3]) {
        res[0] = v[0];
        res[1] = v[1];
        res[2] = v[2];
    }

    inline void vec3_zero(niknum res[3]) {
        res[0] = 0;
        res[1] = 0;
        res[2] = 0;
    }

    inline void vec3_add(niknum res[3], const niknum a[3], const niknum b[3]) {
        res[0] = a[0] + b[0];
        res[1] = a[1] + b[1];
        res[2] = a[2] + b[2];
    }

    inline void vec3_addscl(niknum res[3], const niknum a[3], const niknum b[3], const niknum s) {
        res[0] = a[0] + b[0] * s;
        res[1] = a[1] + b[1] * s;
        res[2] = a[2] + b[2] * s;
    }

    inline void vec3_addto(niknum res[3], const niknum b[3]) {
        res[0] += b[0];
        res[1] += b[1];
        res[2] += b[2];
    }

    inline void vec3_sub(niknum res[3], const niknum a[3], const niknum b[3]) {
        res[0] = a[0] - b[0];
        res[1] = a[1] - b[1];
        res[2] = a[2] - b[2];
    }

    inline void vec3_subto(niknum res[3], const niknum b[3]) {
        res[0] -= b[0];
        res[1] -= b[1];
        res[2] -= b[2];
    }

    inline void vec3_scl(niknum res[3], const niknum a[3], const niknum s) {
        res[0] = a[0] * s;
        res[1] = a[1] * s;
        res[2] = a[2] * s;
    }

    inline niknum vec3_dot(const niknum a[3], const niknum b[3]) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    inline void vec3_cross(niknum res[3], const niknum a[3], const niknum b[3]) {
        res[0] = a[1] * b[2] - a[2] * b[1];
        res[1] = a[2] * b[0] - a[0] * b[2];
        res[2] = a[0] * b[1] - a[1] * b[0];
    }

    inline niknum vec3_length(const niknum v[3]) {
        return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    }

    inline niknum vec3_normalize(niknum res[3], const niknum v[3]) {
        niknum len = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        if (len > 0) {
            niknum invLen = 1.0f / len;
            res[0] = v[0] * invLen;
            res[1] = v[1] * invLen;
            res[2] = v[2] * invLen;
        } else {
            res[0] = v[0];
            res[1] = v[1];
            res[2] = v[2];
        }
        return len;
    }

    inline void vec3_quat_rotate(niknum res[3], const niknum q[4], const niknum v[3]) {
        // Using the formula: v' = q * v * q^(-1)
        // Optimized version that doesn't construct intermediate quaternions
        niknum x2  = q[0] * 2.0f;
        niknum y2  = q[1] * 2.0f;
        niknum z2  = q[2] * 2.0f;
        niknum xx2 = q[0] * x2;
        niknum xy2 = q[0] * y2;
        niknum xz2 = q[0] * z2;
        niknum yy2 = q[1] * y2;
        niknum yz2 = q[1] * z2;
        niknum zz2 = q[2] * z2;
        niknum wx2 = q[3] * x2;
        niknum wy2 = q[3] * y2;
        niknum wz2 = q[3] * z2;
        res[0] = v[0] * (1.0f - yy2 - zz2) + v[1] * (xy2 - wz2) + v[2] * (xz2 + wy2);
        res[1] = v[0] * (xy2 + wz2) + v[1] * (1.0f - xx2 - zz2) + v[2] * (yz2 - wx2);
        res[2] = v[0] * (xz2 - wy2) + v[1] * (yz2 + wx2) + v[2] * (1.0f - xx2 - yy2);
    
        // Compute t = 2 * cross(q.xyz, v)
        // niknum t[3];
        // t[0] = 2.0f * (q[1] * v[2] - q[2] * v[1]);
        // t[1] = 2.0f * (q[2] * v[0] - q[0] * v[2]);
        // t[2] = 2.0f * (q[0] * v[1] - q[1] * v[0]);
        // // Compute cross(q.xyz, t)
        // niknum qxt[3];
        // qxt[0] = q[1] * t[2] - q[2] * t[1];
        // qxt[1] = q[2] * t[0] - q[0] * t[2];
        // qxt[2] = q[0] * t[1] - q[1] * t[0];
        // // Combine all terms: v + qw * t + cross(q.xyz, t)
        // res[0] = v[0] + q[3] * t[0] + qxt[0];
        // res[1] = v[1] + q[3] * t[1] + qxt[1];
        // res[2] = v[2] + q[3] * t[2] + qxt[2];
    }

    inline void quat2rotmat(const niknum q[4], niknum m[9]) {
        niknum x2 = q[0] * 2.0f;
        niknum y2 = q[1] * 2.0f;
        niknum z2 = q[2] * 2.0f;
        
        // Diagonal terms
        m[0] = 1.0f - (q[1] * y2 + q[2] * z2);  // 1 - yy2 - zz2
        m[4] = 1.0f - (q[0] * x2 + q[2] * z2);  // 1 - xx2 - zz2
        m[8] = 1.0f - (q[0] * x2 + q[1] * y2);  // 1 - xx2 - yy2
        
        // Off-diagonal terms
        niknum xy2 = q[0] * y2;
        niknum xz2 = q[0] * z2;
        niknum yz2 = q[1] * z2;
        niknum wx2 = q[3] * x2;
        niknum wy2 = q[3] * y2;
        niknum wz2 = q[3] * z2;
        
        m[1] = xy2 - wz2;
        m[2] = xz2 + wy2;
        m[3] = xy2 + wz2;
        m[5] = yz2 - wx2;
        m[6] = xz2 - wy2;
        m[7] = yz2 + wx2;
    }

    inline void vec3_matmul(niknum res[3], const niknum m[9], const niknum v[3]) {
        res[0] = m[0] * v[0] + m[1] * v[1] + m[2] * v[2];
        res[1] = m[3] * v[0] + m[4] * v[1] + m[5] * v[2];
        res[2] = m[6] * v[0] + m[7] * v[1] + m[8] * v[2];
    }

    inline void vec3_mul(niknum res[3], const niknum a[3], const niknum b[3]) {
        res[0] = a[0] * b[0];
        res[1] = a[1] * b[1];
        res[2] = a[2] * b[2];
    }

    inline void vec3_swap(niknum a[3], niknum b[3]) {
        niknum tmp[3];
        vec3_copy(tmp, a);
        vec3_copy(a, b);
        vec3_copy(b, tmp);
    }

    // Vec4 operations
    inline void vec4_copy(niknum res[4], const niknum v[4]) {
        res[0] = v[0];
        res[1] = v[1];
        res[2] = v[2];
        res[3] = v[3];
    }

    inline void vec4_zero(niknum res[4]) {
        res[0] = 0.0f;
        res[1] = 0.0f;
        res[2] = 0.0f;
        res[3] = 0.0f;
    }

    inline void vec4_add(niknum res[4], const niknum a[4], const niknum b[4]) {
        res[0] = a[0] + b[0];
        res[1] = a[1] + b[1];
        res[2] = a[2] + b[2];
        res[3] = a[3] + b[3];
    }

    inline void vec4_addscl(niknum res[4], const niknum a[4], const niknum b[4], const niknum s) {
        res[0] = a[0] + b[0] * s;
        res[1] = a[1] + b[1] * s;
        res[2] = a[2] + b[2] * s;
        res[3] = a[3] + b[3] * s;
    }

    inline void vec4_sub(niknum res[4], const niknum a[4], const niknum b[4]) {
        res[0] = a[0] - b[0];
        res[1] = a[1] - b[1];
        res[2] = a[2] - b[2];
        res[3] = a[3] - b[3];
    }

    inline void vec4_scl(niknum res[4], const niknum a[4], const niknum s) {
        res[0] = a[0] * s;
        res[1] = a[1] * s;
        res[2] = a[2] * s;
        res[3] = a[3] * s;
    }

    inline niknum vec4_normalize(niknum res[4], const niknum v[4]) {
        niknum len = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2] + v[3] * v[3]);
        if (len > 0) {
            niknum invLen = 1.0f / len;
            res[0] = v[0] * invLen;
            res[1] = v[1] * invLen;
            res[2] = v[2] * invLen;
            res[3] = v[3] * invLen;
        } else {
            res[0] = v[0];
            res[1] = v[1];
            res[2] = v[2];
            res[3] = v[3];
        }
        return len;
    }

    inline void vec4_mul(niknum res[4], const niknum a[4], const niknum b[4]) {
        res[0] = a[0] * b[0];
        res[1] = a[1] * b[1];
        res[2] = a[2] * b[2];
        res[3] = a[3] * b[3];
    }

    // Quat operations
    inline void quat_mul(niknum res[4], const niknum a[4], const niknum b[4]) {
        res[0] = a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1];  // x
        res[1] = a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0];  // y
        res[2] = a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3];  // z
        res[3] = a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2];  // w
    }

    inline void quat_normalize(niknum res[4], const niknum q[4]) {
        niknum norm = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
        if (norm > 0) {
            niknum invNorm = 1.0f / sqrtf(norm);
            res[0] = q[0] * invNorm;
            res[1] = q[1] * invNorm;
            res[2] = q[2] * invNorm;
            res[3] = q[3] * invNorm;
        } else {
            res[0] = q[0];
            res[1] = q[1];
            res[2] = q[2];
            res[3] = q[3];
        }
    }

    inline void quat_conj(niknum res[4], const niknum q[4]) {
        res[0] = -q[0];  // -x
        res[1] = -q[1];  // -y
        res[2] = -q[2];  // -z
        res[3] = q[3];   // w
    }

    inline void quat_get_twist(niknum result[4], const niknum q[4], const niknum direction[3]) {
        niknum dot_prod = direction[0] * q[0] + direction[1] * q[1] + direction[2] * q[2];
        niknum proj[3];

        vec3_scl(proj, direction, dot_prod);
        result[0] = proj[0];
        result[1] = proj[1];
        result[2] = proj[2];
        result[3] = q[3];
        
        quat_normalize(result, result);
        
        if (dot_prod < 0.0f) {
            result[0] = -result[0];
            result[1] = -result[1];
            result[2] = -result[2];
            result[3] = -result[3];
        }
    }

    inline void euler2quat(niknum res[4], const niknum euler[3]) {
        niknum cx = cosf(euler[0] * 0.5f);
        niknum cy = cosf(euler[1] * 0.5f);
        niknum cz = cosf(euler[2] * 0.5f);
        niknum sx = sinf(euler[0] * 0.5f);
        niknum sy = sinf(euler[1] * 0.5f);
        niknum sz = sinf(euler[2] * 0.5f);
        
        res[0] = sx * cy * cz - cx * sy * sz;  // x
        res[1] = cx * sy * cz + sx * cy * sz;  // y
        res[2] = cx * cy * sz - sx * sy * cz;  // z
        res[3] = cx * cy * cz + sx * sy * sz;  // w
    }
} // namespace nik3dsim