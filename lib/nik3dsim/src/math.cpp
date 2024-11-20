#include <math.h>

#include "math.hpp"

namespace nik3dsim {
    // Vec3 operations
    void vec3_copy(niknum res[3], niknum v[3]) {
        res[0] = v[0];
        res[1] = v[1];
        res[2] = v[2];
    }

    void vec3_zero(niknum res[3]) {
        res[0] = 0;
        res[1] = 0;
        res[2] = 0;
    }

    void vec3_add(niknum res[3], niknum a[3], niknum b[3]) {
        res[0] = a[0] + b[0];
        res[1] = a[1] + b[1];
        res[2] = a[2] + b[2];
    }

    void vec3_addscl(niknum res[3], niknum a[3], niknum b[3], niknum s) {
        res[0] = a[0] + b[0] * s;
        res[1] = a[1] + b[1] * s;
        res[2] = a[2] + b[2] * s;
    }

    void vec3_addto(niknum res[3], niknum b[3]) {
        res[0] += b[0];
        res[1] += b[1];
        res[2] += b[2];
    }

    void vec3_sub(niknum res[3], niknum a[3], niknum b[3]) {
        res[0] = a[0] - b[0];
        res[1] = a[1] - b[1];
        res[2] = a[2] - b[2];
    }

    void vec3_subto(niknum res[3], niknum b[3]) {
        res[0] -= b[0];
        res[1] -= b[1];
        res[2] -= b[2];
    }

    void vec3_scl(niknum res[3], niknum a[3], niknum s) {
        res[0] = a[0] * s;
        res[1] = a[1] * s;
        res[2] = a[2] * s;
    }

    niknum vec3_dot(niknum a[3], niknum b[3]) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    void vec3_cross(niknum res[3], niknum a[3], niknum b[3]) {
        res[0] = a[1] * b[2] - a[2] * b[1];
        res[1] = a[2] * b[0] - a[0] * b[2];
        res[2] = a[0] * b[1] - a[1] * b[0];
    }

    niknum vec3_length(niknum v[3]) {
        return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    }

    niknum vec3_normalize(niknum res[3], niknum v[3]) {
        niknum len = vec3_length(v);
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

    void vec3_quat_rotate(niknum res[3], niknum q[4], niknum v[3]) {
        // Using the formula: v' = q * v * q^(-1)
        // Optimized version that doesn't construct intermediate quaternions
        niknum x2 = q[0] * 2.0f;
        niknum y2 = q[1] * 2.0f;
        niknum z2 = q[2] * 2.0f;
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
    }

    void vec3_mul(niknum res[3], niknum a[3], niknum b[3]) {
        res[0] = a[0] * b[0];
        res[1] = a[1] * b[1];
        res[2] = a[2] * b[2];
    }

    // Vec4 operations
    void vec4_copy(niknum res[4], niknum v[4]) {
        res[0] = v[0];
        res[1] = v[1];
        res[2] = v[2];
        res[3] = v[3];
    }

    void vec4_zero(niknum res[4]) {
        res[0] = 0.0f;
        res[1] = 0.0f;
        res[2] = 0.0f;
        res[3] = 0.0f;
    }

    void vec4_add(niknum res[4], niknum a[4], niknum b[4]) {
        res[0] = a[0] + b[0];
        res[1] = a[1] + b[1];
        res[2] = a[2] + b[2];
        res[3] = a[3] + b[3];
    }

    void vec4_addscl(niknum res[4], niknum a[4], niknum b[4], niknum s) {
        res[0] = a[0] + b[0] * s;
        res[1] = a[1] + b[1] * s;
        res[2] = a[2] + b[2] * s;
        res[3] = a[3] + b[3] * s;
    }

    void vec4_sub(niknum res[4], niknum a[4], niknum b[4]) {
        res[0] = a[0] - b[0];
        res[1] = a[1] - b[1];
        res[2] = a[2] - b[2];
        res[3] = a[3] - b[3];
    }

    void vec4_scl(niknum res[4], niknum a[4], niknum s) {
        res[0] = a[0] * s;
        res[1] = a[1] * s;
        res[2] = a[2] * s;
        res[3] = a[3] * s;
    }

    niknum vec4_normalize(niknum res[4], niknum v[4]) {
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

    void vec4_mul(niknum res[4], niknum a[4], niknum b[4]) {
        res[0] = a[0] * b[0];
        res[1] = a[1] * b[1];
        res[2] = a[2] * b[2];
        res[3] = a[3] * b[3];
    }

    // Quat operations
    void quat_mul(niknum res[4], niknum a[4], niknum b[4]) {
        res[0] = a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1];  // x
        res[1] = a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0];  // y
        res[2] = a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3];  // z
        res[3] = a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2];  // w
    }

    void quat_normalize(niknum res[4], niknum q[4]) {
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

    void quat_conj(niknum res[4], niknum q[4]) {
        res[0] = -q[0];  // -x
        res[1] = -q[1];  // -y
        res[2] = -q[2];  // -z
        res[3] = q[3];   // w
    }
} // namespace nik3dsim