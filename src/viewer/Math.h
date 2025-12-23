#pragma once

#include <cmath>
#include "../core/Vec3.h"

struct Mat4 {
    float m[16];

    static Mat4 identity() {
        Mat4 r{};
        r.m[0] = r.m[5] = r.m[10] = r.m[15] = 1.0f;
        return r;
    }

    static Mat4 perspective(float fovy, float aspect, float znear, float zfar) {
        float tan_half = std::tan(fovy * 0.5f);
        Mat4 r{};
        r.m[0] = 1.0f / (aspect * tan_half);
        r.m[5] = 1.0f / tan_half;
        r.m[10] = -(zfar + znear) / (zfar - znear);
        r.m[11] = -1.0f;
        r.m[14] = -(2.0f * zfar * znear) / (zfar - znear);
        return r;
    }

    static Mat4 look_at(const Vec3& eye, const Vec3& center, const Vec3& up) {
        Vec3 f = normalize(center - eye);
        Vec3 s = normalize(cross(f, up));
        Vec3 u = cross(s, f);

        Mat4 r = identity();
        r.m[0] = static_cast<float>(s.x);
        r.m[4] = static_cast<float>(s.y);
        r.m[8] = static_cast<float>(s.z);
        r.m[1] = static_cast<float>(u.x);
        r.m[5] = static_cast<float>(u.y);
        r.m[9] = static_cast<float>(u.z);
        r.m[2] = static_cast<float>(-f.x);
        r.m[6] = static_cast<float>(-f.y);
        r.m[10] = static_cast<float>(-f.z);
        r.m[12] = static_cast<float>(-dot(s, eye));
        r.m[13] = static_cast<float>(-dot(u, eye));
        r.m[14] = static_cast<float>(dot(f, eye));
        return r;
    }
};

inline Mat4 operator*(const Mat4& a, const Mat4& b) {
    Mat4 r{};
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            r.m[i + j * 4] =
                a.m[i + 0 * 4] * b.m[0 + j * 4] +
                a.m[i + 1 * 4] * b.m[1 + j * 4] +
                a.m[i + 2 * 4] * b.m[2 + j * 4] +
                a.m[i + 3 * 4] * b.m[3 + j * 4];
        }
    }
    return r;
}
