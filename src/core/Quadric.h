#pragma once

#include "Vec3.h"

struct Quadric {
    double m[4][4];

    Quadric() {
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                m[i][j] = 0.0;
            }
        }
    }

    Quadric& operator+=(const Quadric& o) {
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                m[i][j] += o.m[i][j];
            }
        }
        return *this;
    }

    static Quadric from_plane(const Vec3& n, double d) {
        Quadric q;
        double p[4] = {n.x, n.y, n.z, d};
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                q.m[i][j] = p[i] * p[j];
            }
        }
        return q;
    }

    double evaluate(const Vec3& v) const {
        double x = v.x, y = v.y, z = v.z;
        double v4[4] = {x, y, z, 1.0};
        double sum = 0.0;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                sum += v4[i] * m[i][j] * v4[j];
            }
        }
        return sum;
    }
};
