// SPDX-License-Identifier: MIT OR Apache-2.0
// Compiled with floating-point contraction off and without merging cos and sin into one call:
// arithmetic.h says why.
#include "arithmetic.h"

#include <algorithm>
#include <cmath>

namespace arithmetic {

Vec3 in_plane(const Vec3& o, const Vec3& x, const Vec3& y, double scale, double u, double v) {
    Vec3 out{};
    for (int k = 0; k < 3; k++) {
        out[k] = o[k] + scale * (x[k] * u + y[k] * v);
    }
    return out;
}

Vec2 on_circle(const Vec2& c, double r, double t) {
    return {c[0] + r * std::cos(t), c[1] + r * std::sin(t)};
}

Vec3 along(const Vec3& o, const Vec3& d, double t) {
    return {o[0] + d[0] * t, o[1] + d[1] * t, o[2] + d[2] * t};
}

Vec3 in_frame(const Vec3& o, const Vec3& x, const Vec3& y, const Vec3& z, const Vec3& c) {
    Vec3 out{};
    for (int k = 0; k < 3; k++) {
        out[k] = o[k] + x[k] * c[0] + y[k] * c[1] + z[k] * c[2];
    }
    return out;
}

Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

namespace {

// Neumaier's compensated summation from a start of 0.
double sum(const Vec3& terms) {
    double total = 0.0 + terms[0];
    double lost = 0.0;
    for (int i = 1; i < 3; i++) {
        const double x = terms[i];
        const double t = total + x;
        lost += std::fabs(total) >= std::fabs(x) ? (total - t) + x : (x - t) + total;
        total = t;
    }
    return lost != 0.0 && std::isfinite(lost) ? total + lost : total;
}

} // namespace

double dot(const Vec3& a, const Vec3& b) {
    return sum({a[0] * b[0], a[1] * b[1], a[2] * b[2]});
}

Vec3 unit(const Vec3& a) {
    const double n = std::sqrt(dot(a, a));
    return {a[0] / n, a[1] / n, a[2] / n};
}

double height_above(const Vec3& p, const Vec3& o, const Vec3& n) {
    return sum({(p[0] - o[0]) * n[0], (p[1] - o[1]) * n[1], (p[2] - o[2]) * n[2]});
}

Vec3 turn(const Vec3& v, const Vec3& k, double a) {
    const double c = std::cos(a);
    const double s = std::sin(a);
    const double kv = dot(k, v);
    const Vec3 kx = cross(k, v);
    Vec3 out{};
    for (int i = 0; i < 3; i++) {
        out[i] = v[i] * c + kx[i] * s + k[i] * kv * (1 - c);
    }
    return out;
}

double between(double a, double b, double t) {
    return a + t * (b - a);
}

// The method of CPython's math.hypot: the squares summed without loss, then one correction of the
// root.
double hypot(double a, double b) {
    a = std::fabs(a);
    b = std::fabs(b);
    const double max = std::max(a, b);
    if (!std::isfinite(a) || !std::isfinite(b) || max < 1e-300) {
        return std::hypot(a, b);
    }
    int exponent = 0;
    std::frexp(max, &exponent);
    const double scale = std::ldexp(1.0, -exponent);
    // 1 + Σ x·y as csum + frac1 + frac2: each product's and each sum's rounding error is kept.
    double csum = 1.0;
    double frac1 = 0.0;
    double frac2 = 0.0;
    const auto add_product = [&](double x, double y) {
        const double product = x * y;
        frac1 += std::fma(x, y, -product);
        const double total = csum + product;
        frac2 += (csum - total) + product;
        csum = total;
    };
    add_product(a * scale, a * scale);
    add_product(b * scale, b * scale);
    double h = std::sqrt(csum - 1.0 + (frac1 + frac2));
    add_product(-h, h);
    h += (csum - 1.0 + (frac1 + frac2)) / (2.0 * h);
    return h / scale;
}

} // namespace arithmetic
