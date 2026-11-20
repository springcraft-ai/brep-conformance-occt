// SPDX-License-Identifier: MIT
// The arithmetic the adapter does itself, on the request's numbers, before it calls OpenCascade.
// arithmetic.cpp is compiled with floating-point contraction off, so every result is the same on
// every compiler and platform: a fused multiply-add rounds once where a multiply and an add round
// twice, and one last digit can decide a case close to a tolerance.
#pragma once

#include <array>

namespace arithmetic {

using Vec2 = std::array<double, 2>;
using Vec3 = std::array<double, 3>;

// o + scale·(x·u + y·v)
Vec3 in_plane(const Vec3& o, const Vec3& x, const Vec3& y, double scale, double u, double v);
// c + r·(cos t, sin t)
Vec2 on_circle(const Vec2& c, double r, double t);
// o + d·t
Vec3 along(const Vec3& o, const Vec3& d, double t);
// o + x·c₀ + y·c₁ + z·c₂
Vec3 in_frame(const Vec3& o, const Vec3& x, const Vec3& y, const Vec3& z, const Vec3& c);
Vec3 cross(const Vec3& a, const Vec3& b);
// a·b, its three products summed with the rounding error carried along, by Neumaier's method.
double dot(const Vec3& a, const Vec3& b);
// a over its length, the length the root of dot(a, a).
Vec3 unit(const Vec3& a);
// (p − o)·n, summed as dot is.
double height_above(const Vec3& p, const Vec3& o, const Vec3& n);
// `v` turned by the angle `a` about the unit axis `k`.
Vec3 turn(const Vec3& v, const Vec3& k, double a);
// a + t·(b − a)
double between(double a, double b, double t);
// sqrt(a² + b²), correctly rounded but for rare ties, by the method of CPython's math.hypot. The C
// library's hypot can be a unit in the last place off, and a helix one unit longer can decide a
// case.
double hypot(double a, double b);

} // namespace arithmetic
