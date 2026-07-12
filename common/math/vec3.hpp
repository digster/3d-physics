// ===========================================================================
//  vec3.hpp — the 3D vector, the atom of everything that follows
// ---------------------------------------------------------------------------
//  Introduced in: Chapter 2 (Vectors)
//
//  A Vec3 is three numbers (x, y, z). What makes it a *vector* rather than
//  three loose floats is the operations we give it: adding two of them,
//  scaling one, measuring its length, and — the two that unlock 3D — the dot
//  and cross products. The docs (docs/chapters/02-vectors.html) build the
//  geometric intuition; this file is the reference implementation.
//
//  Design notes:
//    * Everything is `constexpr` and `inline` so the compiler can fold vector
//      math down to plain arithmetic — a Vec3 has zero overhead over three
//      separate floats.
//    * Operators are provided for the readable cases (a + b, a * s). Named
//      functions (dot, cross, normalize) are used where a symbol would be
//      ambiguous — there is no single obvious meaning for `a * b` on vectors.
// ===========================================================================
#pragma once

#include "scalar.hpp"
#include <cmath>

namespace p3d {

struct Vec3 {
    Real x{}, y{}, z{};   // value-initialised to (0,0,0) by default

    // --- Constructors ------------------------------------------------------
    constexpr Vec3() = default;
    constexpr Vec3(Real x_, Real y_, Real z_) : x(x_), y(y_), z(z_) {}
    // A single value broadcasts to all three lanes: Vec3(0) is the origin.
    explicit constexpr Vec3(Real s) : x(s), y(s), z(s) {}

    // --- Element access ----------------------------------------------------
    // Occasionally it is convenient to treat a vector as an array (e.g. when
    // indexing an axis 0/1/2). These give that without exposing raw memory.
    constexpr Real&       operator[](int i)       { return (&x)[i]; }
    constexpr const Real& operator[](int i) const { return (&x)[i]; }
};

// --- Unary / arithmetic operators ------------------------------------------
// Negation flips the arrow to point the opposite way.
inline constexpr Vec3 operator-(const Vec3& v) { return {-v.x, -v.y, -v.z}; }

// Vector addition is tip-to-tail: walk along a, then along b.
inline constexpr Vec3 operator+(const Vec3& a, const Vec3& b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}
// Subtraction b - a produces the vector that points *from a to b* (the
// displacement). This is one of the most-used facts in the whole engine.
inline constexpr Vec3 operator-(const Vec3& a, const Vec3& b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

// Scaling by a scalar stretches (|s|>1) or shrinks (|s|<1) the arrow; a
// negative scalar also flips it. Provided in both orders (v*s and s*v).
inline constexpr Vec3 operator*(const Vec3& v, Real s) { return {v.x * s, v.y * s, v.z * s}; }
inline constexpr Vec3 operator*(Real s, const Vec3& v) { return v * s; }
inline constexpr Vec3 operator/(const Vec3& v, Real s) { return v * (Real(1) / s); }

// Component-wise (Hadamard) product — NOT a geometric operation, but handy for
// things like scaling per-axis (e.g. non-uniform box half-extents) and tinting.
inline constexpr Vec3 mul(const Vec3& a, const Vec3& b) {
    return {a.x * b.x, a.y * b.y, a.z * b.z};
}

// --- Compound assignment ---------------------------------------------------
inline constexpr Vec3& operator+=(Vec3& a, const Vec3& b) { a = a + b; return a; }
inline constexpr Vec3& operator-=(Vec3& a, const Vec3& b) { a = a - b; return a; }
inline constexpr Vec3& operator*=(Vec3& a, Real s)        { a = a * s; return a; }
inline constexpr Vec3& operator/=(Vec3& a, Real s)        { a = a / s; return a; }

// --- The dot product -------------------------------------------------------
//  dot(a,b) = ax*bx + ay*by + az*bz  =  |a| |b| cos(theta)
//  Intuition: "how much do these two arrows point the same way?" It is
//  positive when the angle between them is acute, zero when perpendicular,
//  negative when obtuse. We use it for projections, for testing which side of
//  a plane a point is on, and for turning a relative velocity into a speed
//  along a contact normal.
inline constexpr Real dot(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

// --- The cross product -----------------------------------------------------
//  cross(a,b) is a vector PERPENDICULAR to both a and b, whose length equals
//  the area of the parallelogram they span (|a||b|sin theta). Its direction
//  follows the right-hand rule: point fingers along a, curl toward b, thumb is
//  the result. We use it to build face normals, to get a rotation axis from
//  two directions, and (as r x F) to turn a force at a point into a torque.
inline constexpr Vec3 cross(const Vec3& a, const Vec3& b) {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    };
}

// --- Length & normalization ------------------------------------------------
// lengthSq avoids the square root. Prefer it whenever you only need to compare
// magnitudes (e.g. "is this within radius r?" → compare against r*r), because
// sqrt is comparatively expensive and, more importantly, monotonic — the
// comparison result is identical.
inline constexpr Real lengthSq(const Vec3& v) { return dot(v, v); }
inline Real           length(const Vec3& v)   { return std::sqrt(lengthSq(v)); }

// Return a unit-length (length 1) copy pointing the same direction. Guards
// against dividing by zero: a zero vector has no direction, so we return it
// unchanged rather than producing NaNs that would poison the simulation.
inline Vec3 normalize(const Vec3& v) {
    const Real len2 = lengthSq(v);
    if (len2 < kEpsilon * kEpsilon) return v;
    return v * (Real(1) / std::sqrt(len2));
}

// Distance between two points is just the length of the displacement between
// them. Squared variant provided for the same reason as lengthSq.
inline Real           distance(const Vec3& a, const Vec3& b)   { return length(a - b); }
inline constexpr Real distanceSq(const Vec3& a, const Vec3& b) { return lengthSq(a - b); }

// --- Handy geometric helpers ----------------------------------------------
// Component-wise min/max — the building blocks of axis-aligned bounding boxes
// (Chapter 12). minv/maxv keep the per-axis smallest/largest coordinate.
inline constexpr Vec3 minv(const Vec3& a, const Vec3& b) {
    return {a.x < b.x ? a.x : b.x, a.y < b.y ? a.y : b.y, a.z < b.z ? a.z : b.z};
}
inline constexpr Vec3 maxv(const Vec3& a, const Vec3& b) {
    return {a.x > b.x ? a.x : b.x, a.y > b.y ? a.y : b.y, a.z > b.z ? a.z : b.z};
}

// Linear interpolation between two points/vectors, component-wise. Used for
// render interpolation between two physics states and for blending colors.
inline constexpr Vec3 lerp(const Vec3& a, const Vec3& b, Real t) {
    return a + (b - a) * t;
}

// Reflect a vector about a surface with unit normal n. This is the mirror
// bounce of geometric optics; the collision code reuses the same identity to
// split a velocity into its along-normal and along-surface parts.
//   r = v - 2 (v . n) n
inline constexpr Vec3 reflect(const Vec3& v, const Vec3& n) {
    return v - Real(2) * dot(v, n) * n;
}

// Project vector a onto the direction of b (the "shadow" of a along b).
inline Vec3 project(const Vec3& a, const Vec3& b) {
    const Real len2 = lengthSq(b);
    if (len2 < kEpsilon * kEpsilon) return Vec3{};
    return b * (dot(a, b) / len2);
}

// --- Named axes / common vectors ------------------------------------------
// Small conveniences that make scene setup read like prose.
namespace axis {
inline constexpr Vec3 X{1, 0, 0};
inline constexpr Vec3 Y{0, 1, 0};   // "up" in our right-handed world
inline constexpr Vec3 Z{0, 0, 1};
}  // namespace axis

}  // namespace p3d
