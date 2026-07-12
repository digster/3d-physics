// ===========================================================================
//  quat.hpp — quaternions: rotation that never jams
// ---------------------------------------------------------------------------
//  Introduced in: Chapter 10 (Rotation & Quaternions)
//
//  A quaternion is four numbers, (w, x, y, z), that encode a 3D rotation. Why
//  not just use three Euler angles or a 3x3 matrix?
//    * Euler angles suffer from GIMBAL LOCK — at certain orientations two of
//      the three axes line up and you lose a degree of freedom. Chapter 10
//      shows this happening.
//    * Matrices drift: multiply thousands of them together and rounding error
//      turns a rotation into a slight shear/scale. Re-normalising a quaternion
//      (just rescale 4 numbers to unit length) is far cheaper and cleaner.
//    * Quaternions interpolate smoothly (slerp), which matrices cannot do
//      directly.
//
//  Intuition: a UNIT quaternion stores a rotation of angle θ about a unit axis
//  a as  q = (cos(θ/2), a * sin(θ/2)). The half-angle looks odd at first; it
//  falls out of how the sandwich product q v q⁻¹ rotates a vector (derived in
//  the docs). You rarely build one by hand — use fromAxisAngle().
//
//  STORAGE: w is the scalar part (first), matching the notation
//  q = w + x i + y j + z k used throughout the documentation.
// ===========================================================================
#pragma once

#include "scalar.hpp"
#include "vec3.hpp"
#include "mat4.hpp"
#include <cmath>

namespace p3d {

struct Quat {
    Real w{1}, x{0}, y{0}, z{0};   // identity rotation by default

    constexpr Quat() = default;
    constexpr Quat(Real w_, Real x_, Real y_, Real z_) : w(w_), x(x_), y(y_), z(z_) {}

    // The identity quaternion represents "no rotation".
    static constexpr Quat identity() { return {1, 0, 0, 0}; }

    // Build a rotation of `angle` radians about `axisIn` (need not be unit).
    static Quat fromAxisAngle(const Vec3& axisIn, Real angle) {
        const Vec3 a = normalize(axisIn);
        const Real half = angle * Real(0.5);
        const Real s = std::sin(half);
        return {std::cos(half), a.x * s, a.y * s, a.z * s};
    }
};

// The vector (imaginary) part, as a Vec3 — convenient in several formulas.
inline constexpr Vec3 vec(const Quat& q) { return {q.x, q.y, q.z}; }

// --- Length & normalization ------------------------------------------------
// Only UNIT quaternions represent pure rotations, so we renormalise often
// (especially after numerical integration, which nudges the length off 1).
inline Real lengthSq(const Quat& q) { return q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z; }
inline Real length(const Quat& q)   { return std::sqrt(lengthSq(q)); }

inline Quat normalize(const Quat& q) {
    const Real len2 = lengthSq(q);
    if (len2 < kEpsilon * kEpsilon) return Quat::identity();
    const Real inv = Real(1) / std::sqrt(len2);
    return {q.w * inv, q.x * inv, q.y * inv, q.z * inv};
}

// Conjugate negates the vector part. For a UNIT quaternion the conjugate is
// also the inverse — i.e. the opposite rotation.
inline constexpr Quat conjugate(const Quat& q) { return {q.w, -q.x, -q.y, -q.z}; }

// --- The Hamilton product --------------------------------------------------
// Multiplying quaternions COMPOSES their rotations. Like matrices, order
// matters and is read right-to-left: (a * b) means "do b, then a". This is the
// non-commutative heart of 3D rotation. The formula looks busy but is just the
// distributive expansion of (aw + a·ijk)(bw + b·ijk) using i²=j²=k²=ijk=-1.
inline constexpr Quat operator*(const Quat& a, const Quat& b) {
    return {
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w,
    };
}

// Scalar scaling and addition — not rotations themselves, but needed to
// integrate angular velocity (below) and to interpolate.
inline constexpr Quat operator*(const Quat& q, Real s) { return {q.w*s, q.x*s, q.y*s, q.z*s}; }
inline constexpr Quat operator+(const Quat& a, const Quat& b) {
    return {a.w+b.w, a.x+b.x, a.y+b.y, a.z+b.z};
}

// --- Rotating a vector -----------------------------------------------------
// Apply the rotation to v via the sandwich product v' = q v q⁻¹. We use the
// well-known optimised form (two cross products) rather than building the full
// product, but the meaning is identical. Assumes q is unit-length.
inline Vec3 rotate(const Quat& q, const Vec3& v) {
    const Vec3 u = vec(q);                       // vector part
    const Vec3 t = Real(2) * cross(u, v);
    return v + q.w * t + cross(u, t);
}

// --- Conversion to a matrix ------------------------------------------------
// The renderer speaks in matrices, so a rigid body converts its orientation
// quaternion to a rotation Mat4 once per frame. Assumes q is unit-length.
inline Mat4 toMat4(const Quat& q) {
    const Real xx = q.x*q.x, yy = q.y*q.y, zz = q.z*q.z;
    const Real xy = q.x*q.y, xz = q.x*q.z, yz = q.y*q.z;
    const Real wx = q.w*q.x, wy = q.w*q.y, wz = q.w*q.z;

    Mat4 m = Mat4::identity();
    m.at(0, 0) = 1 - 2*(yy + zz); m.at(0, 1) = 2*(xy - wz);     m.at(0, 2) = 2*(xz + wy);
    m.at(1, 0) = 2*(xy + wz);     m.at(1, 1) = 1 - 2*(xx + zz); m.at(1, 2) = 2*(yz - wx);
    m.at(2, 0) = 2*(xz - wy);     m.at(2, 1) = 2*(yz + wx);     m.at(2, 2) = 1 - 2*(xx + yy);
    return m;
}

// --- Integrating angular velocity ------------------------------------------
// Given the current orientation q and an angular velocity ω (rad/s, direction
// = spin axis, magnitude = spin rate), advance the orientation by dt.
//
// The exact rule is  dq/dt = ½ · (0, ω) · q. We take one explicit Euler step of
// it and renormalise. This is the rotational analogue of "position += velocity
// * dt" and is where a tumbling rigid body actually turns (Chapters 10–11).
inline Quat integrate(const Quat& q, const Vec3& omega, Real dt) {
    const Quat wq{0, omega.x, omega.y, omega.z};   // pure-vector quaternion
    const Quat dq = (wq * q) * (Real(0.5) * dt);   // ½ ω q dt
    return normalize(q + dq);
}

// --- Spherical linear interpolation (slerp) --------------------------------
// Blend between two orientations along the shortest arc at constant angular
// speed — the "right" way to interpolate rotations (used for smooth camera
// moves and render interpolation of orientation). t in [0,1].
inline Quat slerp(Quat a, const Quat& b, Real t) {
    Real cosTheta = a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z;
    // q and -q are the same rotation; flip a so we take the short way around.
    if (cosTheta < 0) { a = a * Real(-1); cosTheta = -cosTheta; }

    // Very close together → fall back to straight lerp to avoid dividing by
    // sin(θ) ≈ 0.
    if (cosTheta > Real(0.9995)) {
        return normalize(a + (b + a * Real(-1)) * t);
    }
    const Real theta   = std::acos(cosTheta);
    const Real sinT    = std::sin(theta);
    const Real wa      = std::sin((Real(1) - t) * theta) / sinT;
    const Real wb      = std::sin(t * theta) / sinT;
    return normalize(a * wa + b * wb);
}

}  // namespace p3d
