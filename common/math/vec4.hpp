// ===========================================================================
//  vec4.hpp — 4D vectors, the vehicle for homogeneous coordinates
// ---------------------------------------------------------------------------
//  Introduced in: Chapter 4 (Camera & Projection)
//
//  Why does a 3D engine need a 4-component vector? Because a 4x4 matrix can
//  only multiply a 4-component vector, and we deliberately use 4x4 matrices so
//  that *translation* and *perspective* — which are not linear in 3D — become
//  ordinary matrix multiplies in 4D. The fourth coordinate `w` is the trick:
//
//    * A POINT in space is written (x, y, z, 1). The trailing 1 is what lets a
//      matrix add a translation to it.
//    * A DIRECTION is written (x, y, z, 0). The 0 makes translation a no-op —
//      you can move a point, but "north" is still north wherever you stand.
//    * After a perspective matrix, w holds the camera-space depth. Dividing
//      x, y, z by w is what makes far things small (the "perspective divide").
//
//  Chapter 4 walks through all of this; here we just provide the storage and
//  the few operations the projection code needs.
// ===========================================================================
#pragma once

#include "scalar.hpp"
#include "vec3.hpp"

namespace p3d {

struct Vec4 {
    Real x{}, y{}, z{}, w{};

    constexpr Vec4() = default;
    constexpr Vec4(Real x_, Real y_, Real z_, Real w_) : x(x_), y(y_), z(z_), w(w_) {}

    // Promote a Vec3 to homogeneous form. `w = 1` → a position; `w = 0` → a
    // direction. Being explicit about this at the call site prevents the
    // classic bug of translating a direction vector.
    constexpr Vec4(const Vec3& v, Real w_) : x(v.x), y(v.y), z(v.z), w(w_) {}

    constexpr Real&       operator[](int i)       { return (&x)[i]; }
    constexpr const Real& operator[](int i) const { return (&x)[i]; }

    // Drop back to 3D by discarding w (used once we no longer need it).
    constexpr Vec3 xyz() const { return {x, y, z}; }
};

inline constexpr Vec4 operator+(const Vec4& a, const Vec4& b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w};
}
inline constexpr Vec4 operator*(const Vec4& v, Real s) {
    return {v.x * s, v.y * s, v.z * s, v.w * s};
}

// The perspective divide: (x, y, z, w) → (x/w, y/w, z/w). This is the single
// step that turns a parallel-looking scene into one with real depth. Guarded
// so a w of zero (a point exactly on the camera plane) does not produce NaNs.
inline Vec3 perspectiveDivide(const Vec4& v) {
    if (std::fabs(v.w) < kEpsilon) return v.xyz();
    const Real inv = Real(1) / v.w;
    return {v.x * inv, v.y * inv, v.z * inv};
}

}  // namespace p3d
