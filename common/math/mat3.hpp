// ===========================================================================
//  mat3.hpp — 3x3 matrices, mainly for inertia tensors and rotation
// ---------------------------------------------------------------------------
//  Introduced in: Chapter 11 (Rigid-Body Dynamics)
//
//  A 4x4 matrix (mat4.hpp) is the right tool for positioning things, because it
//  folds in translation. But rotation and the INERTIA TENSOR — the 3x3 object
//  that describes how mass is spread around a body — have no translation part,
//  so a 3x3 matrix is the natural, honest size for them.
//
//  Storage matches mat4: COLUMN-MAJOR, element (row r, col c) at e[c*3 + r], and
//  a matrix acts on a column vector on its right (v' = M v).
// ===========================================================================
#pragma once

#include "scalar.hpp"
#include "vec3.hpp"

namespace p3d {

struct Mat3 {
    Real e[9]{};   // column-major, all zeros by default

    constexpr Real&       at(int r, int c)       { return e[c * 3 + r]; }
    constexpr const Real& at(int r, int c) const { return e[c * 3 + r]; }

    static constexpr Mat3 identity() {
        Mat3 m; m.at(0,0) = 1; m.at(1,1) = 1; m.at(2,2) = 1; return m;
    }
    // A diagonal matrix — the shape an inertia tensor takes when the body's own
    // axes are its "principal axes" (which is how we set up our primitives).
    static constexpr Mat3 diagonal(const Vec3& d) {
        Mat3 m; m.at(0,0) = d.x; m.at(1,1) = d.y; m.at(2,2) = d.z; return m;
    }
};

// Matrix × vector: the workhorse. For inertia, this turns an angular velocity
// into an angular momentum (L = I ω) or vice versa.
inline constexpr Vec3 operator*(const Mat3& m, const Vec3& v) {
    return {
        m.at(0,0)*v.x + m.at(0,1)*v.y + m.at(0,2)*v.z,
        m.at(1,0)*v.x + m.at(1,1)*v.y + m.at(1,2)*v.z,
        m.at(2,0)*v.x + m.at(2,1)*v.y + m.at(2,2)*v.z,
    };
}

// Matrix × matrix (compose). Used to rotate an inertia tensor into world space.
inline constexpr Mat3 operator*(const Mat3& a, const Mat3& b) {
    Mat3 r;
    for (int c = 0; c < 3; ++c)
        for (int row = 0; row < 3; ++row) {
            Real s = 0;
            for (int k = 0; k < 3; ++k) s += a.at(row,k) * b.at(k,c);
            r.at(row,c) = s;
        }
    return r;
}

inline constexpr Mat3 operator*(const Mat3& m, Real s) {
    Mat3 r; for (int i = 0; i < 9; ++i) r.e[i] = m.e[i] * s; return r;
}

// Element-wise addition — used to sum the inertia tensors of a body's parts
// into one composite tensor (parallel-axis theorem, see inertia.hpp).
inline constexpr Mat3 operator+(const Mat3& a, const Mat3& b) {
    Mat3 r; for (int i = 0; i < 9; ++i) r.e[i] = a.e[i] + b.e[i]; return r;
}
inline constexpr Mat3 operator-(const Mat3& a, const Mat3& b) {
    Mat3 r; for (int i = 0; i < 9; ++i) r.e[i] = a.e[i] - b.e[i]; return r;
}

// The "skew-symmetric" (cross-product) matrix of a vector: the matrix S such
// that S·w = v × w for any w. It turns the cross product into a matrix multiply,
// which is what lets us build a joint's 3×3 effective-mass matrix (joint.cpp).
inline constexpr Mat3 skew(const Vec3& v) {
    Mat3 m;
    m.at(0,1) = -v.z; m.at(0,2) =  v.y;
    m.at(1,0) =  v.z; m.at(1,2) = -v.x;
    m.at(2,0) = -v.y; m.at(2,1) =  v.x;
    return m;
}

// Transpose. For a pure rotation R, the transpose is the inverse — which is why
// rotating an inertia tensor into world space reads I_world = R · I · Rᵀ.
inline constexpr Mat3 transpose(const Mat3& m) {
    Mat3 r;
    for (int c = 0; c < 3; ++c)
        for (int row = 0; row < 3; ++row)
            r.at(c,row) = m.at(row,c);
    return r;
}

// General 3x3 inverse via cofactors / determinant. We need it to turn an
// inertia tensor I into its inverse I⁻¹ (used constantly: ω = I⁻¹ L). Returns
// the identity if the matrix is singular.
inline Mat3 inverse(const Mat3& m) {
    const Real a = m.at(0,0), b = m.at(0,1), c = m.at(0,2);
    const Real d = m.at(1,0), e = m.at(1,1), f = m.at(1,2);
    const Real g = m.at(2,0), h = m.at(2,1), i = m.at(2,2);

    const Real A =  (e*i - f*h);   // cofactors
    const Real B = -(d*i - f*g);
    const Real C =  (d*h - e*g);
    const Real det = a*A + b*B + c*C;
    if (std::fabs(det) < kEpsilon) return Mat3::identity();
    const Real inv = Real(1) / det;

    Mat3 r;
    r.at(0,0) = A * inv;             r.at(0,1) = -(b*i - c*h) * inv; r.at(0,2) =  (b*f - c*e) * inv;
    r.at(1,0) = B * inv;             r.at(1,1) =  (a*i - c*g) * inv; r.at(1,2) = -(a*f - c*d) * inv;
    r.at(2,0) = C * inv;             r.at(2,1) = -(a*h - b*g) * inv; r.at(2,2) =  (a*e - b*d) * inv;
    return r;
}

}  // namespace p3d
