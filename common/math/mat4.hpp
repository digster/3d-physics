// ===========================================================================
//  mat4.hpp — 4x4 matrices: translate, rotate, scale, look, and project
// ---------------------------------------------------------------------------
//  Introduced in: Chapter 3 (Matrices & Transforms) and Chapter 4 (Camera)
//
//  A 4x4 matrix is a compact recipe for "take a point and move it somewhere".
//  Because we work in homogeneous 4D coordinates (see vec4.hpp), a *single*
//  matrix type can express translation, rotation, scaling, and even the
//  perspective of a camera lens. Chaining transforms is then just multiplying
//  matrices, which is why every renderer speaks in 4x4s.
//
//  STORAGE — column-major.
//  We store the 16 numbers column by column, exactly like OpenGL:
//
//      e[0] e[4] e[ 8] e[12]        column c, row r lives at e[c*4 + r]
//      e[1] e[5] e[ 9] e[13]        so column 3 (e[12..15]) is the
//      e[2] e[6] e[10] e[14]        translation part of an affine transform.
//      e[3] e[7] e[11] e[15]
//
//  CONVENTION — a matrix acts on a *column* vector on its right: v' = M * v.
//  Consequently, in a product A * B, B is applied first and A second. Reading
//  a chain right-to-left ("model, then view, then projection") gives the order
//  a vertex actually experiences:  clip = Projection * View * Model * v.
// ===========================================================================
#pragma once

#include "scalar.hpp"
#include "vec3.hpp"
#include "vec4.hpp"
#include <cmath>

namespace p3d {

struct Mat4 {
    // Column-major storage. Default-initialised to all zeros.
    Real e[16]{};

    // Element access by (row, column). This hides the column-major index math
    // so the rest of the code can think in ordinary matrix terms.
    constexpr Real&       at(int r, int c)       { return e[c * 4 + r]; }
    constexpr const Real& at(int r, int c) const { return e[c * 4 + r]; }

    // The identity matrix: the transform that changes nothing (v' = v).
    static constexpr Mat4 identity() {
        Mat4 m;
        m.at(0, 0) = 1; m.at(1, 1) = 1; m.at(2, 2) = 1; m.at(3, 3) = 1;
        return m;
    }
};

// ---------------------------------------------------------------------------
//  Multiplication
// ---------------------------------------------------------------------------

// Matrix * column-vector. Reading the formula: each output row is the dot of
// that row of M with the vector. This is the fundamental "apply the transform"
// operation; every point that appears on screen goes through one of these.
inline constexpr Vec4 operator*(const Mat4& m, const Vec4& v) {
    Vec4 r;
    for (int row = 0; row < 4; ++row) {
        r[row] = m.at(row, 0) * v.x + m.at(row, 1) * v.y +
                 m.at(row, 2) * v.z + m.at(row, 3) * v.w;
    }
    return r;
}

// Matrix * matrix. Column c of the result is M applied to column c of N — i.e.
// composing the two transforms. O(64) multiplies; fine at our scale.
inline constexpr Mat4 operator*(const Mat4& m, const Mat4& n) {
    Mat4 r;
    for (int col = 0; col < 4; ++col) {
        for (int row = 0; row < 4; ++row) {
            Real sum = 0;
            for (int k = 0; k < 4; ++k) sum += m.at(row, k) * n.at(k, col);
            r.at(row, col) = sum;
        }
    }
    return r;
}

// ---------------------------------------------------------------------------
//  Applying a transform to a 3D point or direction
// ---------------------------------------------------------------------------

// Treat p as a POINT (implicit w = 1): rotation, scale AND translation apply.
// Returns the transformed 3D point, assuming an affine matrix (bottom row
// 0 0 0 1) so no perspective divide is needed.
inline constexpr Vec3 transformPoint(const Mat4& m, const Vec3& p) {
    return {
        m.at(0, 0) * p.x + m.at(0, 1) * p.y + m.at(0, 2) * p.z + m.at(0, 3),
        m.at(1, 0) * p.x + m.at(1, 1) * p.y + m.at(1, 2) * p.z + m.at(1, 3),
        m.at(2, 0) * p.x + m.at(2, 1) * p.y + m.at(2, 2) * p.z + m.at(2, 3),
    };
}

// Treat d as a DIRECTION (implicit w = 0): rotation and scale apply, but the
// translation column is ignored — a direction has no location to move.
inline constexpr Vec3 transformDir(const Mat4& m, const Vec3& d) {
    return {
        m.at(0, 0) * d.x + m.at(0, 1) * d.y + m.at(0, 2) * d.z,
        m.at(1, 0) * d.x + m.at(1, 1) * d.y + m.at(1, 2) * d.z,
        m.at(2, 0) * d.x + m.at(2, 1) * d.y + m.at(2, 2) * d.z,
    };
}

// ---------------------------------------------------------------------------
//  Building the standard affine transforms
// ---------------------------------------------------------------------------

// Move everything by t. Note the translation lives in the last column.
inline constexpr Mat4 translation(const Vec3& t) {
    Mat4 m = Mat4::identity();
    m.at(0, 3) = t.x; m.at(1, 3) = t.y; m.at(2, 3) = t.z;
    return m;
}

// Scale each axis independently (uniform scale: pass Vec3(s)).
inline constexpr Mat4 scaling(const Vec3& s) {
    Mat4 m;
    m.at(0, 0) = s.x; m.at(1, 1) = s.y; m.at(2, 2) = s.z; m.at(3, 3) = 1;
    return m;
}

// Rotations about the three principal axes, angle in RADIANS, counter-
// clockwise when looking down the axis toward the origin (right-hand rule).
inline Mat4 rotationX(Real angle) {
    const Real c = std::cos(angle), s = std::sin(angle);
    Mat4 m = Mat4::identity();
    m.at(1, 1) = c; m.at(1, 2) = -s;
    m.at(2, 1) = s; m.at(2, 2) =  c;
    return m;
}
inline Mat4 rotationY(Real angle) {
    const Real c = std::cos(angle), s = std::sin(angle);
    Mat4 m = Mat4::identity();
    m.at(0, 0) =  c; m.at(0, 2) = s;
    m.at(2, 0) = -s; m.at(2, 2) = c;
    return m;
}
inline Mat4 rotationZ(Real angle) {
    const Real c = std::cos(angle), s = std::sin(angle);
    Mat4 m = Mat4::identity();
    m.at(0, 0) = c; m.at(0, 1) = -s;
    m.at(1, 0) = s; m.at(1, 1) =  c;
    return m;
}

// Rotation by `angle` about an arbitrary unit axis (Rodrigues' rotation
// formula written as a matrix). This is the general case the three above are
// special instances of; quaternions (quat.hpp) are the more robust way to
// accumulate many of these, but the matrix form is the clearest to read.
inline Mat4 rotationAxisAngle(const Vec3& axisIn, Real angle) {
    const Vec3 a = normalize(axisIn);
    const Real c = std::cos(angle), s = std::sin(angle), t = Real(1) - c;
    const Real x = a.x, y = a.y, z = a.z;
    Mat4 m = Mat4::identity();
    m.at(0, 0) = t*x*x + c;   m.at(0, 1) = t*x*y - s*z; m.at(0, 2) = t*x*z + s*y;
    m.at(1, 0) = t*x*y + s*z; m.at(1, 1) = t*y*y + c;   m.at(1, 2) = t*y*z - s*x;
    m.at(2, 0) = t*x*z - s*y; m.at(2, 1) = t*y*z + s*x; m.at(2, 2) = t*z*z + c;
    return m;
}

// ---------------------------------------------------------------------------
//  Camera & projection (Chapter 4)
// ---------------------------------------------------------------------------

// A "look at" view matrix. It re-expresses the world in the camera's frame:
// the camera sits at `eye`, points toward `center`, with `up` giving roll.
// The camera looks down its own local -z (right-handed convention).
inline Mat4 lookAt(const Vec3& eye, const Vec3& center, const Vec3& up) {
    const Vec3 f = normalize(center - eye);   // forward (into the screen)
    const Vec3 s = normalize(cross(f, up));   // right
    const Vec3 u = cross(s, f);               // recomputed true up

    Mat4 m = Mat4::identity();
    // Rows are the camera's right / up / backward axes: this rotates world
    // space into camera space...
    m.at(0, 0) = s.x; m.at(0, 1) = s.y; m.at(0, 2) = s.z;
    m.at(1, 0) = u.x; m.at(1, 1) = u.y; m.at(1, 2) = u.z;
    m.at(2, 0) = -f.x; m.at(2, 1) = -f.y; m.at(2, 2) = -f.z;
    // ...and the last column translates so the camera ends up at the origin.
    m.at(0, 3) = -dot(s, eye);
    m.at(1, 3) = -dot(u, eye);
    m.at(2, 3) =  dot(f, eye);
    return m;
}

// Perspective projection: the lens. Points farther down -z get a larger `w`,
// so the later perspective divide shrinks them — that is what creates the
// sense of depth. Matches the OpenGL convention (clip z in [-near, far] maps
// to NDC [-1, 1]); our software renderer mainly cares about x, y and the w it
// produces for the divide.
//   fovY   : vertical field of view, in radians
//   aspect : viewport width / height
//   near/far: positive distances to the clip planes
inline Mat4 perspective(Real fovY, Real aspect, Real nearZ, Real farZ) {
    const Real f = Real(1) / std::tan(fovY * Real(0.5));  // cotangent of half-FOV
    Mat4 m;  // starts all-zero — most entries of a perspective matrix ARE zero
    m.at(0, 0) = f / aspect;
    m.at(1, 1) = f;
    m.at(2, 2) = (farZ + nearZ) / (nearZ - farZ);
    m.at(2, 3) = (Real(2) * farZ * nearZ) / (nearZ - farZ);
    m.at(3, 2) = -1;  // this -1 copies -z_view into w, driving the divide
    return m;
}

// ---------------------------------------------------------------------------
//  Utilities
// ---------------------------------------------------------------------------

// Transpose: mirror across the diagonal. For a pure rotation matrix the
// transpose is also its inverse (a useful, cheap fact).
inline constexpr Mat4 transpose(const Mat4& m) {
    Mat4 r;
    for (int c = 0; c < 4; ++c)
        for (int row = 0; row < 4; ++row)
            r.at(c, row) = m.at(row, c);
    return r;
}

// General 4x4 inverse via cofactors (adjugate / determinant). We rarely need
// the fully general case, but it is handy for tests and for undoing a compound
// transform. Returns the identity if the matrix is singular (det ~ 0).
inline Mat4 inverse(const Mat4& in) {
    const Real* m = in.e;  // column-major flat view
    Real inv[16];

    inv[0]  =  m[5]*m[10]*m[15] - m[5]*m[11]*m[14] - m[9]*m[6]*m[15] +
               m[9]*m[7]*m[14] + m[13]*m[6]*m[11] - m[13]*m[7]*m[10];
    inv[4]  = -m[4]*m[10]*m[15] + m[4]*m[11]*m[14] + m[8]*m[6]*m[15] -
               m[8]*m[7]*m[14] - m[12]*m[6]*m[11] + m[12]*m[7]*m[10];
    inv[8]  =  m[4]*m[9]*m[15] - m[4]*m[11]*m[13] - m[8]*m[5]*m[15] +
               m[8]*m[7]*m[13] + m[12]*m[5]*m[11] - m[12]*m[7]*m[9];
    inv[12] = -m[4]*m[9]*m[14] + m[4]*m[10]*m[13] + m[8]*m[5]*m[14] -
               m[8]*m[6]*m[13] - m[12]*m[5]*m[10] + m[12]*m[6]*m[9];
    inv[1]  = -m[1]*m[10]*m[15] + m[1]*m[11]*m[14] + m[9]*m[2]*m[15] -
               m[9]*m[3]*m[14] - m[13]*m[2]*m[11] + m[13]*m[3]*m[10];
    inv[5]  =  m[0]*m[10]*m[15] - m[0]*m[11]*m[14] - m[8]*m[2]*m[15] +
               m[8]*m[3]*m[14] + m[12]*m[2]*m[11] - m[12]*m[3]*m[10];
    inv[9]  = -m[0]*m[9]*m[15] + m[0]*m[11]*m[13] + m[8]*m[1]*m[15] -
               m[8]*m[3]*m[13] - m[12]*m[1]*m[11] + m[12]*m[3]*m[9];
    inv[13] =  m[0]*m[9]*m[14] - m[0]*m[10]*m[13] - m[8]*m[1]*m[14] +
               m[8]*m[2]*m[13] + m[12]*m[1]*m[10] - m[12]*m[2]*m[9];
    inv[2]  =  m[1]*m[6]*m[15] - m[1]*m[7]*m[14] - m[5]*m[2]*m[15] +
               m[5]*m[3]*m[14] + m[13]*m[2]*m[7] - m[13]*m[3]*m[6];
    inv[6]  = -m[0]*m[6]*m[15] + m[0]*m[7]*m[14] + m[4]*m[2]*m[15] -
               m[4]*m[3]*m[14] - m[12]*m[2]*m[7] + m[12]*m[3]*m[6];
    inv[10] =  m[0]*m[5]*m[15] - m[0]*m[7]*m[13] - m[4]*m[1]*m[15] +
               m[4]*m[3]*m[13] + m[12]*m[1]*m[7] - m[12]*m[3]*m[5];
    inv[14] = -m[0]*m[5]*m[14] + m[0]*m[6]*m[13] + m[4]*m[1]*m[14] -
               m[4]*m[2]*m[13] - m[12]*m[1]*m[6] + m[12]*m[2]*m[5];
    inv[3]  = -m[1]*m[6]*m[11] + m[1]*m[7]*m[10] + m[5]*m[2]*m[11] -
               m[5]*m[3]*m[10] - m[9]*m[2]*m[7] + m[9]*m[3]*m[6];
    inv[7]  =  m[0]*m[6]*m[11] - m[0]*m[7]*m[10] - m[4]*m[2]*m[11] +
               m[4]*m[3]*m[10] + m[8]*m[2]*m[7] - m[8]*m[3]*m[6];
    inv[11] = -m[0]*m[5]*m[11] + m[0]*m[7]*m[9] + m[4]*m[1]*m[11] -
               m[4]*m[3]*m[9] - m[8]*m[1]*m[7] + m[8]*m[3]*m[5];
    inv[15] =  m[0]*m[5]*m[10] - m[0]*m[6]*m[9] - m[4]*m[1]*m[10] +
               m[4]*m[2]*m[9] + m[8]*m[1]*m[6] - m[8]*m[2]*m[5];

    Real det = m[0]*inv[0] + m[1]*inv[4] + m[2]*inv[8] + m[3]*inv[12];
    if (std::fabs(det) < kEpsilon) return Mat4::identity();

    const Real invDet = Real(1) / det;
    Mat4 out;
    for (int i = 0; i < 16; ++i) out.e[i] = inv[i] * invDet;
    return out;
}

}  // namespace p3d
