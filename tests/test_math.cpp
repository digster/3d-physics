// ===========================================================================
//  test_math.cpp — properties the vector/matrix/quaternion code must satisfy
// ---------------------------------------------------------------------------
//  These are the identities the whole engine relies on. If one of these breaks,
//  something far downstream (a wobbling camera, a body that gains energy) will
//  break mysteriously — so we pin them here.
// ===========================================================================
#include <doctest/doctest.h>
#include "common/math/math.hpp"

using namespace p3d;

// Small helpers for approximate comparison of vectors/quaternions.
static bool close(Real a, Real b, Real eps = 1e-4f) { return std::fabs(a - b) <= eps; }
static bool close(const Vec3& a, const Vec3& b, Real eps = 1e-4f) {
    return close(a.x, b.x, eps) && close(a.y, b.y, eps) && close(a.z, b.z, eps);
}

TEST_CASE("Vec3 dot product") {
    CHECK(close(dot(Vec3{1, 0, 0}, Vec3{0, 1, 0}), 0.0f));        // perpendicular
    CHECK(close(dot(Vec3{1, 2, 3}, Vec3{4, 5, 6}), 32.0f));       // 4+10+18
    CHECK(dot(Vec3{1, 0, 0}, Vec3{-1, 0, 0}) < 0);                // opposite → negative
}

TEST_CASE("Vec3 cross product follows the right-hand rule") {
    CHECK(close(cross(axis::X, axis::Y), axis::Z));   // x cross y = z
    CHECK(close(cross(axis::Y, axis::Z), axis::X));   // y cross z = x
    CHECK(close(cross(axis::Z, axis::X), axis::Y));   // z cross x = y
    // Anticommutative: a×b = -(b×a).
    CHECK(close(cross(axis::X, axis::Y), -cross(axis::Y, axis::X)));
    // A vector crossed with itself is zero.
    CHECK(close(cross(Vec3{2, 3, 4}, Vec3{2, 3, 4}), Vec3{}));
}

TEST_CASE("Vec3 length and normalize") {
    CHECK(close(length(Vec3{3, 4, 0}), 5.0f));               // 3-4-5 triangle
    CHECK(close(length(normalize(Vec3{7, -2, 5})), 1.0f));   // unit length
    CHECK(close(normalize(Vec3{}), Vec3{}));                 // zero stays zero (no NaN)
}

TEST_CASE("Vec3 reflect mirrors across a normal") {
    // A downward velocity hitting a floor (normal up) bounces upward.
    CHECK(close(reflect(Vec3{1, -1, 0}, axis::Y), Vec3{1, 1, 0}));
}

TEST_CASE("Mat4 identity is a no-op") {
    const Mat4 I = Mat4::identity();
    const Vec3 p{2, -3, 5};
    CHECK(close(transformPoint(I, p), p));
}

TEST_CASE("Mat4 translation moves points but not directions") {
    const Mat4 T = translation(Vec3{10, 0, 0});
    CHECK(close(transformPoint(T, Vec3{1, 2, 3}), Vec3{11, 2, 3}));  // point moves
    CHECK(close(transformDir(T, Vec3{1, 0, 0}), Vec3{1, 0, 0}));     // direction unaffected
}

TEST_CASE("Mat4 rotationZ(90 deg) sends +X to +Y") {
    const Mat4 R = rotationZ(radians(90));
    CHECK(close(transformPoint(R, axis::X), axis::Y));
}

TEST_CASE("Mat4 rotationY(90 deg) sends +X to -Z") {
    const Mat4 R = rotationY(radians(90));
    CHECK(close(transformPoint(R, axis::X), Vec3{0, 0, -1}));
}

TEST_CASE("Mat4 composition applies right-to-left") {
    // Rotate 90 deg about Z, THEN translate +X by 5.
    const Mat4 M = translation(Vec3{5, 0, 0}) * rotationZ(radians(90));
    // X first rotates to Y, then shifts by +5 in x.
    CHECK(close(transformPoint(M, axis::X), Vec3{5, 1, 0}));
}

TEST_CASE("Mat4 inverse undoes a transform") {
    const Mat4 M = translation(Vec3{3, -2, 7}) * rotationY(radians(37)) *
                   rotationX(radians(-15)) * scaling(Vec3{2, 2, 2});
    const Mat4 Minv = inverse(M);
    const Mat4 I = M * Minv;
    // M * M^-1 should be the identity.
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            CHECK(close(I.at(r, c), (r == c) ? 1.0f : 0.0f, 1e-3f));
}

TEST_CASE("Quat identity does not rotate") {
    CHECK(close(rotate(Quat::identity(), Vec3{1, 2, 3}), Vec3{1, 2, 3}));
}

TEST_CASE("Quat rotation matches the equivalent matrix") {
    const Vec3 axisv = normalize(Vec3{0.3f, 1.0f, -0.5f});
    const Real angle = radians(50);
    const Quat q = Quat::fromAxisAngle(axisv, angle);
    const Mat4 M = rotationAxisAngle(axisv, angle);

    const Vec3 p{1.2f, -0.7f, 0.9f};
    // Rotating by the quaternion and by the matrix must agree.
    CHECK(close(rotate(q, p), transformPoint(M, p), 1e-3f));
    // ...and the quaternion-derived matrix must agree too.
    CHECK(close(transformPoint(toMat4(q), p), rotate(q, p), 1e-3f));
}

TEST_CASE("Quat multiplication composes rotations") {
    // Two 45-deg turns about Y should equal one 90-deg turn about Y.
    const Quat a = Quat::fromAxisAngle(axis::Y, radians(45));
    const Quat combined = a * a;
    const Quat once = Quat::fromAxisAngle(axis::Y, radians(90));
    const Vec3 p{1, 0, 0};
    CHECK(close(rotate(combined, p), rotate(once, p), 1e-3f));
}

TEST_CASE("Quat stays unit-length through integration") {
    Quat q = Quat::identity();
    const Vec3 omega{0.0f, 3.0f, 0.0f};   // spin about Y
    for (int i = 0; i < 1000; ++i) q = integrate(q, omega, 1.0f / 120.0f);
    CHECK(close(length(q), 1.0f, 1e-3f));   // normalisation keeps it valid
}
