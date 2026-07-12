// ===========================================================================
//  test_rigidbody.cpp — Part III: Mat3, inertia tensors, and rigid bodies
// ---------------------------------------------------------------------------
//  The headline test is the Dzhanibekov effect: spinning about the intermediate
//  axis must actually flip, while spinning about the extreme axes must stay
//  stable. If the rotational dynamics are wrong, that test fails.
// ===========================================================================
#include <doctest/doctest.h>
#include "common/math/math.hpp"
#include "physics/rigidbody.hpp"
#include "physics/inertia.hpp"

using namespace p3d;

static bool close(Real a, Real b, Real eps = 1e-4f) { return std::fabs(a - b) <= eps; }
static bool close(const Vec3& a, const Vec3& b, Real eps = 1e-4f) {
    return close(a.x, b.x, eps) && close(a.y, b.y, eps) && close(a.z, b.z, eps);
}

TEST_CASE("Mat3 inverse undoes a matrix") {
    Mat3 m;
    m.at(0,0)=2; m.at(0,1)=1; m.at(0,2)=0;
    m.at(1,0)=1; m.at(1,1)=3; m.at(1,2)=1;
    m.at(2,0)=0; m.at(2,1)=1; m.at(2,2)=4;
    const Mat3 I = m * inverse(m);
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            CHECK(close(I.at(r,c), (r == c) ? 1.0f : 0.0f, 1e-4f));
}

TEST_CASE("Mat3 times a vector, and transpose") {
    const Mat3 d = Mat3::diagonal({2, 3, 4});
    CHECK(close(d * Vec3{1, 1, 1}, Vec3{2, 3, 4}));
    // Transpose of a diagonal is itself; of a general matrix swaps off-diagonals.
    Mat3 m; m.at(0,1) = 5;
    CHECK(close(transpose(m).at(1,0), 5.0f));
}

TEST_CASE("Inertia tensors match the textbook formulas") {
    // A unit cube (half-extent 0.5, mass 1): all three moments equal m·s²/6 with
    // s = 1 ⇒ 1/6 ≈ 0.1667.
    const Mat3 cube = inertia::box(1.0f, {0.5f, 0.5f, 0.5f});
    CHECK(close(cube.at(0,0), 1.0f / 6.0f, 1e-4f));
    CHECK(close(cube.at(0,0), cube.at(1,1)));
    CHECK(close(cube.at(1,1), cube.at(2,2)));

    // Solid sphere: 2/5 m r².
    const Mat3 s = inertia::sphere(2.0f, 3.0f);
    CHECK(close(s.at(0,0), 0.4f * 2.0f * 9.0f, 1e-3f));
}

TEST_CASE("A quaternion and its Mat3 agree with rotate()") {
    const Quat q = Quat::fromAxisAngle(normalize(Vec3{0.2f, 1, -0.4f}), radians(63));
    const Vec3 p{1.3f, -0.5f, 0.8f};
    CHECK(close(toMat3(q) * p, rotate(q, p), 1e-3f));
}

TEST_CASE("A force off-centre produces torque r x F") {
    RigidBody b;
    b.position = {0, 0, 0};
    b.setInertia(inertia::box(1.0f, {1, 1, 1}));
    // Push +x at a point 1 above the centre: torque = r × F = (0,1,0)×(F,0,0).
    b.applyForceAtPoint({5, 0, 0}, {0, 1, 0});
    CHECK(close(b.forceAccum, Vec3{5, 0, 0}));
    CHECK(close(b.torqueAccum, cross(Vec3{0, 1, 0}, Vec3{5, 0, 0})));  // = (0,0,-5)
    CHECK(close(b.torqueAccum, Vec3{0, 0, -5}));
}

TEST_CASE("Angular momentum is conserved under zero torque") {
    RigidBody b;
    b.setInertia(inertia::box(1.0f, {1, 0.5f, 0.2f}));
    b.setAngularVelocity({1.0f, 4.0f, 0.5f});
    const Vec3 L0 = b.angularMomentum;
    for (int i = 0; i < 2000; ++i) b.integrate(1.0f / 240.0f);
    // With no torque, L must not change at all (we integrate L directly).
    CHECK(close(b.angularMomentum, L0, 1e-5f));
}

TEST_CASE("Dzhanibekov: the intermediate axis flips, the others don't") {
    // Asymmetric box → three distinct moments. For {1,0.5,0.2}:
    //   Ixx (about x) is smallest, Izz largest, Iyy the INTERMEDIATE one.
    // The tell-tale of a flip is that the SPIN AXIS itself (a body-fixed
    // direction) reverses in world space; a stable spin keeps its axis fixed.
    auto makeBox = []() {
        RigidBody b; b.setMass(1.0f);
        b.setInertia(inertia::box(1.0f, {1.0f, 0.5f, 0.2f}));
        return b;
    };
    const Real dt = 1.0f / 480.0f;

    // Spin about `bodyAxis` (with a tiny nudge off it) and return the most the
    // body-fixed spin axis ever turns away from its start, as a dot product:
    // ~+1 means "stayed put" (stable); reaching <0 means it flipped over.
    auto minAlignment = [&](RigidBody b, const Vec3& bodyAxis, const Vec3& nudge) {
        b.setAngularVelocity(bodyAxis * 8.0f + nudge);
        const Vec3 initDir = rotate(b.orientation, bodyAxis);
        Real minDot = 1e9f;
        for (int i = 0; i < 6000; ++i) {             // ~12.5 s
            minDot = std::min(minDot, dot(rotate(b.orientation, bodyAxis), initDir));
            b.integrate(dt);
        }
        return minDot;
    };

    // Intermediate axis (y): the spin axis must reverse at some point.
    CHECK(minAlignment(makeBox(), {0, 1, 0}, {0.05f, 0, 0}) < -0.5f);
    // Minimum axis (x) and maximum axis (z): the spin axis stays put.
    CHECK(minAlignment(makeBox(), {1, 0, 0}, {0, 0.05f, 0}) > 0.9f);
    CHECK(minAlignment(makeBox(), {0, 0, 1}, {0, 0.05f, 0}) > 0.9f);

    // And through all of it, energy stays essentially constant (the midpoint
    // integrator does not let a free spin gain energy).
    RigidBody b = makeBox();
    b.setAngularVelocity({0.05f, 8.0f, 0.0f});
    const Real E0 = b.kineticEnergy();
    for (int i = 0; i < 6000; ++i) b.integrate(dt);
    CHECK(std::fabs(b.kineticEnergy() - E0) < 0.05f * E0);
}

TEST_CASE("Parallel-axis theorem adds the expected offset term") {
    // Shifting a point mass' (tiny box) inertia by d should add ~ m·d² about the
    // perpendicular axes. Use a small box so its own inertia is negligible.
    const Real m = 2.0f;
    const Mat3 Icom = inertia::box(m, {0.01f, 0.01f, 0.01f});
    const Mat3 shifted = inertia::shift(Icom, m, {0, 3.0f, 0});   // offset along y
    // Perpendicular axes (x, z) gain m·d² = 2·9 = 18; the parallel axis (y) does not.
    CHECK(close(shifted.at(0,0) - Icom.at(0,0), 18.0f, 1e-2f));
    CHECK(close(shifted.at(2,2) - Icom.at(2,2), 18.0f, 1e-2f));
    CHECK(close(shifted.at(1,1) - Icom.at(1,1), 0.0f,  1e-3f));
}
