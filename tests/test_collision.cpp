// ===========================================================================
//  test_collision.cpp — Part IV: AABBs, broadphase, narrowphase, SAT, GJK/EPA
// ---------------------------------------------------------------------------
//  The headline guarantees: the three broadphases return the SAME candidate set
//  (a fast method that disagreed with brute force would be a silent bug), and
//  GJK/EPA agrees with SAT on shapes both can handle.
// ===========================================================================
#include <doctest/doctest.h>
#include "physics/aabb.hpp"
#include "physics/broadphase.hpp"
#include "physics/narrowphase.hpp"
#include "physics/sat.hpp"
#include "physics/gjk.hpp"

#include <algorithm>
#include <random>
#include <vector>

using namespace p3d;

static bool close(Real a, Real b, Real eps = 1e-3f) { return std::fabs(a - b) <= eps; }
static bool close(const Vec3& a, const Vec3& b, Real eps = 1e-3f) {
    return close(a.x, b.x, eps) && close(a.y, b.y, eps) && close(a.z, b.z, eps);
}

// Eight world-space corners of an oriented box (for GJK support tests).
static std::vector<Vec3> boxVerts(const Vec3& pos, const Quat& rot, const Vec3& half) {
    std::vector<Vec3> v;
    for (int sx=-1;sx<=1;sx+=2) for (int sy=-1;sy<=1;sy+=2) for (int sz=-1;sz<=1;sz+=2)
        v.push_back(pos + rotate(rot, {sx*half.x, sy*half.y, sz*half.z}));
    return v;
}

TEST_CASE("AABB overlap needs all three axes") {
    const AABB a{{0,0,0},{2,2,2}};
    CHECK(a.overlaps(AABB{{1,1,1},{3,3,3}}));      // clearly overlapping
    CHECK_FALSE(a.overlaps(AABB{{3,0,0},{4,2,2}})); // gap on x only → apart
    CHECK_FALSE(a.overlaps(AABB{{0,5,0},{2,6,2}})); // gap on y only → apart
}

TEST_CASE("A rotated box has a larger world AABB than its half-extents") {
    const AABB axisAligned = aabbOfBox({0,0,0}, Quat::identity(), {1,1,1});
    CHECK(close(axisAligned.extent(), Vec3{1,1,1}));
    // 45° about z: the box pokes out to sqrt(2) on x and y.
    const AABB rotated = aabbOfBox({0,0,0}, Quat::fromAxisAngle({0,0,1}, radians(45)), {1,1,1});
    CHECK(rotated.extent().x > 1.3f);
    CHECK(rotated.extent().x < 1.5f);
}

TEST_CASE("All three broadphases return the same candidate set") {
    // A random cloud of AABBs; the fast methods must agree with brute force.
    std::mt19937 rng(99);
    std::uniform_real_distribution<Real> p(-8, 8), s(0.2f, 1.0f);
    std::vector<AABB> boxes;
    for (int i = 0; i < 200; ++i) {
        const Vec3 c{p(rng), p(rng), p(rng)};
        boxes.push_back(AABB::fromCenterHalf(c, Vec3(s(rng))));
    }
    auto brute = broadphaseBrute(boxes);
    auto sweep = broadphaseSweep(boxes);
    auto grid  = broadphaseGrid(boxes, 1.5f);
    std::sort(brute.begin(), brute.end());
    std::sort(sweep.begin(), sweep.end());
    std::sort(grid.begin(),  grid.end());
    CHECK(sweep == brute);
    CHECK(grid  == brute);
    CHECK(brute.size() > 0);   // the test would be meaningless with no overlaps
}

TEST_CASE("Narrowphase sphere contacts") {
    Contact c;
    // Sphere–sphere: centres 1.5 apart, radii 1 → overlap 0.5, normal B→A = -x.
    REQUIRE(collideSphereSphere({0,0,0},1, {1.5f,0,0},1, c));
    CHECK(close(c.penetration, 0.5f));
    CHECK(close(c.normal, Vec3{-1,0,0}));

    // Sphere–plane floor: centre 0.3 above, r 1 → penetration 0.7, normal +y.
    REQUIRE(collideSpherePlane({0,0.3f,0},1, {0,1,0},0, c));
    CHECK(close(c.penetration, 0.7f));
    CHECK(close(c.normal, Vec3{0,1,0}));

    // Sphere–box: just outside the +x face → shallow contact, normal +x.
    REQUIRE(collideSphereBox({1.8f,0,0},1, {0,0,0}, Quat::identity(), {1,1,1}, c));
    CHECK(close(c.penetration, 0.2f));
    CHECK(close(c.normal, Vec3{1,0,0}));

    // Clearly separated → no contact.
    CHECK_FALSE(collideSphereSphere({0,0,0},1, {5,0,0},1, c));
}

TEST_CASE("SAT box-box produces a face manifold") {
    Contact m[8];
    // B stacked on A, overlapping 0.4 in y → 4-point manifold, normal B→A = -y.
    const int n = collideBoxBox({0,0,0}, Quat::identity(), {1,1,1},
                                {0,1.6f,0}, Quat::identity(), {1,1,1}, m, 8);
    CHECK(n == 4);
    CHECK(close(m[0].normal, Vec3{0,-1,0}));
    CHECK(close(m[0].penetration, 0.4f));

    // Far apart → no contacts.
    CHECK(collideBoxBox({0,0,0}, Quat::identity(), {1,1,1},
                        {5,0,0}, Quat::identity(), {1,1,1}, m, 8) == 0);
}

TEST_CASE("GJK agrees with SAT, and EPA recovers the depth") {
    const Quat I = Quat::identity();
    // Overlapping axis-aligned boxes.
    auto A = boxVerts({0,0,0}, I, {1,1,1});
    auto B = boxVerts({1.5f,0,0}, I, {1,1,1});
    CHECK(gjkIntersect(A, B));
    Penetration p = gjkEpa(A, B);
    CHECK(p.hit);
    CHECK(close(p.depth, 0.5f, 1e-2f));
    CHECK(close(p.normal, Vec3{-1,0,0}, 1e-2f));   // B→A, matching Contact

    // Separated boxes → GJK says no, EPA reports no hit.
    auto C = boxVerts({5,0,0}, I, {1,1,1});
    CHECK_FALSE(gjkIntersect(A, C));
    CHECK_FALSE(gjkEpa(A, C).hit);

    // A rotated overlap: GJK/EPA depth must match SAT's for the same pose.
    const Quat r45 = Quat::fromAxisAngle({0,1,0}, radians(45));
    auto D = boxVerts({1.6f,0,0}, r45, {1,1,1});
    Penetration pe = gjkEpa(A, D);
    Contact m[8];
    const int sn = collideBoxBox({0,0,0}, I, {1,1,1}, {1.6f,0,0}, r45, {1,1,1}, m, 8);
    REQUIRE(pe.hit);
    REQUIRE(sn > 0);
    CHECK(close(pe.depth, m[0].penetration, 2e-2f));   // same penetration depth
}
