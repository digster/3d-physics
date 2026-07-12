// ===========================================================================
//  test_physics.cpp — Part II: particles, forces, and constraints
// ---------------------------------------------------------------------------
//  These pin down the physical properties the demos rely on: that gravity is
//  mass-independent, that a spring conserves momentum, and that the position
//  constraint solver actually converges to the rest length.
// ===========================================================================
#include <doctest/doctest.h>
#include "physics/particle.hpp"
#include "physics/forces.hpp"
#include "physics/constraint.hpp"

#include <vector>

using namespace p3d;

static bool close(Real a, Real b, Real eps = 1e-4f) { return std::fabs(a - b) <= eps; }
static bool close(const Vec3& a, const Vec3& b, Real eps = 1e-4f) {
    return close(a.x, b.x, eps) && close(a.y, b.y, eps) && close(a.z, b.z, eps);
}

TEST_CASE("A constant force produces constant acceleration (v = a t)") {
    Particle p; p.setMass(2.0f);            // 2 kg
    const Vec3 F{4, 0, 0};                  // → a = F/m = (2,0,0)
    const Real dt = 1.0f / 1000.0f;
    for (int i = 0; i < 1000; ++i) { p.addForce(F); p.integrateForces(dt); }
    // After 1 second: v ≈ a·t = 2 m/s. (Semi-implicit Euler is exact in v here.)
    CHECK(close(p.velocity.x, 2.0f, 1e-2f));
    CHECK(p.position.x > 0.9f);             // and it has moved forward ~1 m
}

TEST_CASE("Gravity accelerates every mass equally (Galileo)") {
    Particle light; light.setMass(0.5f);
    Particle heavy; heavy.setMass(50.0f);
    const Vec3 g{0, -9.81f, 0};
    const Real dt = 1.0f / 240.0f;
    for (int i = 0; i < 240; ++i) {
        light.clearForces(); heavy.clearForces();
        force::gravity(light, g); force::gravity(heavy, g);
        light.integrateForces(dt); heavy.integrateForces(dt);
    }
    // Same velocity and position regardless of mass.
    CHECK(close(light.velocity.y, heavy.velocity.y, 1e-3f));
    CHECK(close(light.position.y, heavy.position.y, 1e-3f));
    CHECK(light.velocity.y < -9.0f);        // roughly -9.81 m/s after 1 s
}

TEST_CASE("A static particle never moves, whatever the force") {
    Particle p; p.makeStatic();
    p.addForce({1000, 1000, 1000});
    p.integrateForces(1.0f / 60.0f);
    CHECK(close(p.position, Vec3{}));
    CHECK(close(p.velocity, Vec3{}));
}

TEST_CASE("A spring at its rest length exerts no force") {
    Particle a; a.position = {0, 0, 0};
    Particle b; b.position = {2, 0, 0};     // exactly rest length apart
    force::spring(a, b, 100.0f, 2.0f);
    CHECK(close(a.forceAccum, Vec3{}));
    CHECK(close(b.forceAccum, Vec3{}));
}

TEST_CASE("A stretched spring pulls its ends together") {
    Particle a; a.position = {0, 0, 0};
    Particle b; b.position = {3, 0, 0};     // stretched by 1 beyond rest 2
    force::spring(a, b, 100.0f, 2.0f);      // |F| = k·stretch = 100
    // a is pulled toward +x (toward b); b toward -x (toward a); equal & opposite.
    CHECK(a.forceAccum.x > 0);
    CHECK(b.forceAccum.x < 0);
    CHECK(close(a.forceAccum + b.forceAccum, Vec3{}));   // Newton's third law
    CHECK(close(length(a.forceAccum), 100.0f, 1e-3f));
}

TEST_CASE("An internal spring conserves the pair's momentum") {
    Particle a; a.setMass(1.0f); a.position = {-1.5f, 0, 0};   // stretched...
    Particle b; b.setMass(3.0f); b.position = { 1.5f, 0, 0};   // ...different masses
    const Real dt = 1.0f / 240.0f;
    for (int i = 0; i < 400; ++i) {
        a.clearForces(); b.clearForces();
        force::spring(a, b, 60.0f, 1.0f);   // only internal force, no gravity
        a.integrateForces(dt); b.integrateForces(dt);
    }
    // Started from rest, so total momentum must stay ≈ 0 the whole time.
    const Vec3 momentum = a.velocity * a.mass() + b.velocity * b.mass();
    CHECK(close(momentum, Vec3{}, 1e-3f));
}

TEST_CASE("Drag always removes speed") {
    Particle p; p.setMass(1.0f); p.velocity = {5, 0, 0};
    const Real before = length(p.velocity);
    p.clearForces();
    force::drag(p, 0.5f, 0.1f);
    p.integrateForces(1.0f / 120.0f);
    CHECK(length(p.velocity) < before);     // slowed, never sped up
}

TEST_CASE("A distance constraint snaps to rest length (one static end)") {
    std::vector<Particle> ps(2);
    ps[0].position = {0, 0, 0}; ps[0].makeStatic();   // anchor
    ps[1].position = {5, 0, 0}; ps[1].setMass(1.0f);
    DistanceConstraint c{0, 1, 2.0f, 1.0f};
    c.solve(ps);                             // one pass suffices: all move goes to ps[1]
    CHECK(close(distance(ps[0].position, ps[1].position), 2.0f, 1e-4f));
    CHECK(close(ps[0].position, Vec3{}));    // the anchor stayed put
}

TEST_CASE("A distance constraint splits the move by inverse mass") {
    std::vector<Particle> ps(2);
    ps[0].position = {0, 0, 0}; ps[0].setMass(1.0f);
    ps[1].position = {4, 0, 0}; ps[1].setMass(1.0f);   // equal mass
    const Vec3 midBefore = (ps[0].position + ps[1].position) * 0.5f;
    DistanceConstraint c{0, 1, 2.0f, 1.0f};
    c.solve(ps);
    CHECK(close(distance(ps[0].position, ps[1].position), 2.0f, 1e-4f));
    // Equal masses ⇒ the midpoint is unchanged (each moved the same amount).
    CHECK(close((ps[0].position + ps[1].position) * 0.5f, midBefore, 1e-4f));
}

TEST_CASE("Iterating constraints converges a whole chain") {
    // Five beads strung out too far apart; the first is pinned. With rest 1.0
    // and enough relaxation passes, every adjacent gap should approach 1.0.
    std::vector<Particle> ps(5);
    for (int i = 0; i < 5; ++i) { ps[i].position = {Real(i) * 2.0f, 0, 0}; ps[i].setMass(1.0f); }
    ps[0].makeStatic();
    std::vector<DistanceConstraint> cs;
    for (int i = 0; i < 4; ++i) cs.push_back({i, i + 1, 1.0f, 1.0f});

    satisfyConstraints(cs, ps, 60);
    for (int i = 0; i < 4; ++i)
        CHECK(close(distance(ps[i].position, ps[i + 1].position), 1.0f, 1e-2f));
}
