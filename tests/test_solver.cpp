// ===========================================================================
//  test_solver.cpp — Part V: the contact solver and the World
// ---------------------------------------------------------------------------
//  These pin down the behaviours the whole part promises: bounces scale with
//  restitution, boxes come to rest (and sleep), stacks stay standing, friction
//  stops sliding, and an isolated collision conserves momentum.
// ===========================================================================
#include <doctest/doctest.h>
#include "physics/world.hpp"

using namespace p3d;

// Build a world with a static floor whose top surface is at y = 0.
static World floorWorld(Real floorRestitution = 0.2f, Real floorFriction = 0.6f) {
    World w;
    Collider fl = Collider::box({20, 0.5f, 20}, floorRestitution, floorFriction);
    w.add(makeBody(fl, {0, -0.5f, 0}, 0), fl);
    return w;
}

// Drop a ball and return the highest point its centre reaches after the bounce.
static Real bounceHeight(Real restitution) {
    World w = floorWorld(0.0f);
    w.sleepingEnabled = false;
    Collider bc = Collider::sphere(0.5f, restitution, 0.3f);
    const int ball = w.add(makeBody(bc, {0, 3.0f, 0}, 1.0f), bc);

    bool bounced = false;
    Real apex = 0;
    for (int i = 0; i < 700; ++i) {
        w.step(1.0f / 120.0f);
        if (w.rbs[ball].linearVelocity.y > 0.1f) bounced = true;
        if (bounced && w.rbs[ball].linearVelocity.y <= 0)
            apex = std::max(apex, w.rbs[ball].position.y);
    }
    return apex;
}

TEST_CASE("Bounce height grows with restitution") {
    const Real dead   = bounceHeight(0.0f);
    const Real bouncy = bounceHeight(0.5f);
    const Real superball = bounceHeight(0.9f);
    CHECK(dead < 0.7f);                    // e=0: basically doesn't bounce
    CHECK(bouncy > dead + 0.3f);           // e=0.5: a real bounce
    CHECK(superball > bouncy + 0.4f);      // e=0.9: much higher
    CHECK(superball < 3.0f);               // ...but never higher than it fell
}

TEST_CASE("A box comes to rest on the floor and falls asleep") {
    World w = floorWorld();
    Collider bc = Collider::box({0.5f, 0.5f, 0.5f}, 0.1f, 0.6f);
    const int box = w.add(makeBody(bc, {0, 1.2f, 0}, 1.0f), bc);
    for (int i = 0; i < 400; ++i) w.step(1.0f / 120.0f);
    CHECK(std::fabs(w.rbs[box].position.y - 0.5f) < 0.02f);   // resting on the surface
    CHECK(length(w.rbs[box].linearVelocity) < 0.05f);         // and still
    CHECK(w.awake[box] == 0);                                 // ...and asleep
}

TEST_CASE("A stack of boxes settles, stays standing, and sleeps") {
    // Sleeping enabled (the default): once the stack settles it freezes, which
    // is exactly how a real engine keeps piles stable and cheap (Chapter 18).
    World w = floorWorld(0.0f, 0.8f);
    Collider bc = Collider::box({0.5f, 0.5f, 0.5f}, 0.05f, 0.8f);
    int top = -1;
    for (int k = 0; k < 4; ++k) top = w.add(makeBody(bc, {0, 0.5f + k * 1.0f, 0}, 1.0f), bc);
    for (int i = 0; i < 600; ++i) w.step(1.0f / 120.0f);
    // The top box stays near its start height (3.5) and doesn't slide off.
    CHECK(w.rbs[top].position.y > 3.3f);              // didn't collapse
    CHECK(std::fabs(w.rbs[top].position.x) < 0.15f);  // didn't drift sideways
    CHECK(w.awake[top] == 0);                         // settled and asleep
}

TEST_CASE("Friction brings a sliding box to a stop") {
    World w = floorWorld(0.0f, 0.9f);
    w.sleepingEnabled = false;
    Collider bc = Collider::box({0.5f, 0.5f, 0.5f}, 0.0f, 0.9f);
    const int box = w.add(makeBody(bc, {0, 0.5f, 0}, 1.0f), bc);
    w.rbs[box].linearVelocity = {4.0f, 0, 0};
    for (int i = 0; i < 400; ++i) w.step(1.0f / 120.0f);
    CHECK(std::fabs(w.rbs[box].linearVelocity.x) < 0.2f);   // friction stopped it
}

TEST_CASE("An isolated collision conserves linear momentum") {
    // No gravity, no floor: one ball strikes another. Total momentum is fixed.
    World w;
    w.gravity = {0, 0, 0};
    w.sleepingEnabled = false;
    Collider bc = Collider::sphere(0.5f, 0.5f, 0.0f);   // frictionless so only normal impulses
    const int a = w.add(makeBody(bc, {-2, 0, 0}, 1.0f), bc);
    const int b = w.add(makeBody(bc, { 2, 0, 0}, 1.0f), bc);
    w.rbs[a].linearVelocity = {3.0f, 0, 0};             // a heads toward b

    const Vec3 p0 = w.rbs[a].linearVelocity * w.rbs[a].mass() + w.rbs[b].linearVelocity * w.rbs[b].mass();
    for (int i = 0; i < 300; ++i) w.step(1.0f / 120.0f);
    const Vec3 p1 = w.rbs[a].linearVelocity * w.rbs[a].mass() + w.rbs[b].linearVelocity * w.rbs[b].mass();
    CHECK(length(p1 - p0) < 0.05f);                     // momentum unchanged by the internal collision
    CHECK(w.rbs[b].linearVelocity.x > 0.5f);            // b was actually knocked forward
}
