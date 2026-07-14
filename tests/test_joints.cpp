// ===========================================================================
//  test_joints.cpp — Part VI: distance, ball-socket, and hinge joints
// ---------------------------------------------------------------------------
//  Each joint must hold its constraint while the bodies swing under gravity:
//  a rod keeps its length, a ball-socket keeps two points together, a hinge
//  keeps a shared axis (and lets the body rotate only about it).
// ===========================================================================
#include <doctest/doctest.h>
#include "physics/world.hpp"

#include <memory>

using namespace p3d;

TEST_CASE("A distance joint keeps its length as it swings") {
    World w;
    w.sleepingEnabled = false;
    const int anchor = w.add(makeBody(Collider::box({0.1f, 0.1f, 0.1f}), {0, 5, 0}, 0),
                             Collider::box({0.1f, 0.1f, 0.1f}));
    Collider bc = Collider::box({0.3f, 0.3f, 0.3f});
    const int bob = w.add(makeBody(bc, {2.5f, 5, 0}, 1.0f), bc);
    w.joints.push_back(std::make_unique<DistanceJoint>(anchor, bob, Vec3{}, Vec3{}, 2.5f));

    Real minL = 1e9f, maxL = 0;
    bool swung = false;
    for (int i = 0; i < 600; ++i) {
        w.step(1.0f / 120.0f);
        const Real L = distance(w.rbs[anchor].position, w.rbs[bob].position);
        minL = std::min(minL, L); maxL = std::max(maxL, L);
        if (w.rbs[bob].position.y < 4.0f) swung = true;
    }
    CHECK(minL > 2.45f);   // never stretches or compresses much...
    CHECK(maxL < 2.55f);
    CHECK(swung);          // ...but it did swing down under gravity
}

TEST_CASE("A ball-socket joint keeps its two anchor points together") {
    World w;
    w.sleepingEnabled = false;
    const int anchor = w.add(makeBody(Collider::box({0.1f, 0.1f, 0.1f}), {0, 5, 0}, 0),
                             Collider::box({0.1f, 0.1f, 0.1f}));
    Collider bc = Collider::box({0.5f, 0.5f, 0.5f});
    const int bob = w.add(makeBody(bc, {0, 3.5f, 0}, 1.0f), bc);
    // Pin the bob's top corner to the anchor so it hangs and swings.
    w.joints.push_back(std::make_unique<BallSocketJoint>(anchor, bob, Vec3{}, Vec3{0, 1.5f, 0}));

    Real maxGap = 0;
    for (int i = 0; i < 600; ++i) {
        w.step(1.0f / 120.0f);
        const Vec3 wa = w.rbs[anchor].worldPoint({});
        const Vec3 wb = w.rbs[bob].worldPoint({0, 1.5f, 0});
        maxGap = std::max(maxGap, distance(wa, wb));
    }
    CHECK(maxGap < 0.02f);   // the two anchor points stay glued together
}

TEST_CASE("A hinge joint holds its axis while gravity swings the body about it") {
    // Hinge axis horizontal (z), door sticking out in +x — gravity torques it
    // about the axis, so it swings down like a trapdoor.
    World w;
    w.sleepingEnabled = false;
    const int frame = w.add(makeBody(Collider::box({0.1f, 0.1f, 1.5f}), {0, 3, 0}, 0),
                            Collider::box({0.1f, 0.1f, 1.5f}));
    Collider dc = Collider::box({1.0f, 0.1f, 1.0f});
    const int door = w.add(makeBody(dc, {1.0f, 3, 0}, 1.0f), dc);
    w.joints.push_back(std::make_unique<HingeJoint>(frame, door, Vec3{}, Vec3{-1.0f, 0, 0},
                                                    Vec3{0, 0, 1}, Vec3{0, 0, 1}));

    Real maxPivotGap = 0, minAxisAlign = 1, lowestArm = 1;
    for (int i = 0; i < 250; ++i) {
        w.step(1.0f / 120.0f);
        maxPivotGap = std::max(maxPivotGap,
            distance(w.rbs[frame].worldPoint({}), w.rbs[door].worldPoint({-1.0f, 0, 0})));
        minAxisAlign = std::min(minAxisAlign,
            dot(rotate(w.rbs[door].orientation, {0, 0, 1}), Vec3{0, 0, 1}));
        // The door's arm (local +x) dips below horizontal as it swings.
        lowestArm = std::min(lowestArm, rotate(w.rbs[door].orientation, {1, 0, 0}).y);
    }
    CHECK(maxPivotGap < 0.2f);      // pivot held throughout
    CHECK(minAxisAlign > 0.99f);    // hinge axis (z) stayed aligned — only turned about it
    CHECK(lowestArm < -0.5f);       // and it really swung down under gravity
}
