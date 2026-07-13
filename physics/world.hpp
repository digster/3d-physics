// ===========================================================================
//  world.hpp — the whole engine, assembled: bodies + the simulation step
// ---------------------------------------------------------------------------
//  Introduced in: Chapter 16, grown through Chapter 18 (sleeping)
//
//  This is where four parts finally meet. A World owns a set of rigid bodies
//  (Part III), each with a collider (Part IV), and its step() runs the complete
//  loop:
//
//     1. apply gravity to velocities
//     2. broadphase  → candidate pairs        (Ch 12)
//     3. narrowphase → contacts               (Ch 13–15)
//     4. warm-start + solve impulses          (Ch 16–17)
//     5. integrate positions                  (Ch 11)
//     6. put still bodies to sleep            (Ch 18)
//
//  Bodies are stored as parallel arrays (rigid body, collider, awake flag) so
//  the solver can take the rigid bodies directly.
// ===========================================================================
#pragma once

#include "rigidbody.hpp"
#include "collider.hpp"
#include "solver.hpp"

#include <cstdint>
#include <unordered_map>
#include <vector>

namespace p3d {

class World {
public:
    // --- Scene ------------------------------------------------------------
    std::vector<RigidBody> rbs;         // the bodies
    std::vector<Collider>  colliders;   // one shape+material per body
    std::vector<char>      awake;       // 0 = sleeping, 1 = awake (char = simple bool)
    std::vector<Real>      sleepTimer;  // seconds a body has been nearly still

    // --- Tunables ---------------------------------------------------------
    Vec3 gravity{0, -9.81f, 0};
    SolverConfig solver{};
    bool sleepingEnabled{true};
    Real linearSleepSpeed{0.08f};       // below this (and angular) a body may sleep
    Real angularSleepSpeed{0.15f};
    Real timeToSleep{0.6f};

    // Add a body; returns its index. Give the RigidBody its mass/inertia to match
    // the collider first (see the makeBody helper below, used by the chapters).
    int add(const RigidBody& rb, const Collider& col) {
        rbs.push_back(rb);
        colliders.push_back(col);
        awake.push_back(1);
        sleepTimer.push_back(0);
        return static_cast<int>(rbs.size()) - 1;
    }

    // Advance the whole simulation by dt.
    void step(Real dt);

    // The contacts solved this step (for drawing / stats).
    const std::vector<ContactConstraint>& contacts() const { return constraints_; }
    int awakeCount() const;

private:
    std::vector<ContactConstraint> constraints_;

    // Warm-start cache: for each body pair, the impulses each contact point ended
    // with last frame, so we can re-apply them as a head start this frame.
    struct Cached { Vec3 point; Real n, t1, t2; };
    std::unordered_map<uint64_t, std::vector<Cached>> cache_;

    void generateContacts();
    int  collidePair(int i, int j, Contact out[8]) const;
    void seedFromCache();
    void updateCache();
    void updateSleeping(Real dt);
};

// Convenience: build a RigidBody already fitted to a collider's shape at a mass.
// (Static bodies: pass mass <= 0.)
inline RigidBody makeBody(const Collider& col, const Vec3& pos, Real mass) {
    RigidBody b;
    b.position = pos;
    if (mass <= 0) { b.makeStatic(); }
    else { b.setMass(mass); b.setInertia(col.inertiaTensor(mass)); }
    return b;
}

}  // namespace p3d
