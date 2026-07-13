// ===========================================================================
//  world.cpp — the full simulation step
// ===========================================================================
#include "world.hpp"

#include "broadphase.hpp"
#include "narrowphase.hpp"
#include "sat.hpp"

#include <cmath>

namespace p3d {

// A stable key for a body pair (order-independent).
static uint64_t pairKey(int a, int b) {
    const uint32_t lo = static_cast<uint32_t>(a < b ? a : b);
    const uint32_t hi = static_cast<uint32_t>(a < b ? b : a);
    return (static_cast<uint64_t>(hi) << 32) | lo;
}

int World::awakeCount() const {
    int n = 0;
    for (size_t i = 0; i < rbs.size(); ++i)
        if (!rbs[i].isStatic() && awake[i]) ++n;
    return n;
}

// Dispatch to the right narrowphase test based on the two collider types. Fills
// `out` with contacts whose a/b indices and B→A normal are already correct, and
// returns the count.
int World::collidePair(int i, int j, Contact out[8]) const {
    using T = Collider::Type;
    const Collider& ci = colliders[i];
    const Collider& cj = colliders[j];
    const RigidBody& bi = rbs[i];
    const RigidBody& bj = rbs[j];

    if (ci.type == T::Sphere && cj.type == T::Sphere) {
        Contact c;
        if (collideSphereSphere(bi.position, ci.radius, bj.position, cj.radius, c)) {
            c.a = i; c.b = j; out[0] = c; return 1;
        }
        return 0;
    }
    if (ci.type == T::Sphere && cj.type == T::Box) {
        Contact c;
        if (collideSphereBox(bi.position, ci.radius, bj.position, bj.orientation, cj.half, c)) {
            c.a = i; c.b = j; out[0] = c; return 1;    // sphere is A, box is B
        }
        return 0;
    }
    if (ci.type == T::Box && cj.type == T::Sphere) {
        Contact c;
        if (collideSphereBox(bj.position, cj.radius, bi.position, bi.orientation, ci.half, c)) {
            c.a = j; c.b = i; out[0] = c; return 1;    // keep sphere as A, box as B
        }
        return 0;
    }
    // Box vs box.
    Contact m[8];
    const int k = collideBoxBox(bi.position, bi.orientation, ci.half,
                                bj.position, bj.orientation, cj.half, m, 8);
    for (int c = 0; c < k; ++c) { m[c].a = i; m[c].b = j; out[c] = m[c]; }
    return k;
}

void World::generateContacts() {
    constraints_.clear();
    const int n = static_cast<int>(rbs.size());

    std::vector<AABB> boxes(n);
    for (int i = 0; i < n; ++i) boxes[i] = colliders[i].aabb(rbs[i].position, rbs[i].orientation);

    Contact tmp[8];
    for (const BroadPair& pr : broadphaseGrid(boxes, 2.0f)) {
        const int i = pr.first, j = pr.second;
        if (rbs[i].isStatic() && rbs[j].isStatic()) continue;   // two walls never collide

        const int k = collidePair(i, j, tmp);
        if (k == 0) continue;

        // Contact makes bodies keep each other awake: a moving body wakes what it
        // touches, so a stack doesn't sink into a sleeping neighbour.
        if (awake[i] && !rbs[j].isStatic()) awake[j] = 1;
        if (awake[j] && !rbs[i].isStatic()) awake[i] = 1;
        if (!awake[i] && !awake[j]) continue;                   // both asleep → skip solving

        for (int c = 0; c < k; ++c) {
            ContactConstraint cc;
            cc.a = tmp[c].a; cc.b = tmp[c].b;
            cc.point = tmp[c].point; cc.normal = tmp[c].normal; cc.penetration = tmp[c].penetration;
            cc.restitution = combineRestitution(colliders[cc.a].restitution, colliders[cc.b].restitution);
            cc.friction    = combineFriction(colliders[cc.a].friction, colliders[cc.b].friction);
            constraints_.push_back(cc);
        }
    }
}

// Seed each fresh contact with the impulse the nearest cached contact ended with
// last frame. Matching by proximity is crude but works: contacts barely move
// between frames on a settled stack.
void World::seedFromCache() {
    if (!solver.warmStart) return;
    for (ContactConstraint& c : constraints_) {
        auto it = cache_.find(pairKey(c.a, c.b));
        if (it == cache_.end()) continue;
        Real best = 0.04f * 0.04f;   // squared match tolerance
        const Cached* match = nullptr;
        for (const Cached& cc : it->second) {
            const Real d2 = lengthSq(cc.point - c.point);
            if (d2 < best) { best = d2; match = &cc; }
        }
        if (match) { c.normalImpulse = match->n; c.tangent1Impulse = match->t1; c.tangent2Impulse = match->t2; }
    }
}

void World::updateCache() {
    cache_.clear();
    for (const ContactConstraint& c : constraints_)
        cache_[pairKey(c.a, c.b)].push_back({c.point, c.normalImpulse, c.tangent1Impulse, c.tangent2Impulse});
}

void World::updateSleeping(Real dt) {
    if (!sleepingEnabled) return;
    const int n = static_cast<int>(rbs.size());
    for (int i = 0; i < n; ++i) {
        if (rbs[i].isStatic()) continue;
        const Real lin = length(rbs[i].linearVelocity);
        const Real ang = length(rbs[i].angularVelocity());
        if (lin < linearSleepSpeed && ang < angularSleepSpeed) {
            sleepTimer[i] += dt;
            if (sleepTimer[i] > timeToSleep) {
                awake[i] = 0;
                rbs[i].linearVelocity = Vec3{};      // freeze it cleanly
                rbs[i].angularMomentum = Vec3{};
            }
        } else {
            sleepTimer[i] = 0;
            awake[i] = 1;
        }
    }
}

void World::step(Real dt) {
    const int n = static_cast<int>(rbs.size());

    // 1. Gravity → velocities (awake dynamic bodies only).
    for (int i = 0; i < n; ++i)
        if (!rbs[i].isStatic() && awake[i]) rbs[i].linearVelocity += gravity * dt;

    // 2–3. Find contacts.
    generateContacts();

    // 4. Warm start, prepare, then iterate the impulse solve.
    seedFromCache();
    prepareContacts(constraints_, rbs, solver, dt);
    if (solver.warmStart) warmStartContacts(constraints_, rbs);
    for (int it = 0; it < solver.iterations; ++it) solveVelocities(constraints_, rbs);

    // 5. Integrate positions with the freshly solved velocities.
    for (int i = 0; i < n; ++i)
        if (!rbs[i].isStatic() && awake[i]) rbs[i].integratePosition(dt);

    // 6. Cache impulses for next frame's warm start, then update sleeping.
    updateCache();
    updateSleeping(dt);
}

}  // namespace p3d
