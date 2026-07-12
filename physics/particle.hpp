// ===========================================================================
//  particle.hpp — a point mass, the simplest thing physics can act on
// ---------------------------------------------------------------------------
//  Introduced in: Chapter 6 (Particles & Forces)
//
//  A particle is a point with a mass, a position, and a way to move. It has no
//  size and no orientation — that keeps the physics pure Newton (F = m a) with
//  none of the rotational complications we take on in Part III. Almost
//  everything in Part II is a bag of these: a fountain of them, masses on
//  springs, beads on a rope, or a grid of them stitched into cloth.
//
//  This one struct deliberately supports BOTH styles of simulation we teach in
//  Part II, so you can compare them directly:
//
//    * FORCE-BASED (Chapters 6–7): accumulate forces into `forceAccum`, then
//      call integrateForces() — the semi-implicit Euler from Chapter 5.
//    * POSITION-BASED / Verlet (Chapters 8–9): store the previous position and
//      call integrateVerlet(); motion is inferred from (position - prevPosition),
//      which makes distance CONSTRAINTS trivial to enforce afterwards.
//
//  INVERSE MASS. We store 1/mass rather than mass. Two reasons, both practical:
//  it turns the frequent division F/m into a multiply, and it gives us a clean
//  way to say "infinitely heavy / immovable" — an inverse mass of 0. Pinned
//  cloth corners and rope anchors are just particles with invMass = 0.
// ===========================================================================
#pragma once

#include "../common/math/vec3.hpp"

namespace p3d {

struct Particle {
    Vec3 position{};
    Vec3 velocity{};       // used by the force-based path (Ch 6–7)
    Vec3 prevPosition{};   // used by the Verlet path       (Ch 8–9)
    Vec3 forceAccum{};     // forces summed this step, then cleared

    // Inverse mass. 1 by default (a 1 kg particle); 0 means immovable.
    Real invMass{1};

    // --- Mass helpers ------------------------------------------------------
    void setMass(Real m) { invMass = (m <= 0) ? 0 : Real(1) / m; }
    Real mass() const    { return (invMass == 0) ? 0 : Real(1) / invMass; }
    // "Static" = pinned / infinitely heavy: it feels forces logically but never
    // actually moves. Anchors and pinned cloth corners are static particles.
    bool isStatic() const { return invMass == 0; }
    void makeStatic()     { invMass = 0; velocity = Vec3{}; }

    // --- Force accumulation (the force-based path) -------------------------
    // Force generators (forces.hpp) call addForce during a step; the integrator
    // consumes the total and clearForces resets it for the next step.
    void addForce(const Vec3& f) { forceAccum += f; }
    void clearForces()           { forceAccum = Vec3{}; }

    // Advance one step with semi-implicit Euler using the accumulated force.
    // acceleration = F / m = F * invMass. See Chapter 5 for why velocity is
    // updated before position. Static particles ignore this entirely.
    void integrateForces(Real dt) {
        if (isStatic()) { clearForces(); return; }
        const Vec3 acceleration = forceAccum * invMass;
        velocity += acceleration * dt;   // velocity first...
        position += velocity * dt;       // ...then move with the new velocity
        clearForces();
    }

    // --- Verlet integration (the position-based path) ---------------------
    // Instead of storing velocity explicitly, Verlet remembers where the
    // particle was last step. The difference (position - prevPosition) IS the
    // velocity times dt, so a step is:
    //     next = position + (position - prevPosition) + acceleration * dt²
    // The `damping` factor (slightly < 1) bleeds off a little of that implicit
    // velocity each step, which quiets the jitter that constraint solving can
    // introduce. Static particles stay put.
    void integrateVerlet(const Vec3& acceleration, Real dt, Real damping = Real(0.995)) {
        if (isStatic()) { prevPosition = position; return; }
        const Vec3 velocityStep = (position - prevPosition) * damping;
        const Vec3 next = position + velocityStep + acceleration * (dt * dt);
        prevPosition = position;
        position = next;
    }

    // Reset Verlet history so the particle starts from rest at its current
    // position (prevPosition == position ⇒ zero implicit velocity). Call this
    // when you place a particle before running the Verlet path.
    void syncVerlet(const Vec3& initialVelocity = Vec3{}, Real dt = Real(1) / 120) {
        prevPosition = position - initialVelocity * dt;
    }

    // Convenience for the force-based path: kinetic energy ½ m v².
    Real kineticEnergy() const {
        if (isStatic()) return 0;
        return Real(0.5) * mass() * lengthSq(velocity);
    }
};

}  // namespace p3d
