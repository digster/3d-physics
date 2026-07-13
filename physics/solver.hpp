// ===========================================================================
//  solver.hpp — the sequential-impulse contact solver
// ---------------------------------------------------------------------------
//  Introduced in: Chapter 16 (normal + restitution), grown through Chapter 17
//                 (friction, warm starting, position correction)
//
//  A Contact from Part IV says "these two overlap, here, this way, this deep."
//  The solver's job is to change the bodies' velocities so they stop
//  interpenetrating and bounce/slide correctly. It does this with IMPULSES — an
//  impulse is a force already integrated over the step, so it can change a
//  velocity instantly (a wall can stop a ball in one frame; a force couldn't).
//
//  "Sequential impulse" means: rather than solve all contacts together (a big
//  matrix), fix each contact's velocity one at a time and loop a few times. The
//  fixes ripple through a stack and converge — it is Gauss–Seidel iteration, and
//  it is what lets cheap per-contact math hold up a pyramid of boxes.
// ===========================================================================
#pragma once

#include "contact.hpp"
#include "rigidbody.hpp"
#include <vector>

namespace p3d {

// A Contact, enriched with everything the solver needs and the accumulated
// impulses it carries between iterations (and, via warm starting, frames).
struct ContactConstraint {
    int  a{-1}, b{-1};      // body indices
    Vec3 point{};           // world contact point
    Vec3 normal{};          // unit, B → A (per contact.hpp)
    Real penetration{0};
    Real restitution{0};
    Real friction{0};
    // Persisted accumulated impulses (kept across frames for warm starting).
    Real normalImpulse{0}, tangent1Impulse{0}, tangent2Impulse{0};

    // --- Filled by prepare() ---------------------------------------------
    Vec3 rA{}, rB{};        // contact point relative to each centre of mass
    Vec3 t1{}, t2{};        // two tangent directions (the friction plane)
    Real normalMass{0}, t1Mass{0}, t2Mass{0};   // effective masses
    Real velocityBias{0};   // target normal velocity (restitution + Baumgarte)
};

struct SolverConfig {
    int  iterations{10};             // velocity-solve passes per step
    Real baumgarte{0.2f};            // position-correction stiffness (0..1)
    Real slop{0.005f};               // penetration allowed before we correct
    Real restitutionThreshold{0.5f}; // min closing speed (m/s) that bounces
    bool warmStart{true};
};

// Compute the effective masses, tangent basis, and restitution/Baumgarte bias.
void prepareContacts(std::vector<ContactConstraint>& cs, std::vector<RigidBody>& bodies,
                     const SolverConfig& cfg, Real dt);

// Re-apply last frame's accumulated impulses as a warm start (a great initial
// guess that dramatically speeds convergence and steadies stacks).
void warmStartContacts(std::vector<ContactConstraint>& cs, std::vector<RigidBody>& bodies);

// One velocity-solve pass: for each contact, fix the normal velocity (bounce),
// then the two friction directions (clamped to the Coulomb cone). Call it
// `cfg.iterations` times.
void solveVelocities(std::vector<ContactConstraint>& cs, std::vector<RigidBody>& bodies);

}  // namespace p3d
