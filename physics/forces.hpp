// ===========================================================================
//  forces.hpp — force generators: the things that push on particles
// ---------------------------------------------------------------------------
//  Introduced in: Chapter 6 (gravity, drag) and Chapter 7 (springs)
//
//  A "force generator" is just a function that looks at a particle (or a pair)
//  and adds a force to its accumulator. Keeping each force in its own small
//  function is the whole design pattern: to build a scene you pick which forces
//  act on which particles each step, sum them, then integrate once. Gravity,
//  drag, and springs below are all we need for Part II's force-based demos.
//
//  Every function here only ADDS to Particle::forceAccum — it never integrates
//  and never clears. The order you call them in does not matter, because forces
//  simply add up (the superposition principle).
// ===========================================================================
#pragma once

#include "particle.hpp"

namespace p3d::force {

// Gravity. Note the force is mass × g, so that when the integrator divides by
// mass the acceleration is exactly g — i.e. everything falls at the same rate,
// independent of mass (Galileo's observation, and a good test case). `g` is an
// acceleration, e.g. {0, -9.81, 0}.
void gravity(Particle& p, const Vec3& g);

// Aerodynamic drag, opposing the direction of motion. Uses the standard model
//     F = -v̂ · (k1·|v| + k2·|v|²)
// The linear term (k1) dominates at low speed, the quadratic term (k2) at high
// speed. Drag is what makes a fountain's droplets arc realistically and settle
// instead of oscillating forever.
void drag(Particle& p, Real k1, Real k2);

// A damped spring between two particles (Hooke's law + a damper).
//   * k    : stiffness — how hard it pulls back per unit of stretch.
//   * rest : the natural length at which the spring is relaxed.
//   * c    : damping — resists the rate of stretch, draining energy so the
//            oscillation dies down (Chapter 7 explores under/critical/over-damping).
// The force is applied equal-and-opposite to the two particles, so a spring
// never changes the pair's total momentum.
void spring(Particle& a, Particle& b, Real k, Real rest, Real c = 0);

// The same spring, but with one end nailed to a fixed point in space. Only the
// particle moves. Used for spring pendulums hanging from a ceiling.
void anchoredSpring(Particle& p, const Vec3& anchor, Real k, Real rest, Real c = 0);

// A bungee: like a spring, but it only ever PULLS (it goes slack when
// compressed). Useful for ropes-as-forces and for keeping things from flying
// apart without resisting them coming together.
void bungee(Particle& a, Particle& b, Real k, Real rest);

}  // namespace p3d::force
