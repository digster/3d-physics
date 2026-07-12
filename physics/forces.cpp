// ===========================================================================
//  forces.cpp — implementations of the force generators
// ===========================================================================
#include "forces.hpp"

namespace p3d::force {

void gravity(Particle& p, const Vec3& g) {
    if (p.isStatic()) return;               // infinite mass → gravity can't move it
    p.addForce(g * p.mass());               // F = m·g
}

void drag(Particle& p, Real k1, Real k2) {
    const Real speed = length(p.velocity);
    if (speed < kEpsilon) return;           // no motion → no drag (avoid /0)
    const Vec3 dir = p.velocity * (Real(1) / speed);   // unit velocity
    const Real magnitude = k1 * speed + k2 * speed * speed;
    p.addForce(dir * (-magnitude));         // oppose the motion
}

// Shared core for the two-particle and anchored springs. `va`/`vb` are the two
// endpoints' velocities (the anchor's velocity is zero). Returns the force to
// apply to endpoint A; the caller applies its negation to endpoint B if B moves.
static Vec3 springForce(const Vec3& pa, const Vec3& pb, const Vec3& va, const Vec3& vb,
                        Real k, Real rest, Real c) {
    Vec3 delta = pa - pb;                   // vector from B to A
    const Real dist = length(delta);
    if (dist < kEpsilon) return Vec3{};     // coincident → direction undefined
    const Vec3 dir = delta * (Real(1) / dist);

    // Hooke: restoring force is proportional to how far we are from `rest`.
    const Real stretch = dist - rest;
    Real magnitude = -k * stretch;

    // Damping: resist the speed at which the spring is lengthening/shortening,
    // i.e. the relative velocity projected onto the spring's axis.
    if (c != 0) {
        const Real closingSpeed = dot(va - vb, dir);
        magnitude -= c * closingSpeed;
    }
    return dir * magnitude;                 // points from B toward A when positive
}

void spring(Particle& a, Particle& b, Real k, Real rest, Real c) {
    const Vec3 f = springForce(a.position, b.position, a.velocity, b.velocity, k, rest, c);
    a.addForce(f);
    b.addForce(-f);                         // equal and opposite: momentum conserved
}

void anchoredSpring(Particle& p, const Vec3& anchor, Real k, Real rest, Real c) {
    // The anchor is a fixed point with zero velocity; only p receives a force.
    const Vec3 f = springForce(p.position, anchor, p.velocity, Vec3{}, k, rest, c);
    p.addForce(f);
}

void bungee(Particle& a, Particle& b, Real k, Real rest) {
    Vec3 delta = a.position - b.position;
    const Real dist = length(delta);
    if (dist <= rest || dist < kEpsilon) return;   // slack (or coincident) → no force
    const Vec3 dir = delta * (Real(1) / dist);
    const Vec3 f = dir * (-k * (dist - rest));      // only pulls, never pushes
    a.addForce(f);
    b.addForce(-f);
}

}  // namespace p3d::force
