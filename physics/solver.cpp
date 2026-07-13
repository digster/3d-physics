// ===========================================================================
//  solver.cpp — preparing, warm-starting, and iterating contact impulses
// ===========================================================================
#include "solver.hpp"

#include <cmath>

namespace p3d {

// Effective mass of a constraint along unit direction `d`: how much a unit
// impulse there actually changes the relative velocity, accounting for BOTH
// bodies' linear mass and their rotational resistance (r × d fed through the
// inverse inertia). Small effective mass = hard to move; we invert it so the
// solver multiplies rather than divides.
static Real effectiveMass(Real invMassA, Real invMassB,
                          const Mat3& invIA, const Mat3& invIB,
                          const Vec3& rA, const Vec3& rB, const Vec3& d) {
    const Vec3 rnA = cross(rA, d);
    const Vec3 rnB = cross(rB, d);
    const Real k = invMassA + invMassB +
                   dot(d, cross(invIA * rnA, rA)) +
                   dot(d, cross(invIB * rnB, rB));
    return (k > kEpsilon) ? Real(1) / k : Real(0);
}

void prepareContacts(std::vector<ContactConstraint>& cs, std::vector<RigidBody>& bodies,
                     const SolverConfig& cfg, Real dt) {
    for (ContactConstraint& c : cs) {
        RigidBody& A = bodies[c.a];
        RigidBody& B = bodies[c.b];
        const Mat3 invIA = A.invInertiaWorld();
        const Mat3 invIB = B.invInertiaWorld();

        c.rA = c.point - A.position;
        c.rB = c.point - B.position;

        // Two tangent directions spanning the plane the friction acts in.
        const Vec3 ref = (std::fabs(c.normal.x) < 0.9f) ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
        c.t1 = normalize(cross(c.normal, ref));
        c.t2 = cross(c.normal, c.t1);

        c.normalMass = effectiveMass(A.invMass, B.invMass, invIA, invIB, c.rA, c.rB, c.normal);
        c.t1Mass     = effectiveMass(A.invMass, B.invMass, invIA, invIB, c.rA, c.rB, c.t1);
        c.t2Mass     = effectiveMass(A.invMass, B.invMass, invIA, invIB, c.rA, c.rB, c.t2);

        // Restitution: bounce back a fraction of the closing speed, but only if
        // the bodies are actually approaching fast enough — otherwise resting
        // contacts would jitter with tiny phantom bounces.
        const Vec3 vrel = A.velocityAtPoint(c.point) - B.velocityAtPoint(c.point);
        const Real vn = dot(vrel, c.normal);
        Real restitutionBias = 0;
        if (vn < -cfg.restitutionThreshold) restitutionBias = -c.restitution * vn;

        // Baumgarte: gently feed a bit of the leftover penetration back as a
        // separating velocity, so overlaps are pushed out over a few frames.
        const Real correction = std::max(Real(0), c.penetration - cfg.slop);
        const Real baumgarteBias = (cfg.baumgarte / dt) * correction;

        // Take the LARGER of the two, not their sum. Summing lets position
        // correction pile on top of a real bounce and inject energy (a ball
        // would rebound higher than it fell); the max keeps restitution honest
        // while still pushing deep overlaps apart.
        c.velocityBias = std::max(restitutionBias, baumgarteBias);
    }
}

void warmStartContacts(std::vector<ContactConstraint>& cs, std::vector<RigidBody>& bodies) {
    for (ContactConstraint& c : cs) {
        // Re-apply the total impulse this contact ended with last time. Because
        // stacks change little frame-to-frame, this is an excellent first guess.
        const Vec3 P = c.normal * c.normalImpulse +
                       c.t1 * c.tangent1Impulse +
                       c.t2 * c.tangent2Impulse;
        bodies[c.a].applyImpulse(P, c.point);
        bodies[c.b].applyImpulse(-P, c.point);
    }
}

void solveVelocities(std::vector<ContactConstraint>& cs, std::vector<RigidBody>& bodies) {
    for (ContactConstraint& c : cs) {
        RigidBody& A = bodies[c.a];
        RigidBody& B = bodies[c.b];

        // --- Normal impulse: stop approaching, add the restitution bounce -----
        {
            const Vec3 vrel = A.velocityAtPoint(c.point) - B.velocityAtPoint(c.point);
            const Real vn = dot(vrel, c.normal);
            Real dLambda = (c.velocityBias - vn) * c.normalMass;
            // The accumulated normal impulse can only PUSH (>= 0) — a contact
            // never pulls bodies together. We clamp the total, not the delta.
            const Real oldImpulse = c.normalImpulse;
            c.normalImpulse = std::max(Real(0), oldImpulse + dLambda);
            dLambda = c.normalImpulse - oldImpulse;
            const Vec3 P = c.normal * dLambda;
            A.applyImpulse(P, c.point);
            B.applyImpulse(-P, c.point);
        }

        // --- Friction: oppose sliding, but clamped to the Coulomb cone --------
        // The friction impulse can be at most μ times the normal impulse; beyond
        // that the surfaces slip. We solve each tangent direction and clamp.
        const Real maxFriction = c.friction * c.normalImpulse;
        auto solveFriction = [&](const Vec3& t, Real tMass, Real& accum) {
            const Vec3 vrel = A.velocityAtPoint(c.point) - B.velocityAtPoint(c.point);
            const Real vt = dot(vrel, t);
            Real dLambda = -vt * tMass;
            const Real oldImpulse = accum;
            accum = clamp(oldImpulse + dLambda, -maxFriction, maxFriction);
            dLambda = accum - oldImpulse;
            const Vec3 P = t * dLambda;
            A.applyImpulse(P, c.point);
            B.applyImpulse(-P, c.point);
        };
        solveFriction(c.t1, c.t1Mass, c.tangent1Impulse);
        solveFriction(c.t2, c.t2Mass, c.tangent2Impulse);
    }
}

}  // namespace p3d
