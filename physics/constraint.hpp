// ===========================================================================
//  constraint.hpp — distance constraints solved by position projection
// ---------------------------------------------------------------------------
//  Introduced in: Chapter 8 (Ropes & Constraints), reused in Chapter 9 (Cloth)
//
//  This is the POSITION-BASED half of Part II. Instead of computing a spring
//  force and hoping the integrator keeps two particles the right distance
//  apart, we integrate freely (Verlet) and then simply MOVE the particles so
//  the distance is exactly right. That "just move them" step is Thomas
//  Jakobsen's method (of Hitman fame), and it is wonderfully stable: a rope
//  made of distance constraints cannot stretch explosively the way a stiff
//  spring can, because we never let it stretch in the first place.
//
//  The catch: moving one pair to satisfy its constraint can violate a
//  neighbour's. So we solve the whole list REPEATEDLY — a few relaxation
//  iterations per step — and the positions converge. Chapter 8 visualises that
//  convergence; more iterations ⇒ a stiffer, less stretchy rope.
// ===========================================================================
#pragma once

#include "particle.hpp"
#include <vector>

namespace p3d {

// Keep two particles (referenced by index into a particle array) exactly
// `restLength` apart. `stiffness` in [0,1] scales how much of the needed
// correction is applied each pass — 1 is a rigid link, lower values give a
// springier, stretchier feel while staying stable.
struct DistanceConstraint {
    int  a{0};
    int  b{0};
    Real restLength{1};
    Real stiffness{1};

    // Move a and b along the line between them so their separation becomes
    // restLength. The move is split between them in proportion to inverse mass,
    // so a light bead moves a lot and a heavy (or static) one barely budges.
    void solve(std::vector<Particle>& particles) const {
        Particle& pa = particles[a];
        Particle& pb = particles[b];

        Vec3 delta = pb.position - pa.position;   // from a to b
        const Real dist = length(delta);
        if (dist < kEpsilon) return;              // coincident → no defined direction

        const Real wsum = pa.invMass + pb.invMass;
        if (wsum == 0) return;                    // both immovable → nothing to do

        // Fractional error: >0 too far apart (pull together), <0 too close (push apart).
        const Real diff = (dist - restLength) / dist;
        const Vec3  correction = delta * (stiffness * diff / wsum);

        // Each particle gets a share of the correction weighted by its invMass.
        pa.position += correction * pa.invMass;   // toward b if too far
        pb.position -= correction * pb.invMass;   // toward a if too far
    }
};

// Run `iterations` relaxation passes over the whole constraint list. Because
// each pass nudges positions closer to satisfying every constraint, more passes
// mean a stiffer result. Static particles (invMass 0) act as immovable anchors.
inline void satisfyConstraints(const std::vector<DistanceConstraint>& constraints,
                               std::vector<Particle>& particles, int iterations) {
    for (int it = 0; it < iterations; ++it)
        for (const DistanceConstraint& c : constraints)
            c.solve(particles);
}

}  // namespace p3d
