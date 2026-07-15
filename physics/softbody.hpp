// ===========================================================================
//  softbody.hpp — XPBD soft bodies: things that squish and spring back
// ---------------------------------------------------------------------------
//  Introduced in: Chapter 20 (XPBD & Soft Bodies)
//
//  The finale closes the loop back to Part II. A soft body is, once again, a
//  lattice of particles held together by distance constraints — but now the
//  constraints are COMPLIANT (springy), using XPBD (Extended Position-Based
//  Dynamics). Chapter 8's Jakobsen constraint was infinitely stiff and its
//  apparent softness leaked out of the iteration count; XPBD adds a single term,
//  the COMPLIANCE α (inverse stiffness), and accumulates a Lagrange multiplier λ
//  per constraint. The upshot: a real, tunable stiffness that does not depend on
//  the timestep or the number of iterations. α = 0 is perfectly rigid; larger α
//  is softer jello.
//
//  XPBD works in SUBSTEPS: split the frame into several tiny steps, each doing a
//  predict → solve-once → set-velocity pass. Many small steps beat few big ones
//  for stability, and it's what makes the jello wobble instead of blowing up.
// ===========================================================================
#pragma once

#include "particle.hpp"
#include "../common/render/mesh.hpp"
#include <vector>

namespace p3d {

class SoftBody {
public:
    // A compliant distance constraint. `compliance` (α) is inverse stiffness:
    // 0 = rigid, bigger = softer. `lambda` is the accumulated XPBD multiplier,
    // reset at the start of each substep.
    struct Spring { int a, b; Real restLength; Real compliance; Real lambda; };

    std::vector<Particle> particles;
    std::vector<Spring>   springs;
    Mesh surfaceMesh;                 // rebuilt from particle positions each frame

    // Build a solid cube lattice of n×n×n particles centred at `center`, with
    // structural / shear / volume constraints woven through it.
    void buildCube(const Vec3& center, Real size, int n, Real compliance, Real totalMass);

    // Advance the soft body by dt using `substeps` XPBD substeps under `gravity`,
    // colliding with a floor plane at height `groundY` (with a little friction).
    void step(Real dt, int substeps, const Vec3& gravity, Real groundY);

    // Copy the current particle positions into surfaceMesh for drawing.
    void updateMesh();

    // Pin a particle in place (invMass 0) — e.g. to hang the jello from a ceiling.
    void pin(int index) { particles[index].makeStatic(); }

    int nx{0}, ny{0}, nz{0};          // lattice dimensions
    int index(int i, int j, int k) const { return (i * ny + j) * nz + k; }

private:
    void addSpring(int a, int b, Real compliance);
    void buildSurface();
};

}  // namespace p3d
