// ===========================================================================
//  sat.hpp — the Separating-Axis Theorem for oriented boxes
// ---------------------------------------------------------------------------
//  Introduced in: Chapter 14 (Narrowphase II: SAT & Manifolds)
//
//  The Separating-Axis Theorem says: two convex shapes are apart if and only if
//  there is some axis along which their projected shadows don't overlap. For two
//  boxes there are only 15 axes worth checking (3 face directions each, plus 9
//  edge-cross-edge directions). If ALL 15 shadows overlap, the boxes intersect,
//  and the axis with the SMALLEST overlap is the direction of shallowest
//  penetration — the contact normal.
//
//  Finding the normal is only half the job. To resolve a resting box stably we
//  need a MANIFOLD — several contact points, not one — so this also clips the
//  two touching faces together to produce up to four points.
// ===========================================================================
#pragma once

#include "contact.hpp"
#include "../common/math/quat.hpp"

namespace p3d {

// Oriented Box A vs Oriented Box B. Writes up to `maxOut` contacts (normal from
// B to A, per contact.hpp) and returns how many. Returns 0 if they don't touch.
int collideBoxBox(const Vec3& posA, const Quat& rotA, const Vec3& halfA,
                  const Vec3& posB, const Quat& rotB, const Vec3& halfB,
                  Contact out[], int maxOut);

}  // namespace p3d
