// ===========================================================================
//  narrowphase.hpp — the exact shape-vs-shape tests that produce contacts
// ---------------------------------------------------------------------------
//  Introduced in: Chapter 13 (Narrowphase I: Spheres & Planes)
//
//  Where broadphase asked "might these touch?", narrowphase asks "do they really
//  touch, and exactly where and how deep?" — and answers with a Contact. Each
//  function here handles one pair of shape types. They are the exact, expensive
//  tests we run only on the candidate pairs broadphase hands us.
//
//  Every contact follows the convention in contact.hpp: `normal` points FROM B
//  TOWARD A (the direction to push A to separate the pair).
// ===========================================================================
#pragma once

#include "contact.hpp"
#include "../common/math/quat.hpp"

namespace p3d {

// Sphere A vs Sphere B. The classic: they touch when the distance between
// centres is less than the sum of the radii.
bool collideSphereSphere(const Vec3& ca, Real ra, const Vec3& cb, Real rb, Contact& out);

// Sphere A vs an infinite Plane B. The plane is { x : dot(n, x) = offset } with
// `n` a unit normal; the "outside/keep" side is the +n side.
bool collideSpherePlane(const Vec3& c, Real r, const Vec3& n, Real offset, Contact& out);

// Sphere A vs an oriented Box B. Works by finding the closest point on the box
// to the sphere centre; a sphere whose centre is inside the box is pushed out of
// the nearest face.
bool collideSphereBox(const Vec3& c, Real r,
                      const Vec3& boxPos, const Quat& boxRot, const Vec3& half,
                      Contact& out);

// Oriented Box A vs an infinite Plane B. A box can rest on up to four corners,
// so this appends up to four contacts to `out` and returns how many.
int collideBoxPlane(const Vec3& boxPos, const Quat& boxRot, const Vec3& half,
                    const Vec3& n, Real offset, Contact out[4]);

}  // namespace p3d
