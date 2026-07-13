// ===========================================================================
//  collider.hpp — a shape + material attached to a rigid body
// ---------------------------------------------------------------------------
//  Introduced in: Chapter 16 (Impulses & Restitution)
//
//  Up to now the narrowphase tests took loose shape parameters. To assemble a
//  real world we bundle a body's shape (a sphere or a box) with its surface
//  MATERIAL — how bouncy (restitution) and how grippy (friction) it is. The
//  World (world.hpp) pairs one Collider with each RigidBody and uses it to build
//  bounding boxes, pick the right narrowphase test, and set up the inertia tensor.
// ===========================================================================
#pragma once

#include "aabb.hpp"
#include "inertia.hpp"
#include "../common/math/quat.hpp"

#include <algorithm>
#include <cmath>

namespace p3d {

struct Collider {
    enum class Type { Sphere, Box };
    Type type{Type::Sphere};
    Real radius{0.5f};              // used when type == Sphere
    Vec3 half{0.5f, 0.5f, 0.5f};    // used when type == Box (half-extents)

    // Material. restitution = bounciness (0 = dead, 1 = perfectly elastic);
    // friction = the Coulomb coefficient (0 = ice, ~1 = rubber).
    Real restitution{0.2f};
    Real friction{0.5f};

    static Collider sphere(Real r, Real rest = 0.2f, Real fric = 0.5f) {
        Collider c; c.type = Type::Sphere; c.radius = r; c.restitution = rest; c.friction = fric; return c;
    }
    static Collider box(const Vec3& h, Real rest = 0.2f, Real fric = 0.5f) {
        Collider c; c.type = Type::Box; c.half = h; c.restitution = rest; c.friction = fric; return c;
    }

    // The inertia tensor this shape would have at a given mass (for RigidBody).
    Mat3 inertiaTensor(Real mass) const {
        return (type == Type::Sphere) ? inertia::sphere(mass, radius)
                                      : inertia::box(mass, half);
    }

    // The world-space AABB, for broadphase.
    AABB aabb(const Vec3& pos, const Quat& rot) const {
        return (type == Type::Sphere) ? aabbOfSphere(pos, radius)
                                      : aabbOfBox(pos, rot, half);
    }
};

// How two materials combine at a contact. Restitution takes the max (any bouncy
// surface makes the pair bounce); friction is the geometric mean (a common,
// well-behaved choice). Both are conventions — engines differ.
inline Real combineRestitution(Real a, Real b) { return std::max(a, b); }
inline Real combineFriction(Real a, Real b)    { return std::sqrt(a * b); }

}  // namespace p3d
