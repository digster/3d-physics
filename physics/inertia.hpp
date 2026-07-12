// ===========================================================================
//  inertia.hpp — inertia tensors for the primitive shapes
// ---------------------------------------------------------------------------
//  Introduced in: Chapter 11 (Rigid-Body Dynamics)
//
//  The INERTIA TENSOR is rotation's answer to mass. Mass says "how hard is this
//  to get moving in a straight line"; the inertia tensor says "how hard is this
//  to get SPINNING — about each axis." It's a 3x3 matrix because a body can be
//  easy to spin one way and hard another (think of a pencil: trivial to spin
//  about its length, harder end-over-end).
//
//  For a symmetric primitive aligned with its own axes, the tensor is DIAGONAL —
//  the three numbers are the moments of inertia about x, y, z. The formulas
//  below are the standard textbook results for solid bodies; we derive the box
//  case in the chapter and quote the rest.
//
//  We build these in BODY space (the body's own frame). Turning them into world
//  space as the body rotates is the rigid body's job (see rigidbody.hpp).
// ===========================================================================
#pragma once

#include "../common/math/mat3.hpp"
#include "../common/math/vec3.hpp"

namespace p3d::inertia {

// Solid box. `halfExtents` matches mesh::box (half the width on each axis).
// Moment about an axis depends on the spread PERPENDICULAR to it, hence each
// diagonal entry mixes the other two dimensions.
inline Mat3 box(Real mass, const Vec3& halfExtents) {
    const Real hx = halfExtents.x, hy = halfExtents.y, hz = halfExtents.z;
    const Real k = mass / Real(3);                 // (m/12)·(2h)² collapses to m/3·h²
    return Mat3::diagonal({
        k * (hy*hy + hz*hz),
        k * (hx*hx + hz*hz),
        k * (hx*hx + hy*hy),
    });
}

// Solid sphere: the same about every axis (perfectly symmetric).
inline Mat3 sphere(Real mass, Real radius) {
    const Real i = Real(0.4) * mass * radius * radius;   // 2/5 m r²
    return Mat3::diagonal({i, i, i});
}

// Solid cylinder aligned to the Y axis (matches mesh::cylinder).
inline Mat3 cylinder(Real mass, Real radius, Real height) {
    const Real iy   = Real(0.5) * mass * radius * radius;                 // about the axis
    const Real ixz  = mass * (Real(3) * radius * radius + height * height) / Real(12);
    return Mat3::diagonal({ixz, iy, ixz});
}

// Parallel-axis theorem: shift an inertia tensor from a body's own centre of
// mass to a point offset by `d`. Adding several shifted tensors lets us build
// the inertia of a COMPOSITE body (e.g. a T-handle = shaft + crossbar) about
// the whole thing's centre of mass.
//     I_shifted = I_com + m·( (d·d)·E − d⊗d )
inline Mat3 shift(const Mat3& iCom, Real mass, const Vec3& d) {
    const Real dd = dot(d, d);
    Mat3 r = iCom;
    r.at(0,0) += mass * (dd - d.x*d.x); r.at(0,1) += mass * (-d.x*d.y); r.at(0,2) += mass * (-d.x*d.z);
    r.at(1,0) += mass * (-d.y*d.x);     r.at(1,1) += mass * (dd - d.y*d.y); r.at(1,2) += mass * (-d.y*d.z);
    r.at(2,0) += mass * (-d.z*d.x);     r.at(2,1) += mass * (-d.z*d.y); r.at(2,2) += mass * (dd - d.z*d.z);
    return r;
}

}  // namespace p3d::inertia
