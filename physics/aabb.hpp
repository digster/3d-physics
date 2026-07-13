// ===========================================================================
//  aabb.hpp — axis-aligned bounding boxes, the workhorse of broadphase
// ---------------------------------------------------------------------------
//  Introduced in: Chapter 12 (Broadphase)
//
//  An AABB is the simplest useful bound: the smallest box, aligned to the world
//  axes, that fully contains a shape. It is defined by just two corners — the
//  minimum and maximum. Two AABBs overlap only if they overlap on ALL THREE axes
//  at once, which is three cheap comparisons per axis. That cheapness is the
//  whole point: we test loose AABBs first to quickly rule out pairs that can't
//  possibly touch, and only run the expensive exact tests on what survives.
// ===========================================================================
#pragma once

#include "../common/math/vec3.hpp"
#include "../common/math/quat.hpp"

namespace p3d {

struct AABB {
    Vec3 min{};
    Vec3 max{};

    static AABB fromCenterHalf(const Vec3& c, const Vec3& half) {
        return {c - half, c + half};
    }

    Vec3 center() const { return (min + max) * Real(0.5); }
    Vec3 extent() const { return (max - min) * Real(0.5); }   // half-widths

    // Do two AABBs overlap? They must overlap on every axis simultaneously. A
    // small `margin` inflates the test — broadphase uses a margin so a pair is
    // flagged just BEFORE they touch, giving the solver a frame of warning.
    bool overlaps(const AABB& o, Real margin = 0) const {
        return (min.x - margin <= o.max.x && max.x + margin >= o.min.x) &&
               (min.y - margin <= o.max.y && max.y + margin >= o.min.y) &&
               (min.z - margin <= o.max.z && max.z + margin >= o.min.z);
    }

    // Grow to also contain another AABB (used to build the domain bounds).
    void merge(const AABB& o) { min = minv(min, o.min); max = maxv(max, o.max); }

    // Surface area — the cost metric fancier broadphases (BVHs) minimise.
    Real surfaceArea() const {
        const Vec3 d = max - min;
        return Real(2) * (d.x * d.y + d.y * d.z + d.z * d.x);
    }
};

// The world-space AABB of a sphere is trivial: centre ± radius on each axis.
inline AABB aabbOfSphere(const Vec3& c, Real r) {
    return {c - Vec3(r), c + Vec3(r)};
}

// The world-space AABB of an ORIENTED box (OBB). We can't just use the half
// extents, because a rotated box pokes out further. The neat trick: the extent
// of the world AABB along an axis is the sum of the box's half-widths projected
// onto that axis — i.e. the absolute value of the rotation matrix times the
// half extents. (Absolute value taken component-wise.)
inline AABB aabbOfBox(const Vec3& pos, const Quat& rot, const Vec3& halfExtents) {
    const Mat3 R = toMat3(rot);
    Vec3 worldHalf;
    for (int axis = 0; axis < 3; ++axis) {
        worldHalf[axis] = std::fabs(R.at(axis, 0)) * halfExtents.x +
                          std::fabs(R.at(axis, 1)) * halfExtents.y +
                          std::fabs(R.at(axis, 2)) * halfExtents.z;
    }
    return AABB::fromCenterHalf(pos, worldHalf);
}

}  // namespace p3d
