// ===========================================================================
//  gjk.hpp — GJK intersection + EPA penetration for any convex shapes
// ---------------------------------------------------------------------------
//  Introduced in: Chapter 15 (GJK & EPA)
//
//  SAT (Chapter 14) is great for boxes, but its axis list is shape-specific. The
//  Gilbert–Johnson–Keerthi algorithm (GJK) is the general answer: it decides
//  whether ANY two convex shapes intersect, using nothing but a "support
//  function" — give it a direction, it returns the shape's farthest point that
//  way. Everything else is geometry on the MINKOWSKI DIFFERENCE (the set of all
//  differences a−b): two shapes overlap exactly when that difference contains the
//  origin, and GJK hunts for the origin with a growing simplex (a point, then a
//  line, triangle, tetrahedron).
//
//  GJK only answers yes/no. To get the penetration depth and direction (what the
//  solver needs), EPA — the Expanding Polytope Algorithm — takes over where GJK
//  finished, growing a polytope out to the surface of the Minkowski difference to
//  find the point on it closest to the origin.
//
//  Shapes here are just lists of world-space vertices (any convex hull). The
//  support function is then "the vertex farthest along the direction".
// ===========================================================================
#pragma once

#include "../common/math/vec3.hpp"
#include <vector>

namespace p3d {

// Yes/no: do these two convex point sets overlap? (GJK only.)
bool gjkIntersect(const std::vector<Vec3>& shapeA, const std::vector<Vec3>& shapeB);

struct Penetration {
    bool  hit{false};
    Vec3  normal{};    // unit; points from B toward A (matches Contact)
    Real  depth{0};    // how far they overlap along `normal`
    Vec3  point{};     // a representative contact point in world space
};

// Full test: GJK to decide overlap, then EPA to recover normal + depth.
Penetration gjkEpa(const std::vector<Vec3>& shapeA, const std::vector<Vec3>& shapeB);

}  // namespace p3d
