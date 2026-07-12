// ===========================================================================
//  mesh.hpp — triangle meshes and generators for the basic shapes
// ---------------------------------------------------------------------------
//  Introduced in: Chapter 4 (Camera & Projection)
//
//  A Mesh is the simplest thing that can be: a bag of vertex POSITIONS plus a
//  list of INDICES that group those positions into triangles (three indices
//  per triangle). We store no per-vertex normals — the renderer computes a
//  single flat normal per triangle from its corners, which gives the clean
//  faceted look the physics demos use and keeps the data minimal.
//
//  The generators below build the handful of primitives the whole tutorial
//  reuses: boxes for rigid bodies, spheres for balls, planes for ground, and
//  cylinders for the occasional pillar or capsule stand-in.
// ===========================================================================
#pragma once

#include "../math/vec3.hpp"
#include <vector>

namespace p3d {

struct Mesh {
    std::vector<Vec3> positions;   // vertex positions in local (model) space
    std::vector<int>  indices;     // 3 per triangle, indexing into positions

    int triangleCount() const { return static_cast<int>(indices.size()) / 3; }
};

namespace mesh {

// An axis-aligned box centred on the origin, spanning [-h, +h] on each axis.
// 12 triangles (2 per face). Because faces are flat-shaded, each of the 6
// faces gets its own 4 vertices so a face normal is well defined.
Mesh box(const Vec3& halfExtents);

// A UV sphere of the given radius. `rings` are the horizontal bands
// (latitude), `sectors` the vertical slices (longitude). More = smoother but
// more triangles; the demos keep these modest so hundreds of balls stay fast.
Mesh sphere(Real radius, int rings = 12, int sectors = 18);

// A flat square in the XZ plane (y = 0), centred on the origin, `size` on a
// side, split into subdiv×subdiv cells (subdivision helps depth sorting and
// makes the surface catch light more evenly).
Mesh plane(Real size, int subdiv = 1);

// A cylinder aligned to the Y axis, centred on the origin, with flat caps.
Mesh cylinder(Real radius, Real height, int sectors = 20);

}  // namespace mesh
}  // namespace p3d
