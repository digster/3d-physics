// ===========================================================================
//  debug_draw.hpp — immediate-mode helpers for VISUALIZING the physics
// ---------------------------------------------------------------------------
//  Introduced in: Chapter 4, used heavily from Chapter 6 onward
//
//  Physics is invisible: forces, velocities, contact normals, bounding boxes,
//  and axes are all quantities you compute but never see. These helpers draw
//  them as lines and arrows so you can watch the simulation reason. They all
//  build on Renderer3D::drawLine, so they respect the same camera and near
//  clipping as the solid geometry.
// ===========================================================================
#pragma once

#include "../math/vec3.hpp"
#include "color.hpp"
#include "renderer3d.hpp"

namespace p3d::debug {

// A ground grid on the XZ plane (y = 0): `divisions` cells each way spanning
// ±extent. The reference floor for almost every demo.
void grid(Renderer3D& r, Real extent = 10, int divisions = 20,
          const Color& color = Color::bytes(80, 86, 100));

// A right-handed coordinate gizmo at `origin`: X red, Y green, Z blue. The
// quickest way to stay oriented while the camera swings around.
void axes(Renderer3D& r, const Vec3& origin = Vec3{}, Real length = 1);

// The 12 edges of an axis-aligned bounding box given its min/max corners.
// This is what broadphase (Chapter 12) reasons about.
void aabb(Renderer3D& r, const Vec3& mn, const Vec3& mx,
          const Color& color = palette::amber);

// An arrow from `from` to `to`, with a small arrowhead. Use it to draw a
// velocity, a force, or a contact normal — anything with a direction.
void arrow(Renderer3D& r, const Vec3& from, const Vec3& to,
           const Color& color = palette::teal);

// A little 3D cross marking a single point (e.g. a contact point).
void point(Renderer3D& r, const Vec3& p, Real size = 0.06f,
           const Color& color = palette::coral);

}  // namespace p3d::debug
