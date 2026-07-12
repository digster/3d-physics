// ===========================================================================
//  debug_draw.cpp — implementations of the debug visualization helpers
// ===========================================================================
#include "debug_draw.hpp"

#include "../math/mat4.hpp"
#include <cmath>

namespace p3d::debug {

void grid(Renderer3D& r, Real extent, int divisions, const Color& color) {
    if (divisions < 1) divisions = 1;
    const Real step = (Real(2) * extent) / Real(divisions);
    // Emphasise the two center lines (the X and Z axes) with a brighter tint.
    const Color axisTint = Color::bytes(120, 128, 145);
    for (int i = 0; i <= divisions; ++i) {
        const Real p = -extent + step * Real(i);
        const bool center = std::fabs(p) < 1e-4f;
        const Color& c = center ? axisTint : color;
        r.drawLine({p, 0, -extent}, {p, 0, extent}, c);  // lines parallel to Z
        r.drawLine({-extent, 0, p}, {extent, 0, p}, c);  // lines parallel to X
    }
}

void axes(Renderer3D& r, const Vec3& origin, Real length) {
    r.drawLine(origin, origin + axis::X * length, palette::coral);   // X = red
    r.drawLine(origin, origin + axis::Y * length, palette::lime);    // Y = green
    r.drawLine(origin, origin + axis::Z * length, palette::sky);     // Z = blue
}

void aabb(Renderer3D& r, const Vec3& mn, const Vec3& mx, const Color& color) {
    // The eight corners, then the twelve edges that connect them.
    const Vec3 c[8] = {
        {mn.x, mn.y, mn.z}, {mx.x, mn.y, mn.z}, {mx.x, mx.y, mn.z}, {mn.x, mx.y, mn.z},
        {mn.x, mn.y, mx.z}, {mx.x, mn.y, mx.z}, {mx.x, mx.y, mx.z}, {mn.x, mx.y, mx.z},
    };
    const int edges[12][2] = {
        {0,1},{1,2},{2,3},{3,0},   // bottom face (-z)... actually the mn.z ring
        {4,5},{5,6},{6,7},{7,4},   // the mx.z ring
        {0,4},{1,5},{2,6},{3,7},   // the four connecting struts
    };
    for (auto& e : edges) r.drawLine(c[e[0]], c[e[1]], color);
}

void arrow(Renderer3D& r, const Vec3& from, const Vec3& to, const Color& color) {
    r.drawLine(from, to, color);

    const Vec3 dir = to - from;
    const Real len = length(dir);
    if (len < 1e-5f) return;                 // no direction → no head
    const Vec3 d = dir * (Real(1) / len);

    // Build two vectors perpendicular to the shaft to splay the arrowhead.
    // Pick a reference axis that is not parallel to the shaft to avoid a
    // degenerate cross product.
    const Vec3 ref = (std::fabs(d.y) < 0.95f) ? axis::Y : axis::X;
    const Vec3 side = normalize(cross(d, ref));
    const Vec3 up   = cross(d, side);

    const Real headLen = std::min(Real(0.22) * len, Real(0.35));
    const Real headW   = headLen * Real(0.5);
    const Vec3 base    = to - d * headLen;
    r.drawLine(to, base + side * headW, color);
    r.drawLine(to, base - side * headW, color);
    r.drawLine(to, base + up   * headW, color);
    r.drawLine(to, base - up   * headW, color);
}

void point(Renderer3D& r, const Vec3& p, Real size, const Color& color) {
    r.drawLine(p - axis::X * size, p + axis::X * size, color);
    r.drawLine(p - axis::Y * size, p + axis::Y * size, color);
    r.drawLine(p - axis::Z * size, p + axis::Z * size, color);
}

}  // namespace p3d::debug
