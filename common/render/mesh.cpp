// ===========================================================================
//  mesh.cpp — implementations of the primitive mesh generators
// ===========================================================================
#include "mesh.hpp"

#include "../math/scalar.hpp"
#include <cmath>

namespace p3d::mesh {

// Helper: append one triangle (three fresh vertices) to a mesh. Using fresh
// vertices per triangle (rather than sharing) is exactly what we want for flat
// shading — no vertex is shared between faces that should look distinct.
static void addTri(Mesh& m, const Vec3& a, const Vec3& b, const Vec3& c) {
    const int base = static_cast<int>(m.positions.size());
    m.positions.push_back(a);
    m.positions.push_back(b);
    m.positions.push_back(c);
    m.indices.push_back(base + 0);
    m.indices.push_back(base + 1);
    m.indices.push_back(base + 2);
}

// Helper: append a quad as two triangles. Corners must be given in counter-
// clockwise order when viewed from the front (outside) face, so the renderer's
// backface culling and shading agree on which way is "out".
static void addQuad(Mesh& m, const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d) {
    addTri(m, a, b, c);
    addTri(m, a, c, d);
}

Mesh box(const Vec3& h) {
    Mesh m;
    // Eight corners of the box.
    const Vec3 p000{-h.x, -h.y, -h.z}, p100{ h.x, -h.y, -h.z};
    const Vec3 p110{ h.x,  h.y, -h.z}, p010{-h.x,  h.y, -h.z};
    const Vec3 p001{-h.x, -h.y,  h.z}, p101{ h.x, -h.y,  h.z};
    const Vec3 p111{ h.x,  h.y,  h.z}, p011{-h.x,  h.y,  h.z};

    // Each face wound counter-clockwise as seen from outside.
    addQuad(m, p001, p101, p111, p011);  // +z (front)
    addQuad(m, p100, p000, p010, p110);  // -z (back)
    addQuad(m, p101, p100, p110, p111);  // +x (right)
    addQuad(m, p000, p001, p011, p010);  // -x (left)
    addQuad(m, p011, p111, p110, p010);  // +y (top)
    addQuad(m, p000, p100, p101, p001);  // -y (bottom)
    return m;
}

Mesh sphere(Real radius, int rings, int sectors) {
    Mesh m;
    if (rings < 2)   rings = 2;
    if (sectors < 3) sectors = 3;

    // Sample points on the sphere by latitude (phi) and longitude (theta),
    // then stitch neighbouring samples into quads. Each cell of the grid
    // becomes one flat-shaded quad (two triangles).
    auto point = [&](int ring, int sector) -> Vec3 {
        const Real phi   = kPi * Real(ring)   / Real(rings);     // 0..pi (pole to pole)
        const Real theta = kTwoPi * Real(sector) / Real(sectors); // 0..2pi around
        const Real sinPhi = std::sin(phi);
        return {
            radius * sinPhi * std::cos(theta),
            radius * std::cos(phi),
            radius * sinPhi * std::sin(theta),
        };
    };

    for (int r = 0; r < rings; ++r) {
        for (int s = 0; s < sectors; ++s) {
            const Vec3 a = point(r,     s);
            const Vec3 b = point(r + 1, s);
            const Vec3 c = point(r + 1, s + 1);
            const Vec3 d = point(r,     s + 1);
            // Degenerate triangles at the poles (a == d or b == c) are harmless
            // — they contribute zero area and the renderer skips them.
            addQuad(m, a, b, c, d);
        }
    }
    return m;
}

Mesh plane(Real size, int subdiv) {
    Mesh m;
    if (subdiv < 1) subdiv = 1;
    const Real half = size * Real(0.5);
    const Real step = size / Real(subdiv);

    for (int i = 0; i < subdiv; ++i) {
        for (int j = 0; j < subdiv; ++j) {
            const Real x0 = -half + step * Real(i);
            const Real z0 = -half + step * Real(j);
            const Real x1 = x0 + step;
            const Real z1 = z0 + step;
            // Wound counter-clockwise as seen from above (+y), so the top face
            // points up.
            addQuad(m,
                    {x0, 0, z0}, {x0, 0, z1}, {x1, 0, z1}, {x1, 0, z0});
        }
    }
    return m;
}

Mesh cylinder(Real radius, Real height, int sectors) {
    Mesh m;
    if (sectors < 3) sectors = 3;
    const Real hy = height * Real(0.5);

    auto rim = [&](int s, Real y) -> Vec3 {
        const Real theta = kTwoPi * Real(s) / Real(sectors);
        return {radius * std::cos(theta), y, radius * std::sin(theta)};
    };

    for (int s = 0; s < sectors; ++s) {
        const Vec3 bottomA = rim(s,     -hy), bottomB = rim(s + 1, -hy);
        const Vec3 topA    = rim(s,      hy), topB    = rim(s + 1,  hy);
        // Side wall (one quad per sector).
        addQuad(m, bottomA, topA, topB, bottomB);
        // Top and bottom caps, as fans to the centre point.
        addTri(m, {0,  hy, 0}, topA,    topB);
        addTri(m, {0, -hy, 0}, bottomB, bottomA);
    }
    return m;
}

}  // namespace p3d::mesh
