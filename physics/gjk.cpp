// ===========================================================================
//  gjk.cpp — GJK intersection test and EPA penetration recovery
// ---------------------------------------------------------------------------
//  Read alongside docs/chapters/15-gjk.html. The comments mark each stage of the
//  simplex evolution (GJK) and the polytope expansion (EPA).
// ===========================================================================
#include "gjk.hpp"

#include "../common/math/quat.hpp"
#include <algorithm>
#include <array>
#include <cmath>

namespace p3d {
namespace {

// --- Support functions -----------------------------------------------------
// The farthest vertex of a convex point set in direction `dir`.
Vec3 supportOf(const std::vector<Vec3>& pts, const Vec3& dir) {
    int best = 0;
    Real bestDot = dot(pts[0], dir);
    for (int i = 1; i < static_cast<int>(pts.size()); ++i) {
        const Real d = dot(pts[i], dir);
        if (d > bestDot) { bestDot = d; best = i; }
    }
    return pts[best];
}
// A point on the Minkowski difference A⊖B, farthest along `dir`.
Vec3 support(const std::vector<Vec3>& A, const std::vector<Vec3>& B, const Vec3& dir) {
    return supportOf(A, dir) - supportOf(B, -dir);
}

// The vector triple product cross(cross(a,b),c), used to point "toward origin,
// perpendicular to an edge".
Vec3 tripleCross(const Vec3& a, const Vec3& b, const Vec3& c) {
    return cross(cross(a, b), c);
}

// --- GJK simplex updates ---------------------------------------------------
// Each takes the current simplex (front = most recently added point) and the
// search direction, both by reference, and returns true if the simplex now
// encloses the origin.
using Simplex = std::vector<Vec3>;

bool updateLine(Simplex& s, Vec3& dir) {
    const Vec3 a = s[0], b = s[1];        // a is newest
    const Vec3 ab = b - a, ao = -a;
    if (dot(ab, ao) > 0) {
        dir = tripleCross(ab, ao, ab);    // perpendicular to ab, toward origin
    } else {
        s = {a}; dir = ao;                // origin is behind a → keep just a
    }
    return false;
}

bool updateTriangle(Simplex& s, Vec3& dir) {
    const Vec3 a = s[0], b = s[1], c = s[2];
    const Vec3 ab = b - a, ac = c - a, ao = -a;
    const Vec3 abc = cross(ab, ac);       // triangle normal

    if (dot(cross(abc, ac), ao) > 0) {    // outside edge ac
        if (dot(ac, ao) > 0) { s = {a, c}; dir = tripleCross(ac, ao, ac); return false; }
        s = {a, b}; return updateLine(s, dir);
    }
    if (dot(cross(ab, abc), ao) > 0) {    // outside edge ab
        s = {a, b}; return updateLine(s, dir);
    }
    // Inside the triangle's edges: origin is above or below the face.
    if (dot(abc, ao) > 0) { dir = abc; }              // above
    else { s = {a, c, b}; dir = -abc; }               // below (flip winding)
    return false;
}

bool updateTetra(Simplex& s, Vec3& dir) {
    const Vec3 a = s[0], b = s[1], c = s[2], d = s[3];   // a newest
    const Vec3 ao = -a;
    // Outward normals of the three faces that touch the new point a.
    const Vec3 abc = cross(b - a, c - a);
    const Vec3 acd = cross(c - a, d - a);
    const Vec3 adb = cross(d - a, b - a);

    // If the origin is outside any of those faces, drop to that triangle.
    if (dot(abc, ao) > 0) { s = {a, b, c}; return updateTriangle(s, dir); }
    if (dot(acd, ao) > 0) { s = {a, c, d}; return updateTriangle(s, dir); }
    if (dot(adb, ao) > 0) { s = {a, d, b}; return updateTriangle(s, dir); }
    return true;                                         // origin enclosed
}

bool doSimplex(Simplex& s, Vec3& dir) {
    switch (s.size()) {
        case 2:  return updateLine(s, dir);
        case 3:  return updateTriangle(s, dir);
        default: return updateTetra(s, dir);
    }
}

// Run GJK; on intersection, leave the enclosing simplex in `outSimplex`.
bool gjk(const std::vector<Vec3>& A, const std::vector<Vec3>& B, Simplex& outSimplex) {
    Vec3 dir{1, 0, 0};
    Simplex s = {support(A, B, dir)};
    dir = -s[0];
    for (int iter = 0; iter < 64; ++iter) {
        if (lengthSq(dir) < 1e-12f) dir = {1, 0, 0};
        const Vec3 a = support(A, B, dir);
        if (dot(a, dir) < 0) return false;    // farthest point didn't pass origin → apart
        s.insert(s.begin(), a);               // push to front (a is newest)
        if (doSimplex(s, dir)) { outSimplex = s; return true; }
    }
    outSimplex = s;
    return true;                              // ran out of iterations: treat as touching
}

// --- EPA -------------------------------------------------------------------
struct EFace { int a, b, c; Vec3 normal; Real dist; };

// Any unit vector perpendicular to v.
Vec3 anyPerp(const Vec3& v) {
    const Vec3 a = (std::fabs(v.x) < 0.9f) ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
    return normalize(cross(v, a));
}

// GJK's terminating simplex can be degenerate (a box's flat faces make the
// support function return the same vertex for several directions), which is not
// a valid starting tetrahedron for EPA. This "blows it up" to a real, non-flat
// tetrahedron by fetching fresh support points — the standard robustness step.
bool buildTetrahedron(const std::vector<Vec3>& A, const std::vector<Vec3>& B, Simplex& s) {
    // Drop near-duplicate points first.
    Simplex u;
    for (const Vec3& p : s) {
        bool dup = false;
        for (const Vec3& q : u) if (lengthSq(p - q) < 1e-8f) { dup = true; break; }
        if (!dup) u.push_back(p);
    }
    s = u;

    if (s.empty()) s.push_back(support(A, B, {1, 0, 0}));
    if (s.size() == 1) {                      // find a distinct second point
        const Vec3 dirs[6] = {{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}};
        for (const Vec3& d : dirs) { const Vec3 p = support(A, B, d);
            if (lengthSq(p - s[0]) > 1e-6f) { s.push_back(p); break; } }
    }
    if (s.size() == 2) {                      // sweep around the line for a wide triangle
        const Vec3 ab = s[1] - s[0];
        const Vec3 perp = anyPerp(ab);
        Vec3 best{}; Real bestArea = 0;
        for (int k = 0; k < 6; ++k) {
            const Quat q = Quat::fromAxisAngle(ab, k * (kPi / 3));
            const Vec3 p = support(A, B, rotate(q, perp));
            const Real area = lengthSq(cross(p - s[0], ab));
            if (area > bestArea) { bestArea = area; best = p; }
        }
        s.push_back(best);
    }
    if (s.size() == 3) {                      // add a 4th point off the triangle plane
        Vec3 n = cross(s[1] - s[0], s[2] - s[0]);
        Vec3 p = support(A, B, n);
        if (std::fabs(dot(normalize(n), p - s[0])) < 1e-4f) p = support(A, B, -n);
        s.push_back(p);
    }
    // Reject if still flat (near-zero volume).
    if (s.size() < 4) return false;
    const Real vol = dot(cross(s[1] - s[0], s[2] - s[0]), s[3] - s[0]);
    return std::fabs(vol) > 1e-7f;
}

Penetration epa(const std::vector<Vec3>& A, const std::vector<Vec3>& B, Simplex simplex) {
    Penetration res;
    if (!buildTetrahedron(A, B, simplex)) return res;   // couldn't form a tetra → skip

    std::vector<Vec3> verts(simplex.begin(), simplex.end());
    std::vector<EFace> faces;

    // Add a face keeping the given WINDING (never swap the vertex order — the
    // horizon-edge cancellation during expansion relies on every shared edge
    // appearing in opposite directions on its two faces). We only flip the
    // NORMAL VECTOR so it points away from the origin, for distance/visibility.
    auto addFace = [&](int i, int j, int k) {
        Vec3 n = cross(verts[j] - verts[i], verts[k] - verts[i]);
        const Real len = length(n);
        if (len < 1e-9f) return;              // skip degenerate slivers
        n = n * (Real(1) / len);
        Real d = dot(n, verts[i]);
        if (d < 0) { n = -n; d = -d; }        // flip the normal only, keep winding
        faces.push_back({i, j, k, n, d});
    };
    // This particular ordering is consistently wound: every shared edge appears
    // reversed on its two faces (0-1 as 0→1 and 1→0, etc.), which is what makes
    // the expansion below well-formed.
    addFace(0, 1, 2); addFace(0, 3, 1); addFace(0, 2, 3); addFace(1, 3, 2);
    if (faces.size() < 4) return res;         // simplex was degenerate (flat)

    for (int iter = 0; iter < 64; ++iter) {
        if (faces.empty()) break;
        // Closest face to the origin.
        int closest = 0;
        for (int i = 1; i < static_cast<int>(faces.size()); ++i)
            if (faces[i].dist < faces[closest].dist) closest = i;

        const Vec3 dir = faces[closest].normal;
        const Vec3 p = support(A, B, dir);
        const Real d = dot(p, dir);
        if (d - faces[closest].dist < 1e-4f) {            // converged to the surface
            res.hit = true;
            // EPA's outward normal on A⊖B points from A toward B; the Contact
            // convention wants B → A (the direction to push A out), so negate.
            res.normal = -dir;
            res.depth = faces[closest].dist;
            break;
        }

        // Expand: remove every face the new point can "see", find the horizon
        // (edges on the boundary of the removed region), and stitch p to it.
        const int pi = static_cast<int>(verts.size());
        verts.push_back(p);
        std::vector<std::pair<int, int>> horizon;
        auto addEdge = [&](int i, int j) {
            for (auto it = horizon.begin(); it != horizon.end(); ++it)
                if (it->first == j && it->second == i) { horizon.erase(it); return; }  // shared → interior
            horizon.push_back({i, j});
        };
        for (int i = static_cast<int>(faces.size()) - 1; i >= 0; --i) {
            if (dot(faces[i].normal, p - verts[faces[i].a]) > 0) {   // face visible from p
                addEdge(faces[i].a, faces[i].b);
                addEdge(faces[i].b, faces[i].c);
                addEdge(faces[i].c, faces[i].a);
                faces.erase(faces.begin() + i);
            }
        }
        for (auto& e : horizon) addFace(e.first, e.second, pi);
        if (faces.empty()) break;
    }

    if (res.hit) {
        // A representative contact point: A's farthest point toward B, which is
        // along -normal (normal points B→A, so -normal points A→B).
        res.point = supportOf(A, -res.normal);
    }
    return res;
}

}  // namespace

bool gjkIntersect(const std::vector<Vec3>& shapeA, const std::vector<Vec3>& shapeB) {
    if (shapeA.empty() || shapeB.empty()) return false;
    Simplex s;
    return gjk(shapeA, shapeB, s);
}

Penetration gjkEpa(const std::vector<Vec3>& shapeA, const std::vector<Vec3>& shapeB) {
    Penetration res;
    if (shapeA.empty() || shapeB.empty()) return res;
    Simplex s;
    if (!gjk(shapeA, shapeB, s)) return res;   // apart → no penetration
    return epa(shapeA, shapeB, s);
}

}  // namespace p3d
