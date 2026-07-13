// ===========================================================================
//  sat.cpp — SAT overlap test + face-clipping contact manifold for OBBs
// ===========================================================================
#include "sat.hpp"

#include <array>
#include <cmath>
#include <vector>

namespace p3d {

namespace {

// A convenient bundle: a box's world axes and half-extents.
struct Obb {
    Vec3 pos;
    Vec3 axis[3];     // the three unit world-space axes
    Real half[3];
};

Obb makeObb(const Vec3& pos, const Quat& rot, const Vec3& half) {
    Obb o;
    o.pos = pos;
    o.axis[0] = rotate(rot, {1, 0, 0});
    o.axis[1] = rotate(rot, {0, 1, 0});
    o.axis[2] = rotate(rot, {0, 0, 1});
    o.half[0] = half.x; o.half[1] = half.y; o.half[2] = half.z;
    return o;
}

// How far the box extends along a (unit) axis L: the sum of each half-extent
// projected onto L. This is the box's "radius" along L, used by SAT.
Real projectedRadius(const Obb& b, const Vec3& L) {
    return b.half[0] * std::fabs(dot(b.axis[0], L)) +
           b.half[1] * std::fabs(dot(b.axis[1], L)) +
           b.half[2] * std::fabs(dot(b.axis[2], L));
}

// The face of `b` most anti-parallel to `dir` (i.e. facing back toward `dir`).
// Returns its four world-space corners, in order. Used to pick the incident face.
std::array<Vec3, 4> mostAntiParallelFace(const Obb& b, const Vec3& dir) {
    int best = 0; Real bestDot = 1e9f; Real sign = 1;
    for (int i = 0; i < 3; ++i) {
        const Real d = dot(b.axis[i], dir);
        if (d < bestDot) { bestDot = d; best = i; sign = 1; }        // +axis face
        if (-d < bestDot) { bestDot = -d; best = i; sign = -1; }     // -axis face
    }
    const int u = (best + 1) % 3, v = (best + 2) % 3;
    const Vec3 c = b.pos + b.axis[best] * (sign * b.half[best]);     // face centre
    const Vec3 eu = b.axis[u] * b.half[u];
    const Vec3 ev = b.axis[v] * b.half[v];
    return {c - eu - ev, c + eu - ev, c + eu + ev, c - eu + ev};
}

// Sutherland–Hodgman: clip a convex polygon to the half-space dot(n, p) <= d.
std::vector<Vec3> clipToPlane(const std::vector<Vec3>& poly, const Vec3& n, Real d) {
    std::vector<Vec3> out;
    const int m = static_cast<int>(poly.size());
    for (int i = 0; i < m; ++i) {
        const Vec3& cur = poly[i];
        const Vec3& nxt = poly[(i + 1) % m];
        const Real dc = dot(n, cur) - d, dn = dot(n, nxt) - d;
        const bool inCur = dc <= 0, inNxt = dn <= 0;
        if (inCur) out.push_back(cur);
        if (inCur != inNxt) out.push_back(lerp(cur, nxt, dc / (dc - dn)));
    }
    return out;
}

}  // namespace

int collideBoxBox(const Vec3& posA, const Quat& rotA, const Vec3& halfA,
                  const Vec3& posB, const Quat& rotB, const Vec3& halfB,
                  Contact out[], int maxOut) {
    const Obb A = makeObb(posA, rotA, halfA);
    const Obb B = makeObb(posB, rotB, halfB);
    const Vec3 t = B.pos - A.pos;                 // from A toward B

    Real   minOverlap = 1e9f;
    Vec3   bestAxis{};
    int    bestType = -1;                         // 0 = A face, 1 = B face, 2 = edge

    // Test one axis. Returns false the moment a separating axis is found.
    auto test = [&](Vec3 L, int type) -> bool {
        const Real len2 = lengthSq(L);
        if (len2 < 1e-8f) return true;            // degenerate (parallel edges): skip
        L = L * (Real(1) / std::sqrt(len2));
        const Real overlap = projectedRadius(A, L) + projectedRadius(B, L) - std::fabs(dot(t, L));
        if (overlap < 0) return false;            // a gap exists → not colliding
        if (overlap < minOverlap) { minOverlap = overlap; bestAxis = L; bestType = type; }
        return true;
    };

    for (int i = 0; i < 3; ++i) if (!test(A.axis[i], 0)) return 0;   // A's 3 faces
    for (int i = 0; i < 3; ++i) if (!test(B.axis[i], 1)) return 0;   // B's 3 faces
    for (int i = 0; i < 3; ++i)                                      // 9 edge crosses
        for (int j = 0; j < 3; ++j)
            if (!test(cross(A.axis[i], B.axis[j]), 2)) return 0;

    // Orient the axis to point from A toward B, then the contact normal (B→A) is
    // simply its negation (the direction to push A out).
    if (dot(bestAxis, t) < 0) bestAxis = -bestAxis;
    const Vec3 normalBtoA = -bestAxis;

    int count = 0;

    if (bestType == 2) {
        // Edge–edge: a single contact roughly at the shallowest point. A midpoint
        // between the box centres along the axis is a good, cheap approximation.
        if (maxOut > 0) {
            out[0].normal = normalBtoA;
            out[0].penetration = minOverlap;
            out[0].point = (A.pos + B.pos) * Real(0.5);
            count = 1;
        }
        return count;
    }

    // Face case. The REFERENCE face is on whichever box owns the best axis; the
    // INCIDENT face is the other box's face that most faces back toward it. We
    // clip the incident face against the reference face's side planes.
    const Obb& ref = (bestType == 0) ? A : B;
    const Obb& inc = (bestType == 0) ? B : A;
    // Reference face outward normal (points away from the reference box).
    const Vec3 refNormal = (bestType == 0) ? bestAxis : -bestAxis;

    // Which reference axis is the face normal, and its face centre + side extents.
    int r = 0; Real rbest = -1e9f, rsign = 1;
    for (int i = 0; i < 3; ++i) {
        const Real d = dot(ref.axis[i], refNormal);
        if (std::fabs(d) > rbest) { rbest = std::fabs(d); r = i; rsign = (d > 0) ? 1 : -1; }
    }
    const int ru = (r + 1) % 3, rv = (r + 2) % 3;
    const Vec3 refCenter = ref.pos + ref.axis[r] * (rsign * ref.half[r]);

    // Incident face polygon, then clip it to the reference face's four sides.
    const std::array<Vec3, 4> incFace = mostAntiParallelFace(inc, refNormal);
    std::vector<Vec3> poly(incFace.begin(), incFace.end());
    poly = clipToPlane(poly,  ref.axis[ru], dot(ref.axis[ru], refCenter) + ref.half[ru]);
    poly = clipToPlane(poly, -ref.axis[ru], -dot(ref.axis[ru], refCenter) + ref.half[ru]);
    poly = clipToPlane(poly,  ref.axis[rv], dot(ref.axis[rv], refCenter) + ref.half[rv]);
    poly = clipToPlane(poly, -ref.axis[rv], -dot(ref.axis[rv], refCenter) + ref.half[rv]);

    // Keep the clipped points that lie on/behind the reference face (penetrating).
    for (const Vec3& p : poly) {
        if (count >= maxOut) break;
        const Real depth = dot(refNormal, refCenter - p);   // >0 means below the face
        if (depth < -1e-4f) continue;
        out[count].normal = normalBtoA;
        out[count].penetration = std::max(Real(0), depth);
        out[count].point = p;
        ++count;
    }

    // Degenerate fallback (clipping produced nothing): one central contact.
    if (count == 0 && maxOut > 0) {
        out[0].normal = normalBtoA;
        out[0].penetration = minOverlap;
        out[0].point = (A.pos + B.pos) * Real(0.5);
        count = 1;
    }
    return count;
}

}  // namespace p3d
