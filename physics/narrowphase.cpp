// ===========================================================================
//  narrowphase.cpp — sphere / plane / box contact generation
// ===========================================================================
#include "narrowphase.hpp"

#include <cmath>

namespace p3d {

// Component-wise clamp of `v` into the box [-half, +half].
static Vec3 clampToBox(const Vec3& v, const Vec3& half) {
    return {clamp(v.x, -half.x, half.x),
            clamp(v.y, -half.y, half.y),
            clamp(v.z, -half.z, half.z)};
}

bool collideSphereSphere(const Vec3& ca, Real ra, const Vec3& cb, Real rb, Contact& out) {
    const Vec3 d = ca - cb;                 // from B toward A
    const Real rsum = ra + rb;
    const Real dist2 = lengthSq(d);
    if (dist2 >= rsum * rsum) return false; // too far apart to touch

    const Real dist = std::sqrt(dist2);
    // If the centres coincide exactly, pick an arbitrary separating direction.
    out.normal = (dist > kEpsilon) ? d * (Real(1) / dist) : Vec3{0, 1, 0};
    out.penetration = rsum - dist;
    out.point = cb + out.normal * rb;       // on B's surface, facing A
    return true;
}

bool collideSpherePlane(const Vec3& c, Real r, const Vec3& n, Real offset, Contact& out) {
    // Signed distance of the centre from the plane along +n.
    const Real s = dot(n, c) - offset;
    if (s >= r) return false;               // fully on the outside → no contact
    out.normal = n;                         // plane (B) → sphere (A) is +n
    out.penetration = r - s;
    out.point = c - n * r;                  // the sphere's deepest point
    return true;
}

bool collideSphereBox(const Vec3& c, Real r,
                      const Vec3& boxPos, const Quat& boxRot, const Vec3& half,
                      Contact& out) {
    // Work in the box's local frame, where the box is axis-aligned at the origin.
    const Quat inv = conjugate(boxRot);
    const Vec3 local = rotate(inv, c - boxPos);
    const Vec3 closest = clampToBox(local, half);
    const Vec3 delta = local - closest;
    const Real d2 = lengthSq(delta);

    Vec3 nLocal, pointLocal;
    Real penetration;

    if (d2 > kEpsilon * kEpsilon) {
        // Sphere centre is OUTSIDE the box: nearest point is on a face/edge/corner.
        if (d2 > r * r) return false;       // and too far to touch
        const Real dist = std::sqrt(d2);
        nLocal = delta * (Real(1) / dist);  // box surface → centre
        penetration = r - dist;
        pointLocal = closest;
    } else {
        // Sphere centre is INSIDE the box: push out through the nearest face.
        // Find the axis where the centre is closest to a face.
        const Vec3 dist2face = half - Vec3{std::fabs(local.x), std::fabs(local.y), std::fabs(local.z)};
        int axis = 0;
        if (dist2face.y < dist2face.x) axis = 1;
        if (dist2face.z < dist2face[axis]) axis = 2;
        nLocal = Vec3{};
        nLocal[axis] = (local[axis] >= 0) ? 1.0f : -1.0f;
        penetration = r + dist2face[axis];
        pointLocal = local;
        pointLocal[axis] = nLocal[axis] * half[axis];
    }

    out.normal = rotate(boxRot, nLocal);    // back to world: box (B) → sphere (A)
    out.penetration = penetration;
    out.point = boxPos + rotate(boxRot, pointLocal);
    return true;
}

int collideBoxPlane(const Vec3& boxPos, const Quat& boxRot, const Vec3& half,
                    const Vec3& n, Real offset, Contact out[4]) {
    // Test all eight corners; the ones dipping below the plane are contacts.
    int count = 0;
    for (int sx = -1; sx <= 1; sx += 2)
        for (int sy = -1; sy <= 1; sy += 2)
            for (int sz = -1; sz <= 1; sz += 2) {
                const Vec3 localCorner{sx * half.x, sy * half.y, sz * half.z};
                const Vec3 corner = boxPos + rotate(boxRot, localCorner);
                const Real s = dot(n, corner) - offset;
                if (s < 0 && count < 4) {   // below the plane → penetrating
                    out[count].normal = n;          // plane (B) → box (A) is +n
                    out[count].penetration = -s;
                    out[count].point = corner;
                    ++count;
                }
            }
    return count;
}

}  // namespace p3d
