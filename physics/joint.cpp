// ===========================================================================
//  joint.cpp — the three joints, prepared and solved with impulses
// ===========================================================================
#include "joint.hpp"

#include <cmath>

namespace p3d {

// Scalar effective mass along a unit direction (identical to the contact case).
static Real scalarEffMass(Real imA, Real imB, const Mat3& invIA, const Mat3& invIB,
                          const Vec3& rA, const Vec3& rB, const Vec3& d) {
    const Real k = imA + imB +
                   dot(d, cross(invIA * cross(rA, d), rA)) +
                   dot(d, cross(invIB * cross(rB, d), rB));
    return (k > kEpsilon) ? Real(1) / k : Real(0);
}

// ---------------------------------------------------------------------------
//  Distance joint (a rigid rod).
// ---------------------------------------------------------------------------
void DistanceJoint::prepare(std::vector<RigidBody>& bodies, Real dt) {
    RigidBody& A = bodies[bodyA];
    RigidBody& B = bodies[bodyB];
    worldA = A.worldPoint(localA);
    worldB = B.worldPoint(localB);
    rA = worldA - A.position;
    rB = worldB - B.position;

    Vec3 d = worldB - worldA;
    const Real dist = length(d);
    n = (dist > kEpsilon) ? d * (Real(1) / dist) : Vec3{1, 0, 0};

    effMass = scalarEffMass(A.invMass, B.invMass, A.invInertiaWorld(), B.invInertiaWorld(), rA, rB, n);
    bias = (beta / dt) * (dist - restLength);     // pull toward the rest length
}

void DistanceJoint::warmStart(std::vector<RigidBody>& bodies) {
    const Vec3 P = n * accum;
    bodies[bodyB].applyImpulse(P, worldB);
    bodies[bodyA].applyImpulse(-P, worldA);
}

void DistanceJoint::solve(std::vector<RigidBody>& bodies) {
    RigidBody& A = bodies[bodyA];
    RigidBody& B = bodies[bodyB];
    // Relative speed along the rod (how fast the length is changing).
    const Real cdot = dot(B.velocityAtPoint(worldB) - A.velocityAtPoint(worldA), n);
    const Real lambda = -(cdot + bias) * effMass;   // unclamped: a rod can push or pull
    accum += lambda;
    const Vec3 P = n * lambda;
    B.applyImpulse(P, worldB);
    A.applyImpulse(-P, worldA);
}

// ---------------------------------------------------------------------------
//  Ball-socket joint (point-to-point).
// ---------------------------------------------------------------------------
void BallSocketJoint::prepare(std::vector<RigidBody>& bodies, Real dt) {
    RigidBody& A = bodies[bodyA];
    RigidBody& B = bodies[bodyB];
    worldA = A.worldPoint(localA);
    worldB = B.worldPoint(localB);
    rA = worldA - A.position;
    rB = worldB - B.position;

    const Mat3 invIA = A.invInertiaWorld();
    const Mat3 invIB = B.invInertiaWorld();

    // K = (mA⁻¹+mB⁻¹)·I − skew(rA)·IA⁻¹·skew(rA) − skew(rB)·IB⁻¹·skew(rB).
    // Inverting K turns "the relative velocity error" into "the impulse to fix it".
    Mat3 K = Mat3::identity() * (A.invMass + B.invMass);
    const Mat3 sA = skew(rA), sB = skew(rB);
    K = K - sA * invIA * sA - sB * invIB * sB;
    effMass = inverse(K);

    bias = (worldA - worldB) * (beta / dt);   // drive the two anchors together
}

void BallSocketJoint::warmStart(std::vector<RigidBody>& bodies) {
    bodies[bodyA].applyImpulse(accum, worldA);
    bodies[bodyB].applyImpulse(-accum, worldB);
}

void BallSocketJoint::solve(std::vector<RigidBody>& bodies) {
    RigidBody& A = bodies[bodyA];
    RigidBody& B = bodies[bodyB];
    const Vec3 vrel = A.velocityAtPoint(worldA) - B.velocityAtPoint(worldB);
    const Vec3 P = effMass * (-(vrel + bias));   // 3×3 solve, no clamp (equality)
    accum += P;
    A.applyImpulse(P, worldA);
    B.applyImpulse(-P, worldB);
}

// ---------------------------------------------------------------------------
//  Hinge joint: the ball-socket point part, plus a 2-DOF angular constraint
//  that keeps the two bodies' hinge axes aligned (so only rotation about that
//  axis is free).
// ---------------------------------------------------------------------------
void HingeJoint::prepare(std::vector<RigidBody>& bodies, Real dt) {
    RigidBody& A = bodies[bodyA];
    RigidBody& B = bodies[bodyB];

    // --- Point part (identical to the ball-socket) ---
    worldA = A.worldPoint(localA);
    worldB = B.worldPoint(localB);
    rA = worldA - A.position;
    rB = worldB - B.position;
    const Mat3 invIA = A.invInertiaWorld();
    const Mat3 invIB = B.invInertiaWorld();
    Mat3 K = Mat3::identity() * (A.invMass + B.invMass);
    K = K - skew(rA) * invIA * skew(rA) - skew(rB) * invIB * skew(rB);
    pointMass = inverse(K);
    pointBias = (worldA - worldB) * (beta / dt);

    // --- Angular part: keep axisB aligned with axisA ---
    const Vec3 axisA = normalize(rotate(A.orientation, localAxisA));
    const Vec3 axisB = normalize(rotate(B.orientation, localAxisB));
    // Two directions spanning the plane perpendicular to the hinge axis.
    const Vec3 ref = (std::fabs(axisA.x) < 0.9f) ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
    t1 = normalize(cross(axisA, ref));
    t2 = cross(axisA, t1);

    // 2×2 angular effective mass: K_ij = t_i · (IA⁻¹ + IB⁻¹) · t_j.
    const Mat3 invSum = invIA + invIB;
    const Real k11 = dot(t1, invSum * t1), k12 = dot(t1, invSum * t2);
    const Real k21 = dot(t2, invSum * t1), k22 = dot(t2, invSum * t2);
    const Real det = k11 * k22 - k12 * k21;
    const Real invDet = (std::fabs(det) > kEpsilon) ? Real(1) / det : 0;
    angMassInv[0] =  k22 * invDet; angMassInv[1] = -k12 * invDet;
    angMassInv[2] = -k21 * invDet; angMassInv[3] =  k11 * invDet;

    // Position error: the misalignment of the two axes, along t1 and t2.
    const Vec3 err = cross(axisA, axisB);
    angBias1 = (beta / dt) * dot(err, t1);
    angBias2 = (beta / dt) * dot(err, t2);
}

void HingeJoint::warmStart(std::vector<RigidBody>& bodies) {
    RigidBody& A = bodies[bodyA];
    RigidBody& B = bodies[bodyB];
    A.applyImpulse(pointAccum, worldA);
    B.applyImpulse(-pointAccum, worldB);
    const Vec3 L = t1 * angAccum1 + t2 * angAccum2;   // angular impulse
    A.angularMomentum -= L;
    B.angularMomentum += L;
}

void HingeJoint::solve(std::vector<RigidBody>& bodies) {
    RigidBody& A = bodies[bodyA];
    RigidBody& B = bodies[bodyB];

    // Angular part: cancel the relative spin PERPENDICULAR to the hinge axis,
    // which keeps the two axes aligned while leaving rotation about the axis free.
    {
        const Vec3 wrel = B.angularVelocity() - A.angularVelocity();
        const Real c1 = dot(wrel, t1) + angBias1;
        const Real c2 = dot(wrel, t2) + angBias2;
        const Real l1 = -(angMassInv[0] * c1 + angMassInv[1] * c2);
        const Real l2 = -(angMassInv[2] * c1 + angMassInv[3] * c2);
        angAccum1 += l1; angAccum2 += l2;
        const Vec3 L = t1 * l1 + t2 * l2;
        A.angularMomentum -= L;
        B.angularMomentum += L;
    }
    // Point part LAST, so the pivot is exactly satisfied at the end of each pass
    // (solving it after the angular impulse avoids the two fighting each other).
    {
        const Vec3 vrel = A.velocityAtPoint(worldA) - B.velocityAtPoint(worldB);
        const Vec3 P = pointMass * (-(vrel + pointBias));
        pointAccum += P;
        A.applyImpulse(P, worldA);
        B.applyImpulse(-P, worldB);
    }
}

}  // namespace p3d
