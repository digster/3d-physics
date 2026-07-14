// ===========================================================================
//  joint.hpp — joints: constraints the solver is not allowed to give up on
// ---------------------------------------------------------------------------
//  Introduced in: Chapter 19 (Joints)
//
//  A contact is a ONE-SIDED constraint: it can only push bodies apart, so its
//  impulse is clamped to be non-negative. A JOINT is a TWO-SIDED (equality)
//  constraint: it holds a relationship EXACTLY, pushing or pulling as needed, so
//  its impulse is unclamped. Otherwise a joint uses the very same impulse
//  machinery as a contact — effective mass, a Baumgarte position bias, warm
//  starting — which is why joints drop straight into the Part V solver loop.
//
//  Three joints, from simplest to richest:
//    * DistanceJoint   keeps two anchor points a fixed distance apart (a rod).
//    * BallSocketJoint  pins two anchor points together (a shoulder / a chain link).
//    * HingeJoint       a ball-socket that also forces the two bodies to share a
//                       spin axis, so they can only rotate about that one line.
//
//  Anchors are given in each body's LOCAL frame, so they follow the body as it
//  moves. One body may be static (an anchor to the world).
// ===========================================================================
#pragma once

#include "rigidbody.hpp"
#include <vector>

namespace p3d {

// The common interface. The World calls prepare() once per step, warmStart()
// once, then solve() every velocity iteration.
struct Joint {
    int  bodyA{-1}, bodyB{-1};
    Real beta{0.35f};       // position-correction stiffness (firmer than contacts)

    virtual ~Joint() = default;
    virtual void prepare(std::vector<RigidBody>& bodies, Real dt) = 0;
    virtual void warmStart(std::vector<RigidBody>& bodies) = 0;
    virtual void solve(std::vector<RigidBody>& bodies) = 0;
};

// --- Distance joint: a rigid rod between two anchor points ------------------
struct DistanceJoint : Joint {
    Vec3 localA{}, localB{};
    Real restLength{1};

    DistanceJoint(int a, int b, const Vec3& la, const Vec3& lb, Real len)
        { bodyA = a; bodyB = b; localA = la; localB = lb; restLength = len; }

    void prepare(std::vector<RigidBody>& bodies, Real dt) override;
    void warmStart(std::vector<RigidBody>& bodies) override;
    void solve(std::vector<RigidBody>& bodies) override;

private:
    Vec3 rA{}, rB{}, worldA{}, worldB{}, n{};
    Real effMass{0}, bias{0}, accum{0};
};

// --- Ball-socket joint: pin two anchor points together (3 DOF) --------------
struct BallSocketJoint : Joint {
    Vec3 localA{}, localB{};

    BallSocketJoint(int a, int b, const Vec3& la, const Vec3& lb)
        { bodyA = a; bodyB = b; localA = la; localB = lb; }

    void prepare(std::vector<RigidBody>& bodies, Real dt) override;
    void warmStart(std::vector<RigidBody>& bodies) override;
    void solve(std::vector<RigidBody>& bodies) override;

private:
    Vec3 rA{}, rB{}, worldA{}, worldB{}, bias{}, accum{};
    Mat3 effMass{};   // inverse of the 3×3 constraint mass matrix
};

// --- Hinge joint: ball-socket + a shared spin axis (1 rotational DOF) -------
struct HingeJoint : Joint {
    Vec3 localA{}, localB{};        // pivot anchor in each body
    Vec3 localAxisA{}, localAxisB{}; // hinge axis in each body's frame

    HingeJoint(int a, int b, const Vec3& la, const Vec3& lb, const Vec3& axisA, const Vec3& axisB)
        { bodyA = a; bodyB = b; localA = la; localB = lb; localAxisA = axisA; localAxisB = axisB; }

    void prepare(std::vector<RigidBody>& bodies, Real dt) override;
    void warmStart(std::vector<RigidBody>& bodies) override;
    void solve(std::vector<RigidBody>& bodies) override;

private:
    // Point (ball-socket) part.
    Vec3 rA{}, rB{}, worldA{}, worldB{}, pointBias{}, pointAccum{};
    Mat3 pointMass{};
    // Angular (axis-alignment) part: two constraints in the plane ⟂ to the axis.
    Vec3 t1{}, t2{};
    Real angMassInv[4]{};   // inverse of the 2×2 angular mass, row-major
    Real angBias1{0}, angBias2{0}, angAccum1{0}, angAccum2{0};
};

}  // namespace p3d
