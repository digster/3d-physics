// ===========================================================================
//  rigidbody.hpp — a body that can move AND spin
// ---------------------------------------------------------------------------
//  Introduced in: Chapter 11 (Rigid-Body Dynamics)
//
//  A particle (Part II) was a point: it could move but not turn. A RIGID BODY
//  adds orientation and rotation, and with them a second copy of Newton's laws —
//  the rotational versions:
//
//        LINEAR                           ANGULAR
//    force  F = m a                    torque  τ = I α
//    momentum p = m v                  ang. momentum  L = I ω
//    position x                        orientation q (a quaternion)
//
//  The two halves run in parallel. Force moves the centre of mass; torque
//  (a force applied off-centre, τ = r × F) spins the body.
//
//  A KEY DESIGN CHOICE: we store the angular MOMENTUM L, not the angular
//  velocity ω. Under zero torque, L is exactly conserved (Newton), so keeping it
//  as the primary variable makes free rotation rock-solid — and it's what lets
//  the tumbling T-handle in Chapter 11 flip the way the real one does. We derive
//  ω from L each step via ω = I⁻¹ L, using the inertia tensor rotated into world
//  space for the body's current orientation.
// ===========================================================================
#pragma once

#include "../common/math/math.hpp"

namespace p3d {

struct RigidBody {
    // --- Linear state (centre of mass, in world space) --------------------
    Vec3 position{};
    Vec3 linearVelocity{};
    Vec3 forceAccum{};
    Real invMass{1};

    // --- Angular state ----------------------------------------------------
    Quat orientation{Quat::identity()};
    Vec3 angularMomentum{};   // L, world space (the conserved quantity)
    Vec3 torqueAccum{};       // world space
    // Inertia tensor in BODY space, and its inverse. Constant for a rigid body.
    Mat3 inertiaBody{Mat3::identity()};
    Mat3 invInertiaBody{Mat3::identity()};

    // --- Setup ------------------------------------------------------------
    void setMass(Real m)   { invMass = (m <= 0) ? 0 : Real(1) / m; }
    Real mass() const      { return (invMass == 0) ? 0 : Real(1) / invMass; }
    bool isStatic() const  { return invMass == 0; }

    // Give the body its inertia tensor (from inertia.hpp). We cache the inverse
    // because ω = I⁻¹ L is needed every single step.
    void setInertia(const Mat3& I) { inertiaBody = I; invInertiaBody = inverse(I); }

    void makeStatic() {
        invMass = 0; linearVelocity = Vec3{}; angularMomentum = Vec3{};
        inertiaBody = Mat3{}; invInertiaBody = Mat3{};   // zero ⇒ cannot spin
    }

    // --- Frames: body ↔ world --------------------------------------------
    Mat3 rotation() const { return toMat3(orientation); }        // body → world
    // The inertia tensor and its inverse, rotated into world space:
    //   I_world = R · I_body · Rᵀ   (and likewise for the inverse)
    Mat3 inertiaWorld() const    { Mat3 R = rotation(); return R * inertiaBody    * transpose(R); }
    Mat3 invInertiaWorld() const { return invInertiaWorldFor(orientation); }
    // Same, but for an arbitrary orientation — needed by the midpoint step below.
    Mat3 invInertiaWorldFor(const Quat& q) const {
        Mat3 R = toMat3(q);
        return R * invInertiaBody * transpose(R);
    }

    Vec3 angularVelocity() const { return invInertiaWorld() * angularMomentum; }
    // Set the spin directly (converts ω → L using the current world inertia).
    void setAngularVelocity(const Vec3& omega) { angularMomentum = inertiaWorld() * omega; }

    // A point fixed on the body, expressed in world space (for applying forces
    // at a spot, and for drawing markers attached to the body).
    Vec3 worldPoint(const Vec3& bodyPoint) const {
        return position + rotate(orientation, bodyPoint);
    }
    // The model matrix the renderer wants: place, then orient.
    Mat4 transform() const { return translation(position) * toMat4(orientation); }

    // --- Applying forces / torques ---------------------------------------
    void applyForce(const Vec3& f) { forceAccum += f; }         // through the COM: no spin
    void applyTorque(const Vec3& t) { torqueAccum += t; }
    // A force applied at a world-space point. Off-centre forces both push the
    // body AND twist it: the twist is the torque τ = r × F.
    void applyForceAtPoint(const Vec3& f, const Vec3& worldPos) {
        forceAccum += f;
        torqueAccum += cross(worldPos - position, f);
    }
    void clearAccumulators() { forceAccum = Vec3{}; torqueAccum = Vec3{}; }

    // --- One step of motion ----------------------------------------------
    void integrate(Real dt) {
        // Linear half: identical to a particle (semi-implicit Euler).
        if (!isStatic()) {
            linearVelocity += forceAccum * invMass * dt;
            position       += linearVelocity * dt;
        }

        // Angular half. Torque changes the angular momentum L (which is otherwise
        // conserved). Then we turn the orientation by the angular velocity ω,
        // where ω = I⁻¹_world · L.
        angularMomentum += torqueAccum * dt;

        // We advance the orientation with an exponential-map MIDPOINT step, not a
        // naive q += ½ωq·dt. Why: ω depends on the orientation (through I_world),
        // so a first-order explicit step steadily injects energy — a tumbling
        // body would spin ever faster instead of flipping. Sampling ω at the
        // midpoint of the step (a second-order method) keeps energy nearly
        // constant, so free rotation behaves like the real thing.
        const Vec3 omegaStart = invInertiaWorld() * angularMomentum;
        const Quat qMid = normalize(spin(omegaStart, dt * Real(0.5)) * orientation);
        const Vec3 omegaMid = invInertiaWorldFor(qMid) * angularMomentum;
        orientation = normalize(spin(omegaMid, dt) * orientation);

        clearAccumulators();
    }

    // The exact rotation quaternion for spinning at angular velocity ω for time
    // dt: a rotation of angle |ω|·dt about the ω axis (the "exponential map").
    static Quat spin(const Vec3& omega, Real dt) {
        const Real angle = length(omega) * dt;
        if (angle < Real(1e-8)) return Quat::identity();
        return Quat::fromAxisAngle(omega, angle);
    }

    // Total kinetic energy: ½ m|v|²  (linear)  +  ½ ω·L  (rotational).
    Real kineticEnergy() const {
        const Real lin = isStatic() ? 0 : Real(0.5) * mass() * lengthSq(linearVelocity);
        return lin + Real(0.5) * dot(angularVelocity(), angularMomentum);
    }
};

}  // namespace p3d
