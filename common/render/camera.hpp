// ===========================================================================
//  camera.hpp — an orbit camera that turns mouse drags into view matrices
// ---------------------------------------------------------------------------
//  Introduced in: Chapter 4 (Camera & Projection)
//
//  Every demo uses the same camera model: it ORBITS a target point, like a
//  camera on a boom arm. You store three numbers — how far away (distance) and
//  two angles (yaw around the vertical axis, pitch up/down) — and from those
//  you can place the camera in space. Dragging the mouse changes the angles;
//  the wheel changes the distance. The camera then hands the renderer two
//  matrices: a VIEW matrix (world → camera space) and a PROJECTION matrix
//  (camera space → the flattened clip space). Chapter 4 derives both.
// ===========================================================================
#pragma once

#include "../math/math.hpp"

namespace p3d {

class OrbitCamera {
public:
    // Where the camera looks, and how the boom arm is posed.
    Vec3 target{0, 0, 0};
    Real distance{8};
    Real yaw{radians(35)};    // rotation around the world +y axis
    Real pitch{radians(20)};  // elevation above the horizon

    // Lens parameters.
    Real fovY{radians(55)};
    Real nearZ{0.05f};
    Real farZ{200.0f};

    // The camera's position in world space, reconstructed from the boom-arm
    // angles. This is spherical-to-Cartesian: yaw sweeps around, pitch lifts
    // up, distance sets the radius.
    Vec3 position() const {
        const Real cp = std::cos(pitch), sp = std::sin(pitch);
        const Real cy = std::cos(yaw),   sy = std::sin(yaw);
        return {
            target.x + distance * cp * sy,
            target.y + distance * sp,
            target.z + distance * cp * cy,
        };
    }

    Mat4 viewMatrix() const {
        return lookAt(position(), target, axis::Y);
    }

    Mat4 projectionMatrix(Real aspect) const {
        return perspective(fovY, aspect, nearZ, farZ);
    }

    // --- Input hooks (called by the App in response to mouse events) --------

    // Orbit by mouse-drag deltas (in pixels). `sensitivity` converts pixels to
    // radians. Pitch is clamped just short of the poles so the view never
    // flips upside-down (a classic orbit-camera gotcha).
    void orbit(Real dxPixels, Real dyPixels, Real sensitivity = 0.008f) {
        yaw   -= dxPixels * sensitivity;
        pitch += dyPixels * sensitivity;
        const Real limit = kHalfPi - radians(2);
        pitch = clamp(pitch, -limit, limit);
    }

    // Zoom by a wheel delta. Multiplicative so each notch feels the same at any
    // distance; clamped to a sane range so you cannot fly through the target or
    // to infinity.
    void zoom(Real wheelDelta) {
        distance *= std::pow(Real(0.9), wheelDelta);
        distance = clamp(distance, Real(0.5), Real(120));
    }
};

}  // namespace p3d
