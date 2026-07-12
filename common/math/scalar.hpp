// ===========================================================================
//  scalar.hpp — scalar helpers shared by the whole math library
// ---------------------------------------------------------------------------
//  Introduced in: Chapter 2 (Vectors)
//
//  Everything in the engine is built on `float`. Double precision would be a
//  touch more accurate, but SDL's vertex format is float, GPUs are float, and
//  single precision keeps the demos fast and the numbers easy to eyeball. When
//  a value below actually matters for stability we call it out.
// ===========================================================================
#pragma once

#include <cmath>
#include <numbers>   // std::numbers::pi_v (C++20)

namespace p3d {

// A single typedef for "the real-number type the engine uses". If you ever
// wanted to experiment with double precision, this is the one knob to turn.
using Real = float;

// --- Constants -------------------------------------------------------------
inline constexpr Real kPi      = std::numbers::pi_v<Real>;
inline constexpr Real kTwoPi   = Real(2) * kPi;
inline constexpr Real kHalfPi  = Real(0.5) * kPi;

// A small number used for "is this basically zero?" tests. Chosen well above
// the float epsilon so that accumulated rounding error does not trip it, but
// small enough that any physically meaningful quantity clears it.
inline constexpr Real kEpsilon = Real(1e-6);

// --- Angle conversion ------------------------------------------------------
// Humans think in degrees; trig functions think in radians. Keeping these
// explicit at call sites (e.g. `radians(45.0f)`) avoids the classic bug of
// feeding degrees straight into std::sin.
inline constexpr Real radians(Real degrees) { return degrees * (kPi / Real(180)); }
inline constexpr Real degrees(Real radians) { return radians * (Real(180) / kPi); }

// --- Small utilities -------------------------------------------------------

// Clamp `v` into the inclusive range [lo, hi].
inline constexpr Real clamp(Real v, Real lo, Real hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// Linear interpolation: t=0 gives a, t=1 gives b. Used constantly, from render
// interpolation between physics states to blending colors.
inline constexpr Real lerp(Real a, Real b, Real t) { return a + (b - a) * t; }

// Sign as -1 / 0 / +1. Handy for friction and restitution direction logic.
inline constexpr Real sign(Real v) { return (Real(0) < v) - (v < Real(0)); }

// Squaring shows up everywhere (kinetic energy, distances). A named helper
// reads better than `x * x` buried in a longer expression.
inline constexpr Real sqr(Real v) { return v * v; }

// Floating-point "are these close?" — never compare floats with ==.
inline bool approxEqual(Real a, Real b, Real eps = kEpsilon) {
    return std::fabs(a - b) <= eps;
}

}  // namespace p3d
