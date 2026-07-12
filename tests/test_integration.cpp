// ===========================================================================
//  test_integration.cpp — the ENERGY behaviour of the integrators
// ---------------------------------------------------------------------------
//  Chapter 5 claims: on a harmonic oscillator, explicit Euler gains energy,
//  semi-implicit Euler roughly conserves it, and RK4 is very accurate. Those
//  are testable properties, so we test them. The integrators are reproduced
//  here (small and self-contained) so the test does not depend on a demo.
//
//  The system is a 1D spring: a = -k·x, whose exact motion is a sine wave with
//  constant total energy E = ½v² + ½k·x².
// ===========================================================================
#include <doctest/doctest.h>
#include "common/math/scalar.hpp"
#include <cmath>

using p3d::Real;

namespace {
constexpr Real k = 16.0f;   // spring constant → angular frequency sqrt(k) = 4

struct S { Real x, v; };

Real accel(Real x) { return -k * x; }
Real energy(const S& s) { return 0.5f * s.v * s.v + 0.5f * k * s.x * s.x; }

S explicitEuler(S s, Real dt) {
    const Real a = accel(s.x);
    S n; n.x = s.x + s.v * dt; n.v = s.v + a * dt; return n;
}
S semiImplicitEuler(S s, Real dt) {
    S n; n.v = s.v + accel(s.x) * dt; n.x = s.x + n.v * dt; return n;
}
S rk4(S s, Real dt) {
    const Real k1x = s.v,                  k1v = accel(s.x);
    const Real k2x = s.v + k1v * dt * 0.5f, k2v = accel(s.x + k1x * dt * 0.5f);
    const Real k3x = s.v + k2v * dt * 0.5f, k3v = accel(s.x + k2x * dt * 0.5f);
    const Real k4x = s.v + k3v * dt,        k4v = accel(s.x + k3x * dt);
    S n;
    n.x = s.x + (k1x + 2 * k2x + 2 * k3x + k4x) * (dt / 6.0f);
    n.v = s.v + (k1v + 2 * k2v + 2 * k3v + k4v) * (dt / 6.0f);
    return n;
}
}  // namespace

TEST_CASE("Explicit Euler injects energy into an oscillator") {
    S s{1.0f, 0.0f};
    const Real E0 = energy(s);
    for (int i = 0; i < 2000; ++i) s = explicitEuler(s, 1.0f / 240.0f);
    // It should have gained a clearly measurable amount of energy.
    CHECK(energy(s) > E0 * 1.05f);
}

TEST_CASE("Semi-implicit Euler nearly conserves energy") {
    S s{1.0f, 0.0f};
    const Real E0 = energy(s);
    Real maxE = E0, minE = E0;
    for (int i = 0; i < 2000; ++i) {
        s = semiImplicitEuler(s, 1.0f / 240.0f);
        maxE = std::max(maxE, energy(s));
        minE = std::min(minE, energy(s));
    }
    // Energy only oscillates within a small bounded band — it does not drift.
    CHECK(maxE < E0 * 1.10f);
    CHECK(minE > E0 * 0.90f);
}

TEST_CASE("RK4 tracks the exact solution accurately") {
    // Exact: x(t) = cos(omega t), omega = sqrt(k). Integrate one full period and
    // check we returned near the start.
    const Real omega = std::sqrt(k);
    const Real period = 2.0f * p3d::kPi / omega;
    const Real dt = 1.0f / 240.0f;
    S s{1.0f, 0.0f};
    Real t = 0;
    while (t < period) { s = rk4(s, dt); t += dt; }
    CHECK(std::fabs(s.x - 1.0f) < 1e-2f);   // back near x = 1
    CHECK(std::fabs(s.v) < 1e-1f);          // and near v = 0
}
