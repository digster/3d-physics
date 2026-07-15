// ===========================================================================
//  test_softbody.cpp — Part VII: the XPBD soft body
// ---------------------------------------------------------------------------
//  The soft body must be stable (never explode), respect the floor, keep its
//  shape at rest, and — the whole point of compliance — squish MORE when softer.
// ===========================================================================
#include <doctest/doctest.h>
#include "physics/softbody.hpp"

#include <cmath>

using namespace p3d;

// Height (max−min y) of the cube's particles — how tall it currently stands.
static Real cubeHeight(const SoftBody& sb) {
    Real lo = 1e9f, hi = -1e9f;
    for (const Particle& p : sb.particles) { lo = std::min(lo, p.position.y); hi = std::max(hi, p.position.y); }
    return hi - lo;
}
static Real cubeBottom(const SoftBody& sb) {
    Real lo = 1e9f;
    for (const Particle& p : sb.particles) lo = std::min(lo, p.position.y);
    return lo;
}
static bool allFinite(const SoftBody& sb) {
    for (const Particle& p : sb.particles)
        if (!std::isfinite(p.position.x) || !std::isfinite(p.position.y) || !std::isfinite(p.position.z))
            return false;
    return true;
}

TEST_CASE("A dropped jello cube stays stable and settles on the floor") {
    SoftBody sb;
    sb.buildCube({0, 2.0f, 0}, 1.0f, 4, 0.02f, 3.0f);
    for (int i = 0; i < 400; ++i) sb.step(1.0f / 60.0f, 15, {0, -9.81f, 0}, 0.0f);
    CHECK(allFinite(sb));                       // never blew up
    CHECK(cubeBottom(sb) > -0.05f);             // didn't sink through the floor
    CHECK(cubeBottom(sb) < 0.1f);               // and it's resting ON the floor
    // It squished on impact but sprang back close to its original 1.0 height.
    CHECK(cubeHeight(sb) > 0.85f);
}

TEST_CASE("A softer cube squishes more on impact than a stiffer one") {
    auto minHeightOnImpact = [](Real compliance) {
        SoftBody sb;
        sb.buildCube({0, 2.0f, 0}, 1.0f, 4, compliance, 3.0f);
        Real lowest = 1e9f;
        for (int i = 0; i < 300; ++i) {
            sb.step(1.0f / 60.0f, 15, {0, -9.81f, 0}, 0.0f);
            if (i > 20 && i < 160) lowest = std::min(lowest, cubeHeight(sb));
        }
        return lowest;
    };
    const Real stiff = minHeightOnImpact(0.002f);   // barely deforms
    const Real soft  = minHeightOnImpact(0.03f);    // squishes noticeably
    CHECK(stiff > 0.85f);            // the stiff cube keeps most of its height
    CHECK(soft  < stiff - 0.2f);     // the soft one flattens much more
    CHECK(soft  > 0.1f);             // ...but still recovers, not permanently crushed
}

TEST_CASE("Compliance is stable across substep counts (XPBD's promise)") {
    // XPBD's stiffness shouldn't wildly depend on the substep count. A cube run
    // at 5 vs 20 substeps should settle to a similar height.
    auto settledHeight = [](int substeps) {
        SoftBody sb;
        sb.buildCube({0, 0.6f, 0}, 1.0f, 4, 0.02f, 3.0f);
        for (int i = 0; i < 400; ++i) sb.step(1.0f / 60.0f, substeps, {0, -9.81f, 0}, 0.0f);
        return cubeHeight(sb);
    };
    const Real few  = settledHeight(5);
    const Real many = settledHeight(20);
    CHECK(std::fabs(few - many) < 0.1f);   // similar stiffness regardless of substeps
}
