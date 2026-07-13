// ===========================================================================
//  Chapter 12 — Broadphase
// ---------------------------------------------------------------------------
//  Goal: find, cheaply, which pairs of bodies might be colliding — before doing
//  any expensive exact test. With N bodies there are ~N²/2 possible pairs, far
//  too many to test exactly every frame. Broadphase uses only loose AABBs to
//  throw away the pairs that are nowhere near each other, leaving a short list of
//  CANDIDATES for the narrowphase (Chapters 13–15) to examine closely.
//
//  A cloud of spheres drifts and bounces around a box. Each frame we build their
//  world AABBs and run a broadphase; the candidate pairs it returns are drawn as
//  amber links (with their AABBs), and pairs that are ACTUALLY overlapping are
//  drawn coral. The HUD compares the candidate count against the brute-force
//  O(N²) count — a broadphase does the same job with a fraction of the tests.
//
//  Scenes switch the strategy: 1 uniform grid, 2 sweep-and-prune, 3 brute force.
//  All three flag the same pairs (that's the point). Engine: physics/aabb.hpp,
//  physics/broadphase.hpp. Companion page: docs/chapters/12-broadphase.html
// ===========================================================================
#include "common/app.hpp"
#include "common/render/debug_draw.hpp"
#include "physics/aabb.hpp"
#include "physics/broadphase.hpp"

#include <random>
#include <vector>

using namespace p3d;

struct BroadphaseDemo : App {
    static constexpr int   kCount  = 140;
    static constexpr Real  kDomain = 5.0f;       // bodies bounce inside ±kDomain

    std::vector<Vec3> pos, vel;
    std::vector<Real> radius;
    Mesh ball = mesh::sphere(0.35f, 5, 7);       // unit-ish; scaled per body
    std::mt19937 rng{2024};

    BroadphaseDemo() {
        camera.target   = {0, 0, 0};
        camera.distance = 18;
        camera.pitch    = radians(22);
    }

    std::string caption() const override { return "Ch12: Broadphase"; }

    void onReset() override {
        pos.assign(kCount, {}); vel.assign(kCount, {}); radius.assign(kCount, 0.3f);
        std::uniform_real_distribution<Real> p(-kDomain, kDomain);
        std::uniform_real_distribution<Real> v(-2.5f, 2.5f);
        std::uniform_real_distribution<Real> r(0.22f, 0.5f);
        for (int i = 0; i < kCount; ++i) {
            pos[i] = {p(rng), p(rng), p(rng)};
            vel[i] = {v(rng), v(rng), v(rng)};
            radius[i] = r(rng);
        }
    }

    void fixedUpdate(Real dt) override {
        // Pure kinematic drift, reflecting off the domain walls. (No body-body
        // collision response — this chapter is only about FINDING the pairs.)
        for (int i = 0; i < kCount; ++i) {
            pos[i] += vel[i] * dt;
            for (int a = 0; a < 3; ++a) {
                const Real lim = kDomain - radius[i];
                if (pos[i][a] >  lim) { pos[i][a] =  lim; vel[i][a] = -vel[i][a]; }
                if (pos[i][a] < -lim) { pos[i][a] = -lim; vel[i][a] = -vel[i][a]; }
            }
        }
    }

    void render(Renderer3D& r, Real) override {
        debug::aabb(r, Vec3(-kDomain), Vec3(kDomain), Color::bytes(70, 76, 92));

        // Build one AABB per body.
        std::vector<AABB> boxes(kCount);
        for (int i = 0; i < kCount; ++i) boxes[i] = aabbOfSphere(pos[i], radius[i]);

        // Draw the bodies themselves.
        for (int i = 0; i < kCount; ++i)
            r.drawMesh(ball, translation(pos[i]) * scaling(Vec3(radius[i] / 0.35f)),
                       palette::slate);

        // Run the chosen broadphase.
        std::vector<BroadPair> pairs;
        switch (sceneVariant()) {
            case 2:  pairs = broadphaseSweep(boxes); break;
            case 3:  pairs = broadphaseBrute(boxes); break;
            default: pairs = broadphaseGrid(boxes, 1.4f); break;
        }

        // Draw each candidate pair: link their centres, and (for the truly
        // overlapping ones) colour the link coral and show the AABBs.
        for (const BroadPair& pr : pairs) {
            const bool touching =
                distance(pos[pr.first], pos[pr.second]) < radius[pr.first] + radius[pr.second];
            const Color c = touching ? palette::coral : Color::bytes(180, 150, 70);
            r.drawLine(pos[pr.first], pos[pr.second], c);
            if (touching) {
                debug::aabb(r, boxes[pr.first].min, boxes[pr.first].max, palette::coral);
                debug::aabb(r, boxes[pr.second].min, boxes[pr.second].max, palette::coral);
            }
        }
        lastCandidates_ = static_cast<int>(pairs.size());
        lastBrute_      = static_cast<int>(broadphaseBrute(boxes).size());
    }

    std::vector<std::string> hudLines() const override {
        const char* names[4] = {"", "uniform grid", "sweep & prune", "brute force O(N^2)"};
        const int v = sceneVariant();
        char buf[128];
        const long allPairs = long(kCount) * (kCount - 1) / 2;
        std::snprintf(buf, sizeof(buf), "%s: %d candidate pairs (of %ld possible)",
                      names[(v >= 1 && v <= 3) ? v : 1], lastCandidates_, allPairs);
        char buf2[96];
        std::snprintf(buf2, sizeof(buf2), "bodies=%d   overlaps found by brute=%d (same set)",
                      kCount, lastBrute_);
        return {buf, buf2, "1 grid   2 sweep&prune   3 brute   (amber=candidate, coral=touching)"};
    }

    int lastCandidates_ = 0, lastBrute_ = 0;
};

int main(int argc, char** argv) { return BroadphaseDemo{}.run(argc, argv); }
