// ===========================================================================
//  Chapter 11 — Rigid-Body Dynamics
// ---------------------------------------------------------------------------
//  Goal: give bodies the rotational half of Newton's laws — torque, the inertia
//  tensor, and angular momentum — and watch something genuinely surprising fall
//  out for free: the DZHANIBEKOV EFFECT (a.k.a. the tennis-racket / intermediate-
//  axis theorem). A rigid body has three special spin axes. Spin it about the one
//  with the smallest or largest moment of inertia and it spins steadily. Spin it
//  about the MIDDLE one and it periodically flips end-over-end — all on its own,
//  with no forces at all. It looks impossible; it's just conservation of angular
//  momentum plus a non-spherical inertia tensor.
//
//  Scenes:
//    1  A T-handle spinning about its intermediate axis: watch it flip, again and
//       again. The HUD shows |L| holding steady while ω wanders — that's the whole
//       secret.
//    2  Three T-handles, one per principal axis. Only the middle one misbehaves.
//    3  Forces at a point: a thruster fires off-centre, so torque = r × F both
//       pushes the body and spins it up.
//
//  Engine pieces: physics/rigidbody.hpp, physics/inertia.hpp.
//  Companion page: docs/chapters/11-rigidbody.html
// ===========================================================================
#include "common/app.hpp"
#include "common/render/debug_draw.hpp"
#include "physics/rigidbody.hpp"
#include "physics/inertia.hpp"

#include <algorithm>
#include <array>
#include <vector>

using namespace p3d;

struct RigidBodyDemo : App {
    // --- T-handle geometry (a shaft with a crossbar at one end) -----------
    Vec3 shaftHalf{0.16f, 0.85f, 0.16f};
    Vec3 crossHalf{0.70f, 0.16f, 0.16f};
    Vec3 shaftOffset, crossOffset;        // part centres relative to the COM
    Real totalMass = 1;
    Mat3 bodyInertia = Mat3::identity();
    std::array<int, 3> axisByMoment{0, 1, 2};   // indices sorted min→mid→max

    Mesh shaftMesh = mesh::box(shaftHalf);
    Mesh crossMesh = mesh::box(crossHalf);
    Mesh capMesh   = mesh::box({0.15f, 0.15f, 0.15f});

    std::vector<RigidBody> bodies;
    Real thrustTimer = 0;

    RigidBodyDemo() {
        buildTHandle();
        camera.target   = {0, 0.4f, 0};
        camera.distance = 8;
        camera.pitch    = radians(14);
    }

    std::string caption() const override { return "Ch11: Rigid-Body Dynamics"; }

    // Compute the composite centre of mass and inertia tensor of the T-handle,
    // once. Density is uniform, so a part's mass is proportional to its volume.
    void buildTHandle() {
        auto volume = [](const Vec3& h) { return 8 * h.x * h.y * h.z; };
        const Real mShaft = volume(shaftHalf);
        const Real mCross = volume(crossHalf);
        totalMass = mShaft + mCross;

        // Shaft centred at origin; crossbar sits at the top of the shaft.
        const Vec3 shaftCentre{0, 0, 0};
        const Vec3 crossCentre{0, shaftHalf.y, 0};
        const Vec3 com = (shaftCentre * mShaft + crossCentre * mCross) * (Real(1) / totalMass);
        shaftOffset = shaftCentre - com;    // part positions in the body (COM) frame
        crossOffset = crossCentre - com;

        // Parallel-axis theorem: each part's inertia about the COM, then summed.
        bodyInertia =
            inertia::shift(inertia::box(mShaft, shaftHalf), mShaft, shaftOffset) +
            inertia::shift(inertia::box(mCross, crossHalf), mCross, crossOffset);

        // Rank the three principal moments so we can address the min/mid/max axes.
        const Real m[3] = {bodyInertia.at(0,0), bodyInertia.at(1,1), bodyInertia.at(2,2)};
        axisByMoment = {0, 1, 2};
        std::sort(axisByMoment.begin(), axisByMoment.end(),
                  [&](int a, int b) { return m[a] < m[b]; });
    }

    RigidBody makeBody(const Vec3& pos) {
        RigidBody b;
        b.position = pos;
        b.setMass(totalMass);
        b.setInertia(bodyInertia);
        return b;
    }

    // Spin a body about one of its principal axes, with a tiny nudge off it so
    // the (in)stability has something to grow from — real spins are never perfect.
    void spinAbout(RigidBody& b, int axisIndex, Real speed) {
        Vec3 axisv{}, nudge{};
        axisv[axisIndex] = 1;
        nudge[(axisIndex + 1) % 3] = 0.05f;
        b.setAngularVelocity(axisv * speed + nudge);
    }

    void onReset() override {
        bodies.clear();
        thrustTimer = 0;
        const int v = sceneVariant();

        if (v == 2) {
            // One handle per principal axis, so the odd one out stands out.
            const int order[3] = {axisByMoment[0], axisByMoment[1], axisByMoment[2]}; // min, mid, max
            for (int i = 0; i < 3; ++i) {
                bodies.push_back(makeBody({Real(i - 1) * 3.3f, 0.4f, 0}));
                spinAbout(bodies.back(), order[i], 6.0f);
            }
        } else if (v == 3) {
            bodies.push_back(makeBody({0, 0.4f, 0}));   // starts at rest; a thruster spins it up
        } else {
            // The star: spin about the INTERMEDIATE axis and watch it flip.
            bodies.push_back(makeBody({0, 0.4f, 0}));
            spinAbout(bodies.back(), axisByMoment[1], 6.0f);
        }
    }

    void fixedUpdate(Real dt) override {
        if (sceneVariant() == 3 && !bodies.empty()) {
            // Fire a thruster off-centre for the first stretch of each run: a
            // force at a body point produces torque τ = r × F, so the handle
            // both drifts and spins up. Then it coasts.
            thrustTimer += dt;
            if (thrustTimer < 1.2f) {
                RigidBody& b = bodies[0];
                const Vec3 applyAt = b.worldPoint({0.7f, 0.85f, 0});   // tip of the crossbar
                b.applyForceAtPoint({0, 0, 6.0f}, applyAt);             // push along +z
            }
        }
        for (RigidBody& b : bodies) b.integrate(dt);
    }

    void drawHandle(Renderer3D& r, const RigidBody& b) {
        const Mat4 base = b.transform();
        r.drawMesh(shaftMesh, base * translation(shaftOffset), palette::teal);
        r.drawMesh(crossMesh, base * translation(crossOffset), palette::amber);
        // A bright cap on ONE crossbar tip so the flips are unmistakable.
        r.drawMesh(capMesh, base * translation(crossOffset + Vec3{crossHalf.x, 0, 0}), palette::coral);

        // Body-attached axes (they rotate with the body) — the clearest read on
        // orientation as it tumbles.
        const Vec3 p = b.position;
        debug::arrow(r, p, p + rotate(b.orientation, Vec3{1.1f, 0, 0}), palette::coral);
        debug::arrow(r, p, p + rotate(b.orientation, Vec3{0, 1.1f, 0}), palette::lime);
        debug::arrow(r, p, p + rotate(b.orientation, Vec3{0, 0, 1.1f}), palette::sky);
    }

    void render(Renderer3D& r, Real) override {
        debug::grid(r, 8, 16);
        for (const RigidBody& b : bodies) drawHandle(r, b);

        if (sceneVariant() == 3 && thrustTimer < 1.2f && !bodies.empty()) {
            // Show the thruster force we're applying, as an arrow at its point.
            const Vec3 at = bodies[0].worldPoint({0.7f, 0.85f, 0});
            debug::arrow(r, at, at + Vec3{0, 0, 1.0f}, palette::white);
        }
    }

    std::vector<std::string> hudLines() const override {
        std::vector<std::string> out;
        if (!bodies.empty()) {
            const RigidBody& b = bodies[0];
            char buf[96];
            // |L| stays put; |ω| wanders. That contrast IS the effect.
            std::snprintf(buf, sizeof(buf), "|L|=%.3f (conserved)   |omega|=%.2f",
                          double(length(b.angularMomentum)), double(length(b.angularVelocity())));
            out.emplace_back(buf);
        }
        if (sceneVariant() == 2)
            out.emplace_back("left=min axis  middle=intermediate (flips!)  right=max axis");
        else if (sceneVariant() == 3)
            out.emplace_back("thruster off-centre -> torque = r x F -> it spins AND drifts");
        else
            out.emplace_back("spinning about the intermediate axis -> periodic flips");
        out.emplace_back("1 Dzhanibekov   2 three axes   3 force at a point");
        return out;
    }
};

int main(int argc, char** argv) { return RigidBodyDemo{}.run(argc, argv); }
