// ===========================================================================
//  Chapter 5 — Numerical Integration
// ---------------------------------------------------------------------------
//  Goal: understand how we step a simulation forward in time, and why the
//  choice of method matters. "Integration" here means: given where something
//  is and how fast it is going, advance it by a small time step dt. There are
//  several ways to do that, and they differ dramatically in ACCURACY and
//  ENERGY BEHAVIOUR.
//
//  We race three integrators on the same problem, side by side:
//      * Explicit (forward) Euler   — coral. Simple, but LEAKS energy: on an
//        orbit it spirals outward, and with a big step it blows up.
//      * Semi-implicit (symplectic) Euler — teal. One tiny reordering of Euler
//        that (almost) conserves energy. This is what game engines use.
//      * Runge–Kutta 4 (RK4)        — lime. Four samples per step; very
//        accurate, more expensive.
//
//  Scenes (keys 1–3):
//      1  Orbit under a central spring force a = -k·x. Watch coral drift out
//         while teal and lime hold their orbit — the classic energy-drift plot,
//         but as motion. The HUD shows each integrator's total energy.
//      2  Projectile under gravity, compared against the EXACT parabola
//         (white). Constant acceleration is easy, so all three track well.
//      3  Same orbit as (1) but with a big time step — now explicit Euler blows
//         up almost immediately. Stability depends on dt.
//
//  The integrators are written out in full below because THIS chapter is about
//  them. Companion page: docs/chapters/05-integration.html
// ===========================================================================
#include "common/app.hpp"
#include "common/render/debug_draw.hpp"

#include <array>
#include <deque>
#include <functional>

using namespace p3d;

// A point-mass state: where it is and how fast it moves.
struct State { Vec3 pos, vel; };

// An acceleration field a(x, v). For our scenes it depends only on position,
// but we pass velocity too so the pattern generalises (drag, for instance).
using AccelFn = std::function<Vec3(const Vec3&, const Vec3&)>;

// --- The integrators -------------------------------------------------------
// Each takes a state, an acceleration field, and a step dt, and returns the
// advanced state. Compare them line by line — the differences are tiny in code
// and enormous in behaviour.

State stepExplicitEuler(State s, const AccelFn& a, Real dt) {
    const Vec3 acc = a(s.pos, s.vel);
    s.pos += s.vel * dt;   // position uses the OLD velocity...
    s.vel += acc * dt;     // ...then velocity updates. This ordering leaks energy.
    return s;
}

State stepSemiImplicitEuler(State s, const AccelFn& a, Real dt) {
    const Vec3 acc = a(s.pos, s.vel);
    s.vel += acc * dt;     // update velocity FIRST...
    s.pos += s.vel * dt;   // ...then move with the NEW velocity. Symplectic.
    return s;
}

State stepRK4(State s, const AccelFn& a, Real dt) {
    // Sample the slope four times across the interval and take a weighted
    // average. For a second-order system the "slope" of position is velocity
    // and the "slope" of velocity is acceleration.
    const Vec3 k1x = s.vel,                         k1v = a(s.pos, s.vel);
    const Vec3 k2x = s.vel + k1v * (dt * 0.5f),     k2v = a(s.pos + k1x * (dt * 0.5f), s.vel + k1v * (dt * 0.5f));
    const Vec3 k3x = s.vel + k2v * (dt * 0.5f),     k3v = a(s.pos + k2x * (dt * 0.5f), s.vel + k2v * (dt * 0.5f));
    const Vec3 k4x = s.vel + k3v * dt,              k4v = a(s.pos + k3x * dt,          s.vel + k3v * dt);
    s.pos += (k1x + k2x * 2.0f + k3x * 2.0f + k4x) * (dt / 6.0f);
    s.vel += (k1v + k2v * 2.0f + k3v * 2.0f + k4v) * (dt / 6.0f);
    return s;
}

struct IntegrationDemo : App {
    static constexpr Real kSpring  = 4.0f;      // k in a = -k x
    static constexpr int  kN       = 3;         // number of integrators raced
    static constexpr size_t kTrail = 260;       // trail length (points)

    std::array<State, kN> bodies;
    std::array<std::deque<Vec3>, kN> trails;
    std::array<Color, kN> colors{palette::coral, palette::teal, palette::lime};
    std::array<const char*, kN> names{"Explicit Euler", "Semi-implicit", "RK4"};

    // Exact projectile reference (scene 2).
    State projStart;
    std::deque<Vec3> exactTrail;

    Mesh marker = mesh::sphere(0.12f, 8, 12);
    AccelFn accel;
    Real stepH = 0;   // per-tick step handed to the integrators

    std::string caption() const override { return "Ch5: Numerical Integration"; }

    void onReset() override {
        for (auto& tr : trails) tr.clear();
        exactTrail.clear();
        const int v = sceneVariant();

        if (v == 2) {
            // Projectile: launch up and to the right; compare to the exact arc.
            accel = [](const Vec3&, const Vec3&) { return Vec3{0, -9.81f, 0}; };
            const State init{{-5, 0.2f, 0}, {4.5f, 8.5f, 0}};
            for (auto& b : bodies) b = init;
            projStart = init;
            stepH = fixedDt();
            camera.target = {0, 2.5f, 0}; camera.distance = 16;
            camera.yaw = 0; camera.pitch = radians(6);
        } else {
            // Orbit: circular motion needs speed v = omega * r with omega=sqrt(k).
            accel = [](const Vec3& x, const Vec3&) { return x * (-kSpring); };
            const Real r = 2.5f, omega = std::sqrt(kSpring);
            const State init{{r, 0, 0}, {0, 0, omega * r}};
            for (auto& b : bodies) b = init;
            stepH = (v == 3) ? 0.06f : fixedDt();   // scene 3 uses a big, unstable step
            camera.target = {0, 0, 0}; camera.distance = 11;
            camera.yaw = radians(20); camera.pitch = radians(55);
        }
    }

    void fixedUpdate(Real) override {
        // Advance each integrator with the same field and step, and record a
        // trail point so the divergence is visible as a path, not just a dot.
        bodies[0] = stepExplicitEuler(bodies[0], accel, stepH);
        bodies[1] = stepSemiImplicitEuler(bodies[1], accel, stepH);
        bodies[2] = stepRK4(bodies[2], accel, stepH);

        for (int i = 0; i < kN; ++i) {
            trails[i].push_back(bodies[i].pos);
            if (trails[i].size() > kTrail) trails[i].pop_front();
        }
        if (sceneVariant() == 2) {
            // Exact solution of constant-acceleration motion: x = x0 + v0 t + ½ a t².
            const Real T = simTime();
            const Vec3 g{0, -9.81f, 0};
            const Vec3 exact = projStart.pos + projStart.vel * T + g * (0.5f * T * T);
            exactTrail.push_back(exact);
            if (exactTrail.size() > kTrail) exactTrail.pop_front();
        }
    }

    static void drawTrail(Renderer3D& r, const std::deque<Vec3>& tr, const Color& c) {
        for (size_t i = 1; i < tr.size(); ++i) r.drawLine(tr[i - 1], tr[i], c);
    }

    void render(Renderer3D& r, Real) override {
        debug::grid(r, 8, 16);
        debug::axes(r, Vec3{}, 0.8f);

        // The exact reference first (scene 2), so the integrators draw over it.
        if (sceneVariant() == 2) {
            drawTrail(r, exactTrail, Color::bytes(200, 205, 215));
            r.drawMesh(marker, translation(exactTrail.empty() ? projStart.pos : exactTrail.back()),
                       palette::white);
        }
        for (int i = 0; i < kN; ++i) {
            drawTrail(r, trails[i], colors[i].shaded(0.8f));
            r.drawMesh(marker, translation(bodies[i].pos), colors[i]);
        }
    }

    std::vector<std::string> hudLines() const override {
        std::vector<std::string> out;
        if (sceneVariant() == 2) {
            out.emplace_back("scene 2: dots chase the exact white parabola");
        } else {
            // Total mechanical energy per integrator: ½|v|² + ½k|x|² (unit mass).
            // Watch Explicit Euler's climb — that upward drift IS the spiral.
            for (int i = 0; i < kN; ++i) {
                const Real E = 0.5f * lengthSq(bodies[i].vel) +
                               0.5f * kSpring * lengthSq(bodies[i].pos);
                char buf[96];
                std::snprintf(buf, sizeof(buf), "%-15s E=%.3f", names[i], double(E));
                out.emplace_back(buf);
            }
        }
        out.emplace_back("1 orbit   2 projectile vs exact   3 orbit (big step!)");
        return out;
    }
};

int main(int argc, char** argv) { return IntegrationDemo{}.run(argc, argv); }
