// ===========================================================================
//  Chapter 7 — Springs & Dampers
// ---------------------------------------------------------------------------
//  Goal: connect particles with springs, and understand DAMPING. A spring pulls
//  proportionally to how far it is stretched (Hooke's law, F = -k·x). Left
//  alone it oscillates forever; add a DAMPER — a force that resists the speed of
//  stretching — and the motion dies away. How fast, and whether it overshoots,
//  depends on the damping amount:
//     * under-damped   → overshoots and wobbles before settling (coral)
//     * critically damped → returns as fast as possible, no overshoot (teal)
//     * over-damped     → returns slowly, sluggish (violet)
//  Scene 2 shows all three racing back to the same target line.
//
//  Engine pieces: physics/particle.hpp, physics/forces.hpp (spring, anchoredSpring).
//  Companion page: docs/chapters/07-springs.html
// ===========================================================================
#include "common/app.hpp"
#include "common/render/debug_draw.hpp"
#include "physics/particle.hpp"
#include "physics/forces.hpp"

#include <vector>

using namespace p3d;

struct SpringsDemo : App {
    struct Anchored { int p; Vec3 anchor; Real k, rest, c; };
    struct Link     { int a, b; Real k, rest, c; };

    std::vector<Particle> particles;
    std::vector<Anchored> anchored;
    std::vector<Link>     links;
    std::vector<Color>    colors;
    bool useGravity = true;

    Mesh bob = mesh::sphere(0.16f, 8, 12);

    SpringsDemo() {
        camera.target   = {0, 2.0f, 0};
        camera.distance = 12;
    }

    std::string caption() const override { return "Ch7: Springs & Dampers"; }

    void onReset() override {
        particles.clear(); anchored.clear(); links.clear(); colors.clear();
        const int v = sceneVariant();

        if (v == 2) {
            // --- The damping comparison ---
            // Three identical springs (k, m) with three damping values. No
            // gravity, so the only target is the spring's rest position; the
            // masses start pulled down and race back to it.
            useGravity = false;
            const Real k = 40, rest = 2.0f, m = 1.0f;
            const Real cCrit = 2.0f * std::sqrt(k * m);   // critical damping
            const Real cs[3] = {1.5f, cCrit, 3.0f * cCrit};
            const Color cols[3] = {palette::coral, palette::teal, palette::violet};
            for (int i = 0; i < 3; ++i) {
                const Real x = (i - 1) * 2.5f;
                Particle p; p.setMass(m);
                p.position = {x, 0.0f, 0};                // pulled down to y=0...
                Vec3 anchor{x, 4.0f, 0};                  // ...anchor 4 up, rest 2
                particles.push_back(p);
                anchored.push_back({i, anchor, k, rest, cs[i]});
                colors.push_back(cols[i]);
            }
        } else if (v == 3) {
            // --- A spring chain: a soft pendulum, a preview of ropes ---
            useGravity = true;
            const int n = 8;
            const Vec3 top{0, 5.0f, 0};
            for (int i = 0; i < n; ++i) {
                Particle p; p.setMass(i == 0 ? 0 : 1.0f);  // top link pinned
                // Lay the chain out horizontally so it swings down dramatically.
                p.position = {Real(i) * 0.7f, 5.0f, 0};
                if (i == 0) p.position = top;
                particles.push_back(p);
                colors.push_back(i == 0 ? palette::slate : palette::teal);
            }
            for (int i = 0; i < n - 1; ++i)
                links.push_back({i, i + 1, 220.0f, 0.7f, 2.0f});
        } else {
            // --- A single spring pendulum: bobs AND swings ---
            useGravity = true;
            Particle p; p.setMass(1.0f);
            p.position = {2.2f, 2.0f, 0};                 // offset so it swings too
            particles.push_back(p);
            anchored.push_back({0, {0, 5.0f, 0}, 30.0f, 2.0f, 0.4f});
            colors.push_back(palette::amber);
        }
    }

    void fixedUpdate(Real dt) override {
        const Vec3 g{0, -9.81f, 0};
        for (Particle& p : particles) {
            p.clearForces();
            if (useGravity) force::gravity(p, g);
        }
        for (const Anchored& a : anchored)
            force::anchoredSpring(particles[a.p], a.anchor, a.k, a.rest, a.c);
        for (const Link& l : links)
            force::spring(particles[l.a], particles[l.b], l.k, l.rest, l.c);
        for (Particle& p : particles) p.integrateForces(dt);
    }

    // Draw a spring as a little zig-zag coil so it reads as a spring, not a stick.
    static void drawSpring(Renderer3D& r, const Vec3& a, const Vec3& b, int coils,
                           const Color& col) {
        const Vec3 axis = b - a;
        const Real len = length(axis);
        if (len < 1e-4f) return;
        const Vec3 dir = axis * (Real(1) / len);
        // A perpendicular direction to zig-zag along.
        const Vec3 ref = std::fabs(dir.y) < 0.9f ? axis::Y : axis::X;
        const Vec3 side = normalize(cross(dir, ref));
        const int segs = coils * 2;
        Vec3 prev = a;
        for (int i = 1; i <= segs; ++i) {
            const Real t = Real(i) / segs;
            const Real off = (i % 2 == 0) ? 0.0f : 0.14f * ((i & 2) ? 1 : -1);
            Vec3 pt = a + dir * (len * t) + side * off;
            if (i == segs) pt = b;
            r.drawLine(prev, pt, col);
            prev = pt;
        }
    }

    void render(Renderer3D& r, Real) override {
        debug::grid(r, 8, 16);

        for (const Anchored& a : anchored) {
            drawSpring(r, a.anchor, particles[a.p].position, 8, palette::slate);
            debug::point(r, a.anchor, 0.08f, palette::white);
            // In the damping scene, mark the target (rest) position so overshoot
            // is obvious: the mass should settle exactly on this dot.
            if (sceneVariant() == 2)
                debug::point(r, a.anchor - Vec3{0, a.rest, 0}, 0.1f, Color::bytes(120,130,150));
        }
        for (const Link& l : links)
            drawSpring(r, particles[l.a].position, particles[l.b].position, 5, palette::slate);

        for (size_t i = 0; i < particles.size(); ++i)
            r.drawMesh(bob, translation(particles[i].position), colors[i]);
    }

    std::vector<std::string> hudLines() const override {
        if (sceneVariant() == 2)
            return {"1 pendulum   2 damping race   3 spring chain",
                    "coral=under-damped  teal=critical  violet=over-damped",
                    "grey dot = target; watch who overshoots it"};
        return {"1 spring pendulum   2 damping race   3 spring chain",
                "Hooke's law: F = -k * stretch"};
    }
};

int main(int argc, char** argv) { return SpringsDemo{}.run(argc, argv); }
