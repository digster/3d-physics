// ===========================================================================
//  Chapter 8 — Ropes & Constraints
// ---------------------------------------------------------------------------
//  Goal: a completely different way to hold things together. Instead of a spring
//  FORCE that we hope keeps two beads the right distance apart, we let them move
//  freely (Verlet integration) and then just MOVE them back to the correct
//  distance. That "move them back" is a position CONSTRAINT, and solving a whole
//  chain of them — repeatedly, until they agree — gives a rope that simply
//  cannot stretch explosively the way a stiff spring can.
//
//  This is Thomas Jakobsen's method. The loop each step is:
//     1. Verlet-integrate every bead under gravity,
//     2. satisfy all distance constraints, a few ITERATIONS in a row.
//  More iterations ⇒ a stiffer, less stretchy rope. Press [ and ] to change the
//  iteration count live and watch the rope tighten up.
//
//  Engine pieces: physics/particle.hpp (Verlet), physics/constraint.hpp.
//  Companion page: docs/chapters/08-ropes.html
// ===========================================================================
#include "common/app.hpp"
#include "common/render/debug_draw.hpp"
#include "physics/particle.hpp"
#include "physics/constraint.hpp"

#include <vector>

using namespace p3d;

struct RopesDemo : App {
    std::vector<Particle>          particles;
    std::vector<DistanceConstraint> constraints;
    std::vector<bool>              pinned;
    int  heavyIndex = -1;         // a weighted bead, for the pendulum scene
    int  iterations = 12;         // constraint relaxation passes per step

    Mesh bead    = mesh::sphere(0.08f, 6, 8);
    Mesh weight  = mesh::sphere(0.35f, 10, 14);

    RopesDemo() {
        camera.target   = {0, 2.0f, 0};
        camera.distance = 12;
    }

    std::string caption() const override { return "Ch8: Ropes & Constraints (Verlet)"; }

    // Add a bead at p; static=true makes it an immovable pin (invMass 0).
    int addBead(const Vec3& pos, bool isStatic) {
        Particle p; p.position = pos; p.setMass(1.0f);
        if (isStatic) p.makeStatic();
        p.syncVerlet();                    // prevPosition = position (start at rest)
        particles.push_back(p);
        pinned.push_back(isStatic);
        return static_cast<int>(particles.size()) - 1;
    }

    void onReset() override {
        particles.clear(); constraints.clear(); pinned.clear();
        heavyIndex = -1;
        const int v = sceneVariant();
        const Real seg = 0.35f;

        if (v == 2) {
            // Necklace: pinned at BOTH ends, sags into a catenary curve.
            const int n = 22;
            const Vec3 left{-3.2f, 4.5f, 0}, right{3.2f, 4.5f, 0};
            for (int i = 0; i < n; ++i) {
                const Real t = Real(i) / (n - 1);
                addBead(lerp(left, right, t), i == 0 || i == n - 1);
            }
            for (int i = 0; i < n - 1; ++i)
                constraints.push_back({i, i + 1, distance(particles[i].position, particles[i+1].position), 1.0f});
        } else {
            // Single rope pinned at the top-left, laid out horizontally so it
            // swings down when released. Scene 3 hangs a heavy weight on the end.
            const int n = 18;
            const Vec3 start{-2.5f, 4.5f, 0};
            for (int i = 0; i < n; ++i)
                addBead(start + Vec3{Real(i) * seg, 0, 0}, i == 0);
            for (int i = 0; i < n - 1; ++i)
                constraints.push_back({i, i + 1, seg, 1.0f});
            if (v == 3) { heavyIndex = n - 1; particles[heavyIndex].setMass(12.0f); }
        }
    }

    // [ and ] change how many relaxation passes we do — i.e. rope stiffness.
    void onKey(SDL_Keycode key) override {
        if (key == SDLK_LEFTBRACKET)  iterations = (iterations > 1)  ? iterations - 1 : 1;
        if (key == SDLK_RIGHTBRACKET) iterations = (iterations < 40) ? iterations + 1 : 40;
    }

    void fixedUpdate(Real dt) override {
        const Vec3 g{0, -9.81f, 0};
        // 1. Integrate every bead freely under gravity (Verlet).
        for (Particle& p : particles) p.integrateVerlet(g, dt);
        // 2. Pull them back onto their constraints, a few passes for convergence.
        satisfyConstraints(constraints, particles, iterations);
    }

    void render(Renderer3D& r, Real) override {
        debug::grid(r, 8, 16);

        // The rope itself: a line per constraint.
        for (const DistanceConstraint& c : constraints)
            r.drawLine(particles[c.a].position, particles[c.b].position, palette::amber);

        for (size_t i = 0; i < particles.size(); ++i) {
            const Color col = pinned[i] ? palette::coral : palette::teal;
            if (static_cast<int>(i) == heavyIndex)
                r.drawMesh(weight, translation(particles[i].position), palette::violet);
            else
                r.drawMesh(bead, translation(particles[i].position), col);
        }
    }

    std::vector<std::string> hudLines() const override {
        char buf[96];
        std::snprintf(buf, sizeof(buf), "iterations=%d  ([ / ] to change) -> higher = stiffer", iterations);
        return {"1 rope   2 necklace (pinned both ends)   3 rope + heavy weight",
                buf, "red = pinned (immovable)"};
    }
};

int main(int argc, char** argv) { return RopesDemo{}.run(argc, argv); }
