// ===========================================================================
//  Chapter 6 — Particles & Forces
// ---------------------------------------------------------------------------
//  Goal: put Newton's laws to work. A particle is a point mass; a FORCE changes
//  its motion via F = m·a. We build a fountain: hundreds of particles launched
//  upward, pulled down by gravity, slowed by air drag, bouncing off the floor.
//
//  The pattern to notice — the "force accumulator" — is how every force-based
//  physics engine is organised. Each step, for every particle:
//     1. clear last step's forces,
//     2. let each force generator ADD its contribution (they just sum),
//     3. integrate once (semi-implicit Euler from Chapter 5),
//     4. handle collisions.
//  Forces never integrate themselves; they only add to the pile. That clean
//  separation is what lets you combine gravity + drag + wind by just calling
//  three functions.
//
//  Engine pieces used: physics/particle.hpp, physics/forces.hpp.
//  Companion page: docs/chapters/06-particles.html
// ===========================================================================
#include "common/app.hpp"
#include "common/render/debug_draw.hpp"
#include "physics/particle.hpp"
#include "physics/forces.hpp"

#include <random>
#include <vector>

using namespace p3d;

struct ParticlesDemo : App {
    static constexpr int kCount = 220;
    static constexpr Real kGroundY = 0.0f;

    std::vector<Particle> particles;
    std::vector<Real>     life;        // seconds remaining before respawn
    Mesh droplet = mesh::sphere(0.09f, 6, 8);   // low-poly: we draw hundreds

    std::mt19937 rng{1337};            // fixed seed → deterministic (see Ch 5)

    ParticlesDemo() {
        camera.target   = {0, 2.0f, 0};
        camera.distance = 11;
        camera.pitch    = radians(12);
    }

    std::string caption() const override { return "Ch6: Particles & Forces (a fountain)"; }

    // Give a particle a fresh launch from the nozzle: a randomised upward cone.
    void respawn(Particle& p, Real& l) {
        std::uniform_real_distribution<Real> ang(0, kTwoPi);
        std::uniform_real_distribution<Real> spread(0, 0.5f);
        std::uniform_real_distribution<Real> speed(6.5f, 8.5f);
        std::uniform_real_distribution<Real> lifespan(2.0f, 4.0f);

        const Real a = ang(rng), s = spread(rng), v = speed(rng);
        p.position = {0, 0.15f, 0};
        p.velocity = {std::cos(a) * s * v, v, std::sin(a) * s * v};
        p.setMass(1.0f);
        p.clearForces();
        l = lifespan(rng);
    }

    void onReset() override {
        particles.assign(kCount, Particle{});
        life.assign(kCount, 0.0f);
        // Stagger the initial lifetimes so droplets don't all launch in unison.
        std::uniform_real_distribution<Real> stagger(0, 3.0f);
        for (int i = 0; i < kCount; ++i) { respawn(particles[i], life[i]); life[i] = stagger(rng); }
    }

    void fixedUpdate(Real dt) override {
        const int v = sceneVariant();
        const Vec3 g{0, -9.81f, 0};
        // Scene 3 turns drag off so you can see the difference; scene 2 adds wind.
        const bool useDrag = (v != 3);
        const Vec3 wind = (v == 2) ? Vec3{3.5f, 0, 0} : Vec3{};

        for (int i = 0; i < kCount; ++i) {
            Particle& p = particles[i];

            // --- The force accumulator pattern ---
            p.clearForces();
            force::gravity(p, g);
            if (useDrag) force::drag(p, 0.06f, 0.02f);
            if (lengthSq(wind) > 0) p.addForce(wind);   // a steady sideways push
            p.integrateForces(dt);

            // --- Collide with the floor: reflect and lose some energy ---
            if (p.position.y < kGroundY) {
                p.position.y = kGroundY;
                p.velocity.y = -p.velocity.y * 0.35f;    // bounce, restitution 0.35
                p.velocity.x *= 0.7f;                    // a little floor friction
                p.velocity.z *= 0.7f;
            }

            // --- Lifetime: recycle dead droplets back to the nozzle ---
            life[i] -= dt;
            if (life[i] <= 0) respawn(p, life[i]);
        }
    }

    void render(Renderer3D& r, Real) override {
        debug::grid(r, 8, 16);

        // Tint each droplet by speed: slow = cool teal, fast = warm amber.
        for (const Particle& p : particles) {
            const Real speed = length(p.velocity);
            const Real t = clamp(speed / 9.0f, 0, 1);
            const Color c = lerp3(palette::sky, palette::amber, t);
            r.drawMesh(droplet, translation(p.position), c);
        }
    }

    // Small helper: blend two colors (Color has no lerp of its own).
    static Color lerp3(const Color& a, const Color& b, Real t) {
        return {lerp(a.r, b.r, t), lerp(a.g, b.g, t), lerp(a.b, b.b, t), 1};
    }

    std::vector<std::string> hudLines() const override {
        return {"1 fountain   2 + wind   3 no drag (compare the arcs)",
                std::string("particles=") + std::to_string(kCount)};
    }
};

int main(int argc, char** argv) { return ParticlesDemo{}.run(argc, argv); }
