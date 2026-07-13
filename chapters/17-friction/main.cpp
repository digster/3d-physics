// ===========================================================================
//  Chapter 17 — Friction & the Solver
// ---------------------------------------------------------------------------
//  Goal: the two ingredients that make a pile of boxes actually stand up.
//    * FRICTION — a sideways impulse that resists sliding, but only up to a
//      limit: μ times the normal push (the Coulomb cone). Grip harder than that
//      and the surfaces slip. Friction is what stops a stack from squirting
//      sideways.
//    * ITERATION — one pass of the sequential-impulse solver can't satisfy a
//      whole stack at once (fixing the top contact disturbs the one below).
//      Looping the solve a few times lets the impulses settle — and WARM
//      STARTING (re-using last frame's impulses) makes it converge far faster,
//      the difference between a rock-solid tower and a jittery one.
//
//  Scenes:
//    1  A tall tower of boxes.
//    2  A pyramid.
//    3  A friction ramp: three boxes with low / medium / high friction on a
//       tilted floor — the slippery one slides off, the grippy one holds.
//
//  Keys: [ and ] change solver iterations; F toggles warm starting. Drop the
//  iterations low or turn warm starting off and watch the stacks get wobbly.
//  Companion page: docs/chapters/17-friction.html
// ===========================================================================
#include "common/app.hpp"
#include "common/render/debug_draw.hpp"
#include "physics/world.hpp"

#include <vector>

using namespace p3d;

struct FrictionDemo : App {
    World world;
    Mesh cube = mesh::box({1, 1, 1});

    FrictionDemo() {
        camera.target   = {0, 2.0f, 0};
        camera.distance = 14;
        camera.pitch    = radians(10);
    }

    std::string caption() const override { return "Ch17: Friction & the Solver"; }

    int addBox(const Vec3& pos, const Vec3& half, Real mass, Real fric, const Quat& rot = Quat::identity()) {
        Collider c = Collider::box(half, 0.0f, fric);
        int idx = world.add(makeBody(c, pos, mass), c);
        world.rbs[idx].orientation = rot;
        return idx;
    }

    void onReset() override {
        world = World{};
        world.solver.iterations = 12;
        world.sleepingEnabled = false;   // keep everything live so you can watch it
        const int v = sceneVariant();

        if (v == 2) {
            addBox({0, -0.5f, 0}, {8, 0.5f, 8}, 0, 0.7f);            // floor
            // A 4-3-2-1 pyramid.
            const Real w = 0.55f;
            for (int row = 0; row < 4; ++row)
                for (int k = 0; k < 4 - row; ++k) {
                    const Real x = (k - (3 - row) * 0.5f) * (2 * w + 0.02f);
                    addBox({x, 0.5f + row * (2 * w), 0}, {w, w, w}, 1.0f, 0.7f);
                }
        } else if (v == 3) {
            // A tilted floor and three boxes of different friction.
            const Quat tilt = Quat::fromAxisAngle({0, 0, 1}, radians(18));
            addBox({0, -0.5f, 0}, {8, 0.5f, 8}, 0, 0.6f, tilt);
            const Real fr[3] = {0.05f, 0.4f, 1.2f};
            for (int i = 0; i < 3; ++i) {
                // Sit each box just above the incline surface.
                const Vec3 up = rotate(tilt, {0, 1, 0});
                addBox(Vec3{0, 2.2f, Real(i - 1) * 2.2f} + up * 0.55f, {0.5f, 0.5f, 0.5f}, 1.0f, fr[i], tilt);
            }
        } else {
            addBox({0, -0.5f, 0}, {8, 0.5f, 8}, 0, 0.7f);            // floor
            for (int k = 0; k < 7; ++k)                              // a tower
                addBox({0, 0.5f + k * 1.0f, 0}, {0.5f, 0.5f, 0.5f}, 1.0f, 0.7f);
        }
    }

    void onKey(SDL_Keycode key) override {
        if (key == SDLK_LEFTBRACKET)  world.solver.iterations = std::max(1, world.solver.iterations - 1);
        if (key == SDLK_RIGHTBRACKET) world.solver.iterations = std::min(40, world.solver.iterations + 1);
        if (key == SDLK_F)            world.solver.warmStart = !world.solver.warmStart;
    }

    void fixedUpdate(Real dt) override { world.step(dt); }

    void render(Renderer3D& r, Real) override {
        debug::grid(r, 8, 16);
        const int n = static_cast<int>(world.rbs.size());
        for (int i = 0; i < n; ++i) {
            const RigidBody& b = world.rbs[i];
            const Collider& c = world.colliders[i];
            const Color col = b.isStatic() ? Color::bytes(52, 58, 72)
                            : (sceneVariant() == 3 ? frictionColor(c.friction) : palette::amber);
            r.drawMesh(cube, translation(b.position) * toMat4(b.orientation) * scaling(c.half), col);
        }
    }

    static Color frictionColor(Real f) {
        // slippery = coral, grippy = teal.
        const Real t = clamp(f / 1.2f, 0, 1);
        return {lerp(0.94f, 0.16f, t), lerp(0.39f, 0.78f, t), lerp(0.37f, 0.74f, t), 1};
    }

    std::vector<std::string> hudLines() const override {
        char buf[96];
        std::snprintf(buf, sizeof(buf), "iterations=%d ([ ]),  warm start=%s (F)",
                      world.solver.iterations, world.solver.warmStart ? "ON" : "off");
        return {buf, "1 tower   2 pyramid   3 friction ramp (coral=slippery, teal=grippy)"};
    }
};

int main(int argc, char** argv) { return FrictionDemo{}.run(argc, argv); }
