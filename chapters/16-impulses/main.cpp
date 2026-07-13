// ===========================================================================
//  Chapter 16 — Impulses & Restitution
// ---------------------------------------------------------------------------
//  Goal: make contacts push back. A detected contact (Part IV) does nothing on
//  its own; the solver turns it into an IMPULSE — an instantaneous change in
//  momentum — that stops the bodies interpenetrating and, if the surface is
//  bouncy, sends them back. How bouncy is the RESTITUTION: 0 = a dead thud,
//  1 = a perfect, energy-preserving bounce.
//
//  Scenes:
//    1  A row of balls with restitution 0.0, 0.3, 0.6, 0.9 — dropped together,
//       they bounce to wildly different heights. Restitution, made visible.
//    2  A bin of bouncy balls sloshing around.
//    3  Bouncing boxes — the same impulse math, now with rotation from the
//       off-centre contacts.
//
//  This is our first use of the full World (physics/world.hpp): it runs the
//  whole pipeline — gravity, broadphase, narrowphase, solve, integrate.
//  Companion page: docs/chapters/16-impulses.html
// ===========================================================================
#include "common/app.hpp"
#include "common/render/debug_draw.hpp"
#include "physics/world.hpp"

#include <vector>

using namespace p3d;

struct ImpulsesDemo : App {
    World world;
    Mesh ball = mesh::sphere(1.0f, 12, 16);
    Mesh cube = mesh::box({1, 1, 1});
    Mesh slab = mesh::box({1, 1, 1});

    ImpulsesDemo() {
        camera.target   = {0, 1.5f, 0};
        camera.distance = 14;
        camera.pitch    = radians(10);
    }

    std::string caption() const override { return "Ch16: Impulses & Restitution"; }

    // A wide static floor whose top surface is at y = 0.
    void addFloor(Real restitution) {
        Collider c = Collider::box({12, 0.5f, 12}, restitution, 0.6f);
        world.add(makeBody(c, {0, -0.5f, 0}, 0), c);
    }

    void onReset() override {
        world = World{};
        world.solver.iterations = 8;
        const int v = sceneVariant();

        if (v == 2) {
            addFloor(0.1f);
            // walls (static boxes) to keep the balls in a bin
            for (int s = -1; s <= 1; s += 2) {
                Collider wx = Collider::box({0.4f, 3, 6}, 0.4f, 0.4f);
                world.add(makeBody(wx, {s * 6.0f, 3, 0}, 0), wx);
                Collider wz = Collider::box({6, 3, 0.4f}, 0.4f, 0.4f);
                world.add(makeBody(wz, {0, 3, s * 6.0f}, 0), wz);
            }
            for (int i = 0; i < 24; ++i) {
                Collider bc = Collider::sphere(0.45f, 0.7f, 0.4f);
                world.add(makeBody(bc, {Real((i % 5) - 2) * 1.3f, 3.0f + (i / 5) * 1.3f,
                                        Real((i / 5 % 3) - 1) * 1.3f}, 1.0f), bc);
            }
        } else if (v == 3) {
            addFloor(0.3f);
            for (int i = 0; i < 6; ++i) {
                Collider bc = Collider::box({0.5f, 0.5f, 0.5f}, 0.5f, 0.4f);
                world.add(makeBody(bc, {Real(i - 3) * 1.6f + 0.2f, 4.0f + i * 0.3f, 0.1f * i}, 1.0f), bc);
            }
        } else {
            // The restitution row: same drop, four different bounciness values.
            addFloor(0.0f);
            const Real rest[4] = {0.0f, 0.3f, 0.6f, 0.9f};
            for (int i = 0; i < 4; ++i) {
                Collider bc = Collider::sphere(0.5f, rest[i], 0.4f);
                world.add(makeBody(bc, {Real(i - 1) * 2.2f - 1.1f, 5.0f, 0}, 1.0f), bc);
            }
        }
    }

    void fixedUpdate(Real dt) override { world.step(dt); }

    void drawBody(Renderer3D& r, int i, const Color& col) {
        const RigidBody& b = world.rbs[i];
        const Collider& c = world.colliders[i];
        const Mat4 m = translation(b.position) * toMat4(b.orientation);
        if (c.type == Collider::Type::Sphere)
            r.drawMesh(ball, m * scaling(Vec3(c.radius)), col);
        else
            r.drawMesh(cube, m * scaling(c.half), col);
    }

    void render(Renderer3D& r, Real) override {
        debug::grid(r, 8, 16);
        const int n = static_cast<int>(world.rbs.size());
        for (int i = 0; i < n; ++i) {
            if (world.rbs[i].isStatic()) { drawBody(r, i, Color::bytes(52, 58, 72)); continue; }
            // In the restitution row, tint by bounciness (dim=dead, bright=bouncy).
            Color col = palette::sky;
            if (sceneVariant() == 1) col = lerp3(palette::slate, palette::teal, world.colliders[i].restitution);
            drawBody(r, i, col);
        }
    }

    static Color lerp3(const Color& a, const Color& b, Real t) {
        return {lerp(a.r, b.r, t), lerp(a.g, b.g, t), lerp(a.b, b.b, t), 1};
    }

    std::vector<std::string> hudLines() const override {
        return {"1 restitution row (0 / 0.3 / 0.6 / 0.9)   2 bouncy bin   3 boxes",
                "restitution = bounciness: fraction of speed returned on impact"};
    }
};

int main(int argc, char** argv) { return ImpulsesDemo{}.run(argc, argv); }
