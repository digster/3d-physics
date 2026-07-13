// ===========================================================================
//  Chapter 18 — Resting Contact & Sleeping
// ---------------------------------------------------------------------------
//  Goal: the finishing touches that make a big pile behave — and stay cheap.
//    * RESTING CONTACT — a body sitting still on the ground is really being held
//      up by a continuous stream of tiny impulses fighting gravity. Warm starting
//      and a little position slop keep that quiet instead of buzzing.
//    * SLEEPING — a body that has barely moved for a while is put to SLEEP: we
//      stop integrating and solving it until something disturbs it. A settled pile
//      of hundreds of boxes then costs almost nothing, and stops its last twitch.
//      Bodies that touch keep each other awake, so a pile sleeps and wakes as a
//      group (an "island").
//
//  Sleeping bodies are drawn dim blue; awake ones bright amber. Watch the HUD's
//  awake-count fall to zero as the pile settles.
//    Scenes: 1 a pile settling; 2 a settled pile that a falling body re-wakes;
//    3 sleeping OFF, so you can see the endless micro-jitter it removes.
//  Key S also toggles sleeping. Companion page: docs/chapters/18-resting.html
// ===========================================================================
#include "common/app.hpp"
#include "common/render/debug_draw.hpp"
#include "physics/world.hpp"

#include <random>
#include <vector>

using namespace p3d;

struct RestingDemo : App {
    World world;
    Mesh cube = mesh::box({1, 1, 1});
    Mesh ball = mesh::sphere(1.0f, 10, 14);
    std::mt19937 rng{5};
    Real dropTimer = 0;

    static constexpr int kStatic = 5;   // floor + 4 walls
    int spawned = 0;

    RestingDemo() {
        camera.target   = {0, 1.0f, 0};
        camera.distance = 13;
        camera.pitch    = radians(34);   // look down INTO the bin
    }

    std::string caption() const override { return "Ch18: Resting Contact & Sleeping"; }

    void addBin() {
        Collider fl = Collider::box({3.0f, 0.5f, 3.0f}, 0.1f, 0.7f);
        world.add(makeBody(fl, {0, -0.5f, 0}, 0), fl);
        // Short walls so the pile stays contained but never hides behind them.
        for (int s = -1; s <= 1; s += 2) {
            Collider wx = Collider::box({0.3f, 1.6f, 3.0f}, 0.2f, 0.6f);
            world.add(makeBody(wx, {s * 3.0f, 1.1f, 0}, 0), wx);
            Collider wz = Collider::box({3.0f, 1.6f, 0.3f}, 0.2f, 0.6f);
            world.add(makeBody(wz, {0, 1.1f, s * 3.0f}, 0), wz);
        }
    }

    // Drop one body from up high, at a random spot. Spawning ONE AT A TIME (over
    // several frames) avoids the deep overlap that a burst-spawn would cause.
    void dropBody() {
        std::uniform_real_distribution<Real> px(-2.2f, 2.2f);
        std::uniform_real_distribution<Real> pick(0, 1);
        const Vec3 pos{px(rng), 5.0f, px(rng)};
        if (pick(rng) < 0.5f) {
            Collider c = Collider::box({0.42f, 0.42f, 0.42f}, 0.1f, 0.6f);
            world.add(makeBody(c, pos, 1.0f), c);
        } else {
            Collider c = Collider::sphere(0.42f, 0.2f, 0.5f);
            world.add(makeBody(c, pos, 1.0f), c);
        }
    }

    void onReset() override {
        world = World{};
        world.solver.iterations = 10;
        world.sleepingEnabled = (sceneVariant() != 3);   // scene 3 turns sleeping off
        dropTimer = 0; spawned = 0;
        addBin();
    }

    void fixedUpdate(Real dt) override {
        world.step(dt);
        dropTimer += dt;
        const int cap = (sceneVariant() == 2) ? 14 : 40;
        // Fill the bin one body at a time...
        if (spawned < cap && dropTimer > 0.12f) { dropTimer = 0; dropBody(); ++spawned; }
        // ...then, in scene 2, keep lobbing one in every couple of seconds to
        // re-wake the settled (sleeping) island.
        else if (sceneVariant() == 2 && spawned >= cap && dropTimer > 2.2f) {
            dropTimer = 0; dropBody(); ++spawned;
        }
    }

    void render(Renderer3D& r, Real) override {
        debug::grid(r, 7, 14);
        const int n = static_cast<int>(world.rbs.size());
        for (int i = 0; i < n; ++i) {
            const RigidBody& b = world.rbs[i];
            const Collider& c = world.colliders[i];
            Color col;
            if (b.isStatic())      col = Color::bytes(48, 54, 68);
            else if (world.awake[i]) col = palette::amber;     // awake = bright
            else                     col = Color::bytes(70, 96, 150);   // asleep = dim blue
            const Mat4 m = translation(b.position) * toMat4(b.orientation);
            if (c.type == Collider::Type::Sphere) r.drawMesh(ball, m * scaling(Vec3(c.radius)), col);
            else                                  r.drawMesh(cube, m * scaling(c.half), col);
        }
    }

    std::vector<std::string> hudLines() const override {
        char buf[96];
        std::snprintf(buf, sizeof(buf), "awake: %d / %d dynamic bodies    sleeping=%s (S)",
                      world.awakeCount(), int(world.rbs.size()) - 5,
                      world.sleepingEnabled ? "ON" : "off");
        return {buf, "amber = awake, dim blue = asleep",
                "1 pile settles & sleeps   2 drop wakes the island   3 sleeping OFF"};
    }

    void onKey(SDL_Keycode key) override {
        if (key == SDLK_S) world.sleepingEnabled = !world.sleepingEnabled;
    }
};

int main(int argc, char** argv) { return RestingDemo{}.run(argc, argv); }
