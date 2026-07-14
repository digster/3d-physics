// ===========================================================================
//  Chapter 19 — Joints
// ---------------------------------------------------------------------------
//  Goal: connect bodies. A contact is a one-sided constraint (it can only push);
//  a JOINT is a two-sided one (it holds a relationship exactly, pushing OR
//  pulling). Remarkably, joints drop straight into the Part V solver loop — they
//  use the same impulses, effective masses, and warm starting as contacts, just
//  without the "≥ 0" clamp. Three joints cover an enormous amount of ground:
//    * distance    — a rigid rod between two points.
//    * ball-socket — pins two points together (3 DOF): shoulders, chain links.
//    * hinge       — a ball-socket that also shares a spin axis (1 DOF): doors.
//
//  Scenes:
//    1  A chain of links hanging from a fixed anchor, released horizontal, swings
//       and settles — a rope of ball-socket joints.
//    2  A trapdoor on a hinge with a box resting on it; released, it swings open
//       and the box slides off.
//    3  A ragdoll — boxes for a torso, head, arms, and legs joined by ball
//       sockets — dropped in a heap.
//
//  Uses World + the joints in physics/joint.hpp. Note: jointed bodies are exempt
//  from colliding with each other, so a chain's links don't fight their joints.
//  Companion page: docs/chapters/19-joints.html
// ===========================================================================
#include "common/app.hpp"
#include "common/render/debug_draw.hpp"
#include "physics/world.hpp"

#include <memory>
#include <vector>

using namespace p3d;

struct JointsDemo : App {
    World world;
    Mesh cube = mesh::box({1, 1, 1});
    Mesh ball = mesh::sphere(1.0f, 10, 14);

    JointsDemo() {
        camera.target   = {0, 2.0f, 0};
        camera.distance = 13;
        camera.pitch    = radians(10);
    }

    std::string caption() const override { return "Ch19: Joints"; }

    int addBox(const Vec3& pos, const Vec3& half, Real mass, const Quat& rot = Quat::identity(), Real fric = 0.5f) {
        Collider c = Collider::box(half, 0.1f, fric);
        int idx = world.add(makeBody(c, pos, mass), c);
        world.rbs[idx].orientation = rot;
        return idx;
    }
    void addFloor() {
        Collider c = Collider::box({12, 0.5f, 12}, 0.1f, 0.7f);
        world.add(makeBody(c, {0, -0.5f, 0}, 0), c);
    }

    void onReset() override {
        world = World{};
        world.solver.iterations = 12;
        const int v = sceneVariant();

        if (v == 2) {
            // --- A hinged trapdoor, with a box that rides it open ---
            addFloor();
            int post = addBox({-2.2f, 3.0f, 0}, {0.15f, 0.15f, 1.2f}, 0);        // static hinge post
            // Door sticks out in +x from the post; hinge axis is z (horizontal).
            int door = addBox({-1.0f, 3.0f, 0}, {1.1f, 0.1f, 1.0f}, 3.0f);
            world.joints.push_back(std::make_unique<HingeJoint>(
                post, door, Vec3{0, 0, 0}, Vec3{-1.1f, 0, 0}, Vec3{0, 0, 1}, Vec3{0, 0, 1}));
            addBox({-1.2f, 3.7f, 0}, {0.4f, 0.4f, 0.4f}, 1.0f);                  // a box sitting on it
            // A second, plain swinging door hinged the same way, for variety.
            int post2 = addBox({1.6f, 3.0f, 0}, {0.15f, 0.15f, 1.2f}, 0);
            int door2 = addBox({2.8f, 3.0f, 0}, {1.1f, 0.1f, 1.0f}, 3.0f);
            world.joints.push_back(std::make_unique<HingeJoint>(
                post2, door2, Vec3{0, 0, 0}, Vec3{-1.1f, 0, 0}, Vec3{0, 0, 1}, Vec3{0, 0, 1}));
        } else if (v == 3) {
            // --- A ragdoll of boxes joined by ball sockets ---
            addFloor();
            const Real y = 3.2f;
            int torso = addBox({0, y, 0}, {0.32f, 0.5f, 0.16f}, 3.0f);
            int head  = addBox({0, y + 0.95f, 0}, {0.22f, 0.22f, 0.22f}, 1.0f);
            world.joints.push_back(std::make_unique<BallSocketJoint>(torso, head, Vec3{0, 0.5f, 0}, Vec3{0, -0.35f, 0}));
            // Arms (upper only, kept simple).
            for (int s = -1; s <= 1; s += 2) {
                int arm = addBox({s * 0.95f, y + 0.35f, 0}, {0.32f, 0.12f, 0.12f}, 0.8f);
                world.joints.push_back(std::make_unique<BallSocketJoint>(
                    torso, arm, Vec3{s * 0.32f, 0.4f, 0}, Vec3{-s * 0.32f, 0, 0}));
            }
            // Legs.
            for (int s = -1; s <= 1; s += 2) {
                int leg = addBox({s * 0.18f, y - 0.95f, 0}, {0.13f, 0.45f, 0.13f}, 1.2f);
                world.joints.push_back(std::make_unique<BallSocketJoint>(
                    torso, leg, Vec3{s * 0.18f, -0.5f, 0}, Vec3{0, 0.45f, 0}));
            }
        } else {
            // --- A chain hanging from a fixed anchor, released horizontal ---
            const int n = 11;
            const Real h = 0.34f;   // link half-length
            int anchor = addBox({-3.6f, 5.0f, 0}, {0.1f, 0.1f, 0.1f}, 0);   // static
            int prev = anchor;
            Vec3 prevRight{0, 0, 0};   // local anchor on prev (its right end)
            for (int i = 0; i < n; ++i) {
                const Real cx = -3.6f + h + 2 * h * i;
                int link = addBox({cx, 5.0f, 0}, {h, 0.09f, 0.09f}, 1.0f);
                world.joints.push_back(std::make_unique<BallSocketJoint>(
                    prev, link, prevRight, Vec3{-h, 0, 0}));
                prev = link;
                prevRight = Vec3{h, 0, 0};
            }
        }
    }

    void fixedUpdate(Real dt) override { world.step(dt); }

    void render(Renderer3D& r, Real) override {
        debug::grid(r, 8, 16);
        const int n = static_cast<int>(world.rbs.size());
        for (int i = 0; i < n; ++i) {
            const RigidBody& b = world.rbs[i];
            const Collider& c = world.colliders[i];
            const Color col = b.isStatic() ? Color::bytes(52, 58, 72)
                            : (sceneVariant() == 3 ? palette::amber : palette::teal);
            const Mat4 m = translation(b.position) * toMat4(b.orientation);
            if (c.type == Collider::Type::Sphere) r.drawMesh(ball, m * scaling(Vec3(c.radius)), col);
            else                                  r.drawMesh(cube, m * scaling(c.half), col);
        }
    }

    std::vector<std::string> hudLines() const override {
        return {"1 hanging chain   2 hinged trapdoor + box   3 ragdoll",
                "joints = two-sided constraints (they push AND pull)"};
    }
};

int main(int argc, char** argv) { return JointsDemo{}.run(argc, argv); }
