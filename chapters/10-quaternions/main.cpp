// ===========================================================================
//  Chapter 10 — Rotation & Quaternions
// ---------------------------------------------------------------------------
//  Goal: represent and update ORIENTATION well. The obvious approach — three
//  Euler angles (yaw, pitch, roll) — has a nasty failure called GIMBAL LOCK: at
//  certain orientations two of the three axes line up and you lose a degree of
//  freedom. Quaternions avoid it entirely, interpolate smoothly, and don't drift.
//
//  Three scenes:
//    1  Euler angles & gimbal lock — three coloured arrows are the yaw/pitch/roll
//       axes. As pitch sweeps to ±90°, the roll axis swings up to meet the yaw
//       axis: they coincide, and "yaw" and "roll" now do the same thing. Locked.
//    2  Quaternion spin — the same little ship, spun by a quaternion integrated
//       from a constant angular velocity. It tumbles through every orientation
//       forever, no lock, no drift.
//    3  Slerp — smoothly interpolate between two orientations along the shortest
//       arc (the "right" way to blend rotations).
//
//  Engine pieces: common/math/quat.hpp. Companion page: docs/chapters/10-quaternions.html
// ===========================================================================
#include "common/app.hpp"
#include "common/render/debug_draw.hpp"

using namespace p3d;

struct QuaternionsDemo : App {
    Real t = 0;

    // Ship parts (drawn together so the object's facing is unmistakable).
    Mesh fuselage = mesh::box({0.16f, 0.16f, 0.62f});
    Mesh wings     = mesh::box({0.72f, 0.05f, 0.16f});
    Mesh tail       = mesh::box({0.05f, 0.24f, 0.12f});
    Mesh nose       = mesh::box({0.10f, 0.10f, 0.12f});

    // Slerp endpoints (scene 3).
    Quat qA = Quat::fromAxisAngle({0, 1, 0}, radians(-70));
    Quat qB = Quat::fromAxisAngle(normalize(Vec3{1, 0.4f, 0.3f}), radians(120));

    QuaternionsDemo() {
        camera.target   = {0, 0.3f, 0};
        camera.distance = 7;
        camera.pitch    = radians(16);
    }

    std::string caption() const override { return "Ch10: Rotation & Quaternions"; }
    void onReset() override { t = 0; }
    void fixedUpdate(Real dt) override { t += dt; }

    // Draw the ship, oriented by `rot` (a rotation matrix), centred at the origin.
    void drawShip(Renderer3D& r, const Mat4& rot) {
        r.drawMesh(fuselage, rot,                                     palette::teal);
        r.drawMesh(wings,    rot * translation({0, 0, -0.05f}),       palette::amber);
        r.drawMesh(tail,     rot * translation({0, 0.2f, -0.5f}),     palette::coral);
        r.drawMesh(nose,     rot * translation({0, 0, 0.66f}),        palette::white);   // marks the nose
    }

    void render(Renderer3D& r, Real) override {
        debug::grid(r, 6, 12);
        const int v = sceneVariant();

        if (v == 2) {
            // --- Quaternion spin: integrate a constant angular velocity ------
            // Rebuild the orientation from scratch each frame from t so the demo
            // is stateless; a real body would integrate incrementally (Ch 11).
            const Vec3 axis = normalize(Vec3{0.3f, 1.0f, 0.5f});
            const Quat q = Quat::fromAxisAngle(axis, t * 1.6f);
            debug::arrow(r, -axis * 2.0f, axis * 2.0f, palette::violet);   // the spin axis
            drawShip(r, toMat4(q));
        } else if (v == 3) {
            // --- Slerp between two orientations ------------------------------
            const Real s = 0.5f + 0.5f * std::sin(t * 0.9f);
            const Quat q = slerp(qA, qB, s);
            drawShip(r, toMat4(q));
            // Faint ghosts of the two endpoints we're blending between.
            r.setAmbient(0.6f);
        } else {
            // --- Euler angles & gimbal lock ---------------------------------
            const Real yaw   = t * 0.6f;
            const Real pitch = radians(89) * std::sin(t * 0.5f);   // sweeps to ±90°
            const Real roll  = t * 1.1f;

            const Mat4 Ry = rotationY(yaw);
            const Mat4 Rp = Ry * rotationX(pitch);
            const Mat4 R  = Rp * rotationZ(roll);

            // The three Euler rotation axes, in world space. Watch the roll axis
            // (red) swing up toward the yaw axis (green) as pitch nears 90°.
            const Vec3 yawAxis   = axis::Y;
            const Vec3 pitchAxis = transformDir(Ry, axis::X);
            const Vec3 rollAxis  = transformDir(Rp, axis::Z);
            debug::arrow(r, Vec3{}, yawAxis   * 2.2f, palette::lime);
            debug::arrow(r, Vec3{}, pitchAxis * 2.0f, palette::sky);
            debug::arrow(r, Vec3{}, rollAxis  * 2.0f, palette::coral);

            drawShip(r, R);
        }
    }

    std::vector<std::string> hudLines() const override {
        std::vector<std::string> out;
        const int v = sceneVariant();
        if (v == 2)      out.emplace_back("quaternion spin: smooth, no lock, no drift");
        else if (v == 3) out.emplace_back("slerp: shortest-arc blend between two orientations");
        else {
            const Real pitch = radians(89) * std::sin(t * 0.5f);
            const bool locked = std::fabs(std::fabs(degrees(pitch)) - 90) < 8;
            out.emplace_back(locked ? "** GIMBAL LOCK: yaw & roll axes aligned! **"
                                    : "green=yaw  blue=pitch  red=roll axis");
        }
        out.emplace_back("1 Euler/gimbal lock   2 quaternion spin   3 slerp");
        return out;
    }
};

int main(int argc, char** argv) { return QuaternionsDemo{}.run(argc, argv); }
