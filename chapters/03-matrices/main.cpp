// ===========================================================================
//  Chapter 3 — Matrices & Transforms
// ---------------------------------------------------------------------------
//  Goal: see that a 4x4 matrix is a verb — "translate", "rotate", "scale" — and
//  that MULTIPLYING matrices CHAINS those verbs. A cube spins in place while a
//  smaller cube orbits it; the orbit is built purely by composing transforms,
//  which is exactly how scene hierarchies (a moon around a planet, a hand on an
//  arm) work.
//
//  Watch the model matrix being assembled in render():
//      model = R_spin * S_pulse            (rotate, and scale, the big cube)
//      child = R_orbit * T_out * R_selfspin * S_small
//  Remember the reading order is RIGHT-TO-LEFT: the rightmost transform happens
//  first, in the object's own local space. Press W to see the wireframe, and
//  keys 1–4 to switch which transforms are active.
//
//  Companion page: docs/chapters/03-matrices.html
// ===========================================================================
#include "common/app.hpp"
#include "common/render/debug_draw.hpp"

using namespace p3d;

struct MatricesDemo : App {
    Real t = 0;
    Mesh cube = mesh::box(Vec3{1, 1, 1});
    Mesh small = mesh::box(Vec3{0.4f, 0.4f, 0.4f});

    MatricesDemo() {
        camera.target   = {0, 0, 0};
        camera.distance = 9;
    }

    std::string caption() const override { return "Ch3: Matrices & Transforms"; }
    void onReset() override { t = 0; }
    void fixedUpdate(Real dt) override { t += dt; }

    void render(Renderer3D& r, Real) override {
        debug::grid(r);
        debug::axes(r, Vec3{}, 1.5f);

        const int v = sceneVariant();

        // --- The central cube --------------------------------------------
        // Spin: two rotations composed. Because they are applied together, the
        // cube tumbles about a slanted axis — the product of two simple verbs.
        Mat4 spin = rotationY(t * 0.8f) * rotationX(t * 0.5f);

        // Variant 2+ : also pulse the scale, to show scaling composed with
        // rotation (rotate a stretched box, not stretch a rotated one — order!).
        Mat4 model = spin;
        if (v >= 2) {
            const Real s = 1.0f + 0.25f * std::sin(t * 2.0f);
            model = spin * scaling(Vec3{s, s, s});
        }
        r.drawMesh(cube, model, palette::teal);

        // --- The orbiting child (variants 3–4) ----------------------------
        // Read right-to-left: shrink the small cube, spin it about its own
        // centre, push it out along +x, THEN rotate that whole arrangement
        // around the origin. That final rotation is what makes it orbit.
        if (v >= 3) {
            Mat4 orbit =
                rotationY(t * 1.1f) *              // revolve around the big cube
                translation(Vec3{3.0f, 0, 0}) *    // arm length
                rotationY(t * 2.5f) *              // child's own spin
                rotationX(t * 1.3f);
            r.drawMesh(small, orbit, palette::amber);

            // Draw the arm so the composition is visible: origin → child centre.
            const Vec3 childPos = transformPoint(orbit, Vec3{0, 0, 0});
            r.drawLine(Vec3{0, 0, 0}, childPos, Color::bytes(120, 110, 70));
        }
    }

    std::vector<std::string> hudLines() const override {
        return {"1 spin   2 +scale pulse   3 +orbiting child   4 all",
                "reading order is right-to-left: local space first"};
    }
};

int main(int argc, char** argv) { return MatricesDemo{}.run(argc, argv); }
