// ===========================================================================
//  Chapter 20 — XPBD & Soft Bodies
// ---------------------------------------------------------------------------
//  The finale. Everything so far has been perfectly rigid; now we make things
//  SQUISH. A soft body is — once again — a lattice of particles held by distance
//  constraints, exactly like the cloth of Chapter 9. The new ingredient is XPBD
//  (Extended Position-Based Dynamics): each constraint gets a COMPLIANCE (inverse
//  stiffness), so instead of the rigid "snap to length" of Chapter 8, it behaves
//  like a real spring with a physically meaningful, timestep-independent
//  stiffness. Turn compliance up and the cube turns to jelly; turn it down and it
//  firms up toward rigid. It's the whole course in one object — particles,
//  constraints, integration, and collision, wobbling.
//
//  Scenes:
//    1  A jello cube dropped on the floor: it squishes flat on impact and springs
//       back, jiggling.
//    2  Four cubes, stiff → soft (left to right), dropped together — the same
//       impact, four very different amounts of squish.
//    3  A jello cube hung from its top face, sagging and wobbling under its own
//       weight.
//  Press W for the wireframe lattice underneath the surface.
//
//  Engine: physics/softbody.hpp. Companion page: docs/chapters/20-xpbd.html
// ===========================================================================
#include "common/app.hpp"
#include "common/render/debug_draw.hpp"
#include "physics/softbody.hpp"

#include <vector>

using namespace p3d;

struct XpbdDemo : App {
    std::vector<SoftBody> bodies;
    std::vector<Color>    colors;
    Real groundY = 0;

    XpbdDemo() {
        camera.target   = {0, 1.5f, 0};
        camera.distance = 12;
        camera.pitch    = radians(12);
    }

    std::string caption() const override { return "Ch20: XPBD & Soft Bodies"; }

    // A jello green that shifts toward coral as it gets softer, so the
    // stiff/soft comparison reads at a glance.
    static Color jelloColor(Real compliance) {
        const Real t = clamp(compliance / 0.08f, 0, 1);
        return {lerp(0.32f, 0.94f, t), lerp(0.82f, 0.42f, t), lerp(0.45f, 0.4f, t), 1};
    }

    void onReset() override {
        bodies.clear(); colors.clear();
        const int v = sceneVariant();

        if (v == 2) {
            // Stiff → soft, left to right.
            const Real comps[4] = {0.001f, 0.008f, 0.025f, 0.07f};
            for (int i = 0; i < 4; ++i) {
                SoftBody sb;
                sb.buildCube({Real(i - 1.5f) * 2.6f, 3.0f, 0}, 1.4f, 4, comps[i], 3.0f);
                bodies.push_back(std::move(sb));
                colors.push_back(jelloColor(comps[i]));
            }
            groundY = 0;
        } else if (v == 3) {
            // A cube hung by its whole top face — it sags and wobbles.
            SoftBody sb;
            sb.buildCube({0, 3.2f, 0}, 1.6f, 4, 0.03f, 3.0f);
            for (int i = 0; i < sb.nx; ++i)
                for (int k = 0; k < sb.nz; ++k)
                    sb.pin(sb.index(i, sb.ny - 1, k));   // pin the top layer (max j)
            bodies.push_back(std::move(sb));
            colors.push_back(jelloColor(0.03f));
            groundY = -100;                              // no floor to hit
        } else {
            SoftBody sb;
            sb.buildCube({0, 3.0f, 0}, 1.6f, 5, 0.02f, 3.0f);
            bodies.push_back(std::move(sb));
            colors.push_back(jelloColor(0.02f));
            groundY = 0;
        }
    }

    void fixedUpdate(Real dt) override {
        const Vec3 g{0, -9.81f, 0};
        for (SoftBody& sb : bodies) sb.step(dt, 12, g, groundY);
    }

    void render(Renderer3D& r, Real) override {
        debug::grid(r, 8, 16);
        for (size_t i = 0; i < bodies.size(); ++i)
            r.drawMesh(bodies[i].surfaceMesh, Mat4::identity(), colors[i], /*doubleSided=*/true);
    }

    std::vector<std::string> hudLines() const override {
        return {"1 jello drop   2 stiff -> soft row   3 hanging jello   (W = lattice)",
                "compliance = inverse stiffness: 0 is rigid, higher is squishier"};
    }
};

int main(int argc, char** argv) { return XpbdDemo{}.run(argc, argv); }
