// ===========================================================================
//  Chapter 9 — Cloth
// ---------------------------------------------------------------------------
//  Goal: the payoff of Part II. Cloth is just a GRID of Verlet particles laced
//  together with the distance constraints from Chapter 8 — but with three kinds
//  of link, each holding a different failure mode in check:
//     * STRUCTURAL links join each particle to its up/down/left/right neighbour
//       — they give the sheet its size.
//     * SHEAR links cross the diagonals — without them the grid folds flat like
//       a collapsing lattice.
//     * BEND links skip a particle (reach to the neighbour-two-away) — they
//       resist sharp creasing, so the cloth folds in smooth waves.
//  Add gravity and a bit of wind and you get a flag. That's it — the same tiny
//  solve() from the last chapter, run over a mesh instead of a line.
//
//  We rebuild a triangle mesh from the particle grid every frame and draw it
//  double-sided (a flag has no "inside", and we see both faces as it ripples).
//
//  Engine pieces: physics/particle.hpp (Verlet), physics/constraint.hpp.
//  Companion page: docs/chapters/09-cloth.html
// ===========================================================================
#include "common/app.hpp"
#include "common/render/debug_draw.hpp"
#include "physics/particle.hpp"
#include "physics/constraint.hpp"

#include <vector>

using namespace p3d;

struct ClothDemo : App {
    static constexpr int COLS = 22;
    static constexpr int ROWS = 15;
    static constexpr Real SP = 0.30f;     // spacing between neighbours

    std::vector<Particle>           particles;
    std::vector<DistanceConstraint> constraints;
    Mesh clothMesh;                       // rebuilt from particle positions each frame
    int  iterations = 8;
    Real windPhase  = 0;

    int idx(int r, int c) const { return r * COLS + c; }

    ClothDemo() {
        camera.target   = {0, 2.3f, 0};
        camera.distance = 12;
        camera.yaw      = radians(35);
    }

    std::string caption() const override { return "Ch9: Cloth (a flag in the wind)"; }

    void addLink(int a, int b, Real stiffness) {
        constraints.push_back({a, b, distance(particles[a].position, particles[b].position), stiffness});
    }

    void onReset() override {
        particles.clear(); constraints.clear();
        windPhase = 0;
        const int v = sceneVariant();
        const Vec3 origin{-Real(COLS - 1) * SP * 0.5f, 5.0f, 0};

        // Lay the grid out flat in the XY plane (facing +z).
        for (int r = 0; r < ROWS; ++r)
            for (int c = 0; c < COLS; ++c) {
                Particle p;
                p.position = origin + Vec3{Real(c) * SP, -Real(r) * SP, 0};
                p.setMass(1.0f);
                // Pinning depends on the scene:
                //  1 flag  → pin the left column (a pole on the left)
                //  2 curtain → pin the whole top row
                //  3 corners → pin only the two top corners
                bool pin = false;
                if (v == 2)      pin = (r == 0);
                else if (v == 3) pin = (r == 0 && (c == 0 || c == COLS - 1));
                else             pin = (c == 0);
                if (pin) p.makeStatic();
                p.syncVerlet();
                particles.push_back(p);
            }

        // Weave the three kinds of link.
        for (int r = 0; r < ROWS; ++r)
            for (int c = 0; c < COLS; ++c) {
                if (c + 1 < COLS) addLink(idx(r, c), idx(r, c + 1), 1.0f);       // structural →
                if (r + 1 < ROWS) addLink(idx(r, c), idx(r + 1, c), 1.0f);       // structural ↓
                if (c + 1 < COLS && r + 1 < ROWS) {                              // shear ⤡ ⤢
                    addLink(idx(r, c),     idx(r + 1, c + 1), 0.9f);
                    addLink(idx(r, c + 1), idx(r + 1, c),     0.9f);
                }
                if (c + 2 < COLS) addLink(idx(r, c), idx(r, c + 2), 0.5f);       // bend →→
                if (r + 2 < ROWS) addLink(idx(r, c), idx(r + 2, c), 0.5f);       // bend ↓↓
            }

        // Build the mesh index buffer once (two triangles per grid cell). The
        // positions get refreshed every frame in render().
        clothMesh.positions.assign(particles.size(), Vec3{});
        clothMesh.indices.clear();
        for (int r = 0; r < ROWS - 1; ++r)
            for (int c = 0; c < COLS - 1; ++c) {
                const int a = idx(r, c), b = idx(r, c + 1);
                const int cc = idx(r + 1, c), d = idx(r + 1, c + 1);
                clothMesh.indices.insert(clothMesh.indices.end(), {a, cc, d,  a, d, b});
            }
    }

    void fixedUpdate(Real dt) override {
        const int v = sceneVariant();
        const Vec3 g{0, -9.81f, 0};
        windPhase += dt;

        // Wind blows along +z, gusting over time. Scene 3 blows much harder.
        const Real base = (v == 3) ? 7.0f : 3.0f;
        for (int r = 0; r < ROWS; ++r)
            for (int c = 0; c < COLS; ++c) {
                Particle& p = particles[idx(r, c)];
                // A little spatial + temporal variation makes the ripple lively
                // rather than a uniform push.
                const Real gust = base * (0.6f + 0.4f * std::sin(windPhase * 2.0f + Real(c) * 0.4f + Real(r) * 0.2f));
                p.integrateVerlet(g + Vec3{0, 0, gust}, dt);
            }

        satisfyConstraints(constraints, particles, iterations);
    }

    void render(Renderer3D& r, Real) override {
        debug::grid(r, 8, 16);

        // Refresh the mesh from the current particle positions and draw it as a
        // solid, double-sided sheet.
        for (size_t i = 0; i < particles.size(); ++i)
            clothMesh.positions[i] = particles[i].position;
        r.drawMesh(clothMesh, Mat4::identity(), Color::bytes(70, 150, 210), /*doubleSided=*/true);

        // Mark the pinned particles so the attachment is clear.
        for (const Particle& p : particles)
            if (p.isStatic()) debug::point(r, p.position, 0.07f, palette::coral);
    }

    std::vector<std::string> hudLines() const override {
        return {"1 flag (pole on left)   2 curtain (top edge)   3 corners + gusts",
                "structural + shear + bend links = a stable sheet",
                "W wireframe shows the weave"};
    }
};

int main(int argc, char** argv) { return ClothDemo{}.run(argc, argv); }
