// ===========================================================================
//  Chapter 4 — Camera & Projection
// ---------------------------------------------------------------------------
//  Goal: understand how a 3D scene becomes a 2D image. This demo assembles a
//  little scene of flat-shaded primitives and lets you fly around it. Every
//  pixel you see went through the pipeline in common/render/renderer3d.cpp:
//
//     model → world → view (the camera) → clip (the lens) → screen
//
//  Two things to play with, because they ARE the projection:
//    * Orbit and zoom (drag / wheel): you are changing the VIEW matrix.
//    * Field of view ( [ and ] keys ): you are changing the PROJECTION matrix.
//      Narrow FOV looks telephoto and flat; wide FOV looks fish-eye and
//      exaggerates depth. Same scene, different lens.
//
//  Notice how nearer objects correctly cover farther ones — that is the
//  painter's algorithm (back-to-front sort) inside the renderer at work.
//
//  Companion page: docs/chapters/04-camera.html
// ===========================================================================
#include "common/app.hpp"
#include "common/render/debug_draw.hpp"

using namespace p3d;

struct CameraDemo : App {
    // A body in the scene: which mesh, where, and what color.
    struct Item { const Mesh* mesh; Vec3 pos; Color color; Real spin; };

    Mesh ground   = mesh::plane(20.0f, 10);
    Mesh boxMesh  = mesh::box(Vec3{0.8f, 0.8f, 0.8f});
    Mesh ballMesh = mesh::sphere(0.9f, 16, 22);
    Mesh pillar   = mesh::cylinder(0.6f, 2.2f, 22);

    std::vector<Item> items;
    Real t = 0;

    CameraDemo() {
        camera.target   = {0, 1.0f, 0};
        camera.distance = 12;
        camera.pitch    = radians(18);
    }

    std::string caption() const override { return "Ch4: Camera & Projection"; }

    void onReset() override {
        t = 0;
        items.clear();
        const int v = sceneVariant();
        if (v <= 1) {
            // A "primitive zoo": one of each shape in a row.
            items.push_back({&boxMesh,  {-4, 0.8f, 0}, palette::teal,   0.6f});
            items.push_back({&ballMesh, {-1.3f, 0.9f, 0}, palette::amber, 0});
            items.push_back({&pillar,   { 1.6f, 1.1f, 0}, palette::violet, 0});
            items.push_back({&boxMesh,  { 4, 0.8f, 0}, palette::coral,  -0.9f});
        } else if (v == 2) {
            // A grid of cubes — many triangles, lots of depth overlap, a good
            // stress test for the back-to-front sort.
            for (int i = -3; i <= 3; ++i)
                for (int j = -3; j <= 3; ++j)
                    items.push_back({&boxMesh, {Real(i) * 2, 0.8f, Real(j) * 2},
                                     ((i + j) & 1) ? palette::teal : palette::sky, 0});
        } else {
            // A cluster of balls at different depths.
            for (int i = 0; i < 9; ++i) {
                const Real a = Real(i) / 9 * kTwoPi;
                items.push_back({&ballMesh,
                                 {std::cos(a) * 3.5f, 0.9f, std::sin(a) * 3.5f},
                                 i & 1 ? palette::lime : palette::amber, 0});
            }
        }
    }

    void fixedUpdate(Real dt) override { t += dt; }

    // Field of view on the [ and ] keys — a direct handle on the projection.
    void onKey(SDL_Keycode key) override {
        if (key == SDLK_LEFTBRACKET)  camera.fovY = clamp(camera.fovY - radians(3), radians(15), radians(110));
        if (key == SDLK_RIGHTBRACKET) camera.fovY = clamp(camera.fovY + radians(3), radians(15), radians(110));
    }

    void render(Renderer3D& r, Real) override {
        debug::grid(r, 10, 20);
        r.drawMesh(ground, translation(Vec3{0, 0, 0}), palette::ground);

        for (const Item& it : items) {
            Mat4 model = translation(it.pos);
            if (it.spin != 0) model = model * rotationY(t * it.spin);
            r.drawMesh(*it.mesh, model, it.color);
        }
    }

    std::vector<std::string> hudLines() const override {
        char buf[96];
        std::snprintf(buf, sizeof(buf), "FOV=%.0f deg ([ / ] to change)   bodies=%d",
                      double(degrees(camera.fovY)), int(items.size()));
        return {buf, "1 zoo   2 cube grid   3 ball ring"};
    }
};

int main(int argc, char** argv) { return CameraDemo{}.run(argc, argv); }
