// ===========================================================================
//  renderer3d.hpp — a tiny software 3D renderer on top of SDL's 2D triangles
// ---------------------------------------------------------------------------
//  Introduced in: Chapter 4 (Camera & Projection)
//
//  This is the piece that lets us SEE the physics. It is deliberately NOT a GPU
//  shader pipeline: we do the whole vertex journey — model → world → view →
//  clip → screen — on the CPU, and hand SDL flat 2D triangles to fill. That
//  journey IS the subject of chapters 3–4, so it belongs in code you can read.
//
//  How to use it, once per frame:
//      r.begin(view, proj, width, height);   // set up the frame
//      r.drawMesh(boxMesh, modelMatrix, color);
//      r.drawMesh(ballMesh, ...);
//      r.drawLine(a, b, color);               // debug overlays
//      r.end();                               // sort everything, emit to SDL
//
//  The key idea inside: `drawMesh` does not draw immediately. It transforms and
//  shades triangles and stashes them in a buffer. `end()` sorts that buffer
//  back-to-front (the painter's algorithm) so nearer objects correctly cover
//  farther ones, then submits it. See renderer3d.cpp for the pipeline stages.
// ===========================================================================
#pragma once

#include "../math/math.hpp"
#include "color.hpp"
#include "mesh.hpp"
#include <vector>

struct SDL_Renderer;  // forward-declared: only the .cpp needs the SDL headers

namespace p3d {

class Renderer3D {
public:
    explicit Renderer3D(SDL_Renderer* renderer);

    // Start a frame: supply the camera's view & projection matrices and the
    // pixel size of the viewport. Clears the internal triangle/line buffers.
    void begin(const Mat4& view, const Mat4& proj, int viewportW, int viewportH);

    // Queue a mesh for drawing, positioned/oriented/scaled by `model` and
    // tinted `color`. Triangles are transformed, backface-culled, near-clipped
    // and flat-shaded now; actual submission happens in end().
    void drawMesh(const Mesh& mesh, const Mat4& model, const Color& color);

    // Queue a world-space line segment (for debug overlays: axes, AABBs,
    // contact normals). Lines are drawn on top of the shaded geometry.
    void drawLine(const Vec3& a, const Vec3& b, const Color& color);

    // Finish the frame: depth-sort and emit everything queued since begin().
    void end();

    // --- Look & feel knobs -------------------------------------------------
    void setLightDir(const Vec3& worldDir) { lightDir_ = normalize(worldDir); }
    void setAmbient(Real a)                { ambient_  = a; }
    void setWireframe(bool on)             { wireframe_ = on; }
    bool wireframe() const                 { return wireframe_; }

    // Project a world-space point to pixel coordinates. Returns false if the
    // point is behind the near plane (nothing sensible to draw). Handy for
    // placing 2D text labels next to 3D things.
    bool projectToScreen(const Vec3& worldPoint, float& outX, float& outY) const;

private:
    // A triangle reduced to screen coordinates plus a depth key for sorting.
    struct ScreenTri {
        float x[3], y[3];  // pixel positions of the three corners
        Color color;       // already flat-shaded
        float depth;       // average view-space z (more negative = farther)
    };
    struct ScreenLine {
        float x0, y0, x1, y1;
        Color color;
    };

    SDL_Renderer* renderer_;
    Mat4 view_{Mat4::identity()};
    Mat4 proj_{Mat4::identity()};
    int  vpW_{1}, vpH_{1};

    // A soft key light coming from the upper-left-front, plus ambient fill so
    // faces turned away from the light are dim but not pure black.
    Vec3 lightDir_{normalize(Vec3{-0.4f, -1.0f, -0.35f})};
    Real ambient_{0.28f};
    bool wireframe_{false};

    std::vector<ScreenTri>  tris_;
    std::vector<ScreenLine> lines_;

    // Map a view-space point to pixel coordinates via projection + the
    // perspective divide + the viewport transform.
    void viewToScreen(const Vec3& viewPos, float& outX, float& outY) const;
};

}  // namespace p3d
