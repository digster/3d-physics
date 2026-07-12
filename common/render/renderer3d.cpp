// ===========================================================================
//  renderer3d.cpp — the software 3D pipeline, stage by stage
// ---------------------------------------------------------------------------
//  Read this alongside docs/chapters/04-camera.html. The comments below mark
//  each classic stage of a rasteriser so you can map code to theory.
// ===========================================================================
#include "renderer3d.hpp"

#include <SDL3/SDL.h>
#include <algorithm>   // std::sort
#include <array>

namespace p3d {

Renderer3D::Renderer3D(SDL_Renderer* renderer) : renderer_(renderer) {}

void Renderer3D::begin(const Mat4& view, const Mat4& proj, int viewportW, int viewportH) {
    view_ = view;
    proj_ = proj;
    vpW_  = viewportW  > 0 ? viewportW  : 1;
    vpH_  = viewportH > 0 ? viewportH : 1;
    tris_.clear();
    lines_.clear();
}

// --- Viewport transform ----------------------------------------------------
// Given a point already in VIEW space, run it through the projection matrix,
// do the perspective divide (this is where depth makes far things small), and
// map the resulting normalised device coordinates (NDC, a -1..+1 cube) to
// pixels. Note the Y flip: NDC y points up, screen y points down.
void Renderer3D::viewToScreen(const Vec3& viewPos, float& outX, float& outY) const {
    const Vec4 clip = proj_ * Vec4(viewPos, Real(1));
    const Vec3 ndc  = perspectiveDivide(clip);
    outX = (ndc.x * Real(0.5) + Real(0.5)) * Real(vpW_);
    outY = (Real(1) - (ndc.y * Real(0.5) + Real(0.5))) * Real(vpH_);
}

bool Renderer3D::projectToScreen(const Vec3& worldPoint, float& outX, float& outY) const {
    const Vec3 v = transformPoint(view_, worldPoint);
    // In view space the camera sits at the origin looking down -z, so anything
    // with z >= -near is on or behind the near plane and cannot be drawn.
    if (v.z >= -0.001f) return false;
    viewToScreen(v, outX, outY);
    return true;
}

// --- Near-plane clipping ---------------------------------------------------
// A triangle can straddle the camera plane: part in front, part behind. If we
// naively projected a behind-camera vertex we would get a garbage position
// (division by a negative/zero w). So we clip the triangle polygon against the
// plane z = -near in VIEW space, keeping only the in-front portion. The result
// is 0, 1, or 2 triangles (a clipped triangle can become a quad).
//
// This is Sutherland–Hodgman clipping against a single plane. "Inside" means
// far enough in front of the camera: z <= -near.
namespace {
constexpr Real kNearClip = 0.02f;   // view-space near plane distance

std::vector<Vec3> clipAgainstNear(const std::array<Vec3, 3>& tri) {
    // signed distance: positive when the point is on the keep side (in front)
    auto sdist = [](const Vec3& p) { return -kNearClip - p.z; };

    std::vector<Vec3> out;
    out.reserve(4);
    for (int i = 0; i < 3; ++i) {
        const Vec3& cur  = tri[i];
        const Vec3& next = tri[(i + 1) % 3];
        const Real  dCur  = sdist(cur);
        const Real  dNext = sdist(next);
        const bool  inCur  = dCur  >= 0;
        const bool  inNext = dNext >= 0;

        if (inCur) out.push_back(cur);
        // If the edge crosses the plane, insert the intersection point.
        if (inCur != inNext) {
            const Real t = dCur / (dCur - dNext);   // where along the edge it crosses
            out.push_back(lerp(cur, next, t));
        }
    }
    return out;
}
}  // namespace

void Renderer3D::drawMesh(const Mesh& mesh, const Mat4& model, const Color& color,
                          bool doubleSided) {
    const Vec3 toLight = -lightDir_;   // direction from surface toward the light
    const int  triCount = mesh.triangleCount();

    for (int t = 0; t < triCount; ++t) {
        const int i0 = mesh.indices[t * 3 + 0];
        const int i1 = mesh.indices[t * 3 + 1];
        const int i2 = mesh.indices[t * 3 + 2];

        // Stage 1: model → world. Place the triangle in the scene.
        const Vec3 w0 = transformPoint(model, mesh.positions[i0]);
        const Vec3 w1 = transformPoint(model, mesh.positions[i1]);
        const Vec3 w2 = transformPoint(model, mesh.positions[i2]);

        // Flat-shading normal, from the world-space triangle. CCW winding (see
        // mesh.cpp) makes this point OUT of the surface. Skip degenerate
        // triangles (zero area) such as the ones at a UV sphere's poles.
        const Vec3 faceCross = cross(w1 - w0, w2 - w0);
        if (lengthSq(faceCross) < 1e-12f) continue;
        const Vec3 normal = normalize(faceCross);

        // Stage 2: world → view. Move everything in front of the camera.
        const Vec3 v0 = transformPoint(view_, w0);
        const Vec3 v1 = transformPoint(view_, w1);
        const Vec3 v2 = transformPoint(view_, w2);

        // Stage 3: backface culling. A face pointing away from the camera is
        // never visible on a closed solid, so we skip it — halving the work.
        // The view-space normal points toward the camera for front faces, so
        // dot(viewNormal, centroid) < 0 means "facing us". Skipped in
        // wireframe mode, where we want to see every edge.
        const Vec3 viewNormal = cross(v1 - v0, v2 - v0);
        const Vec3 centroid   = (v0 + v1 + v2) * (Real(1) / Real(3));
        if (!wireframe_ && !doubleSided && dot(viewNormal, centroid) >= 0) continue;

        // Stage 4: flat shading. One Lambert term — brightness is how squarely
        // the surface faces the light — lifted by an ambient floor so shadowed
        // faces stay readable. A double-sided surface is lit on whichever side
        // faces the light, so we take the absolute value of the dot product.
        const Real facing     = dot(normal, toLight);
        const Real lambert    = doubleSided ? std::fabs(facing) : std::max(Real(0), facing);
        const Real brightness = ambient_ + (Real(1) - ambient_) * lambert;
        const Color shaded    = color.shaded(brightness);

        // Stage 5: near-plane clip (view space), then fan-triangulate whatever
        // polygon survives.
        const std::vector<Vec3> poly = clipAgainstNear({v0, v1, v2});
        if (poly.size() < 3) continue;   // entirely behind the camera

        for (size_t k = 1; k + 1 < poly.size(); ++k) {
            const Vec3& a = poly[0];
            const Vec3& b = poly[k];
            const Vec3& c = poly[k + 1];

            ScreenTri st;
            viewToScreen(a, st.x[0], st.y[0]);
            viewToScreen(b, st.x[1], st.y[1]);
            viewToScreen(c, st.x[2], st.y[2]);
            st.color = shaded;
            // Depth key = average view-space z. Camera looks down -z, so more
            // negative = farther away → drawn first (painter's algorithm).
            st.depth = (a.z + b.z + c.z) * (Real(1) / Real(3));
            tris_.push_back(st);
        }
    }
}

void Renderer3D::drawLine(const Vec3& a, const Vec3& b, const Color& color) {
    // Lines get the same near-plane treatment as triangles, but for a segment
    // clipping is just moving the behind-camera endpoint onto the plane.
    Vec3 va = transformPoint(view_, a);
    Vec3 vb = transformPoint(view_, b);
    auto sdist = [](const Vec3& p) { return -kNearClip - p.z; };
    Real da = sdist(va), db = sdist(vb);

    if (da < 0 && db < 0) return;          // both behind the camera → nothing
    if (da < 0 || db < 0) {                // one behind → clip to the plane
        const Real tClip = da / (da - db);
        const Vec3 hit   = lerp(va, vb, tClip);
        if (da < 0) va = hit; else vb = hit;
    }

    ScreenLine sl;
    viewToScreen(va, sl.x0, sl.y0);
    viewToScreen(vb, sl.x1, sl.y1);
    sl.color = color;
    lines_.push_back(sl);
}

void Renderer3D::end() {
    if (wireframe_) {
        // Wireframe: draw the three edges of every queued triangle. Depth order
        // matters less here, so we skip the sort.
        for (const ScreenTri& s : tris_) {
            SDL_SetRenderDrawColorFloat(renderer_, s.color.r, s.color.g, s.color.b, 1.0f);
            SDL_RenderLine(renderer_, s.x[0], s.y[0], s.x[1], s.y[1]);
            SDL_RenderLine(renderer_, s.x[1], s.y[1], s.x[2], s.y[2]);
            SDL_RenderLine(renderer_, s.x[2], s.y[2], s.x[0], s.y[0]);
        }
    } else {
        // Painter's algorithm: sort far-to-near (most negative z first) so that
        // nearer triangles are painted over farther ones.
        std::sort(tris_.begin(), tris_.end(),
                  [](const ScreenTri& a, const ScreenTri& b) { return a.depth < b.depth; });

        // Flatten into one big vertex array and submit in a single call — this
        // is the whole reason the software pipeline is fast enough.
        std::vector<SDL_Vertex> verts;
        verts.reserve(tris_.size() * 3);
        for (const ScreenTri& s : tris_) {
            for (int k = 0; k < 3; ++k) {
                SDL_Vertex v;
                v.position  = SDL_FPoint{s.x[k], s.y[k]};
                v.color     = SDL_FColor{s.color.r, s.color.g, s.color.b, s.color.a};
                v.tex_coord = SDL_FPoint{0, 0};
                verts.push_back(v);
            }
        }
        if (!verts.empty()) {
            SDL_RenderGeometry(renderer_, nullptr, verts.data(),
                               static_cast<int>(verts.size()), nullptr, 0);
        }
    }

    // Debug lines always draw last, on top of the shaded scene.
    for (const ScreenLine& l : lines_) {
        SDL_SetRenderDrawColorFloat(renderer_, l.color.r, l.color.g, l.color.b, l.color.a);
        SDL_RenderLine(renderer_, l.x0, l.y0, l.x1, l.y1);
    }
}

}  // namespace p3d
