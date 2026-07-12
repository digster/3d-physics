// ===========================================================================
//  Chapter 2 — Vectors
// ---------------------------------------------------------------------------
//  Goal: build intuition for the two products that make 3D possible.
//    * the DOT product   a·b : "how much do these point the same way?"
//                              → a number; zero when perpendicular.
//    * the CROSS product a×b : a new vector PERPENDICULAR to both, whose
//                              length is the area of the parallelogram they
//                              span, and whose direction is the right-hand rule.
//
//  Two vectors a (teal) and b (amber) slowly swing around. We draw, live:
//    * a and b as arrows from the origin,
//    * a + b by the tip-to-tail rule (walk along a, then along b),
//    * a × b as an arrow perpendicular to both,
//    * the projection of a onto b (a's "shadow" along b).
//  The HUD reports |a|, |b|, a·b, the angle between them, and |a×b| so you can
//  watch the numbers move with the geometry.
//
//  This is our first demo built on the App framework (common/app.hpp): we only
//  write fixedUpdate() to advance time and render() to draw. Companion page:
//  docs/chapters/02-vectors.html
// ===========================================================================
#include "common/app.hpp"
#include "common/render/debug_draw.hpp"

using namespace p3d;

struct VectorsDemo : App {
    Real t = 0;
    Vec3 a, b;

    VectorsDemo() {
        camera.target   = {0, 0.5f, 0};
        camera.distance = 7;
    }

    std::string caption() const override { return "Ch2: Vectors  (dot / cross / projection)"; }

    void onReset() override { t = 0; }

    // Advance a clock; the two vectors are functions of it so they sweep
    // through many relative angles (parallel, perpendicular, obtuse...).
    void fixedUpdate(Real dt) override { t += dt; }

    void render(Renderer3D& r, Real) override {
        // Recompute the two vectors from the clock. Their exact motion is
        // arbitrary — the point is that the products below track them.
        a = Vec3{2.2f * std::cos(t * 0.6f), 1.1f + 0.6f * std::sin(t * 0.9f),
                 2.2f * std::sin(t * 0.6f)};
        b = Vec3{1.9f * std::cos(-t * 0.4f + 1.2f), 1.6f,
                 1.9f * std::sin(-t * 0.4f + 1.2f)};

        debug::grid(r);
        debug::axes(r, Vec3{}, 1.0f);

        const Vec3 O{0, 0, 0};

        // a and b themselves.
        debug::arrow(r, O, a, palette::teal);
        debug::arrow(r, O, b, palette::amber);

        // a + b, drawn tip-to-tail: a faint copy of b starting at a's tip, then
        // the resultant from the origin to the far corner.
        debug::arrow(r, a, a + b, Color::bytes(150, 120, 60));   // b, shifted
        debug::arrow(r, O, a + b, palette::violet);

        // a × b: perpendicular to the a–b plane. Scaled down so a big area does
        // not shoot off screen; it still points the right way (right-hand rule).
        const Vec3 c = cross(a, b);
        debug::arrow(r, O, c * 0.5f, palette::coral);

        // Projection of a onto b: drop a perpendicular from a's tip onto the
        // line of b. The base of that perpendicular is proj_b(a).
        const Vec3 p = project(a, b);
        debug::arrow(r, O, p, palette::lime);
        r.drawLine(a, p, Color::bytes(90, 150, 90));   // the perpendicular "drop"
    }

    std::vector<std::string> hudLines() const override {
        const Real dab   = dot(a, b);
        const Real la    = length(a), lb = length(b);
        const Real cosT  = (la > 0 && lb > 0) ? clamp(dab / (la * lb), -1, 1) : 0;
        const Real angle = degrees(std::acos(cosT));
        char buf[160];
        std::snprintf(buf, sizeof(buf),
                      "|a|=%.2f |b|=%.2f  a.b=%.2f  angle=%.0f deg  |axb|=%.2f",
                      double(la), double(lb), double(dab), double(angle),
                      double(length(cross(a, b))));
        return {buf,
                "teal=a  amber=b  violet=a+b  coral=axb  lime=proj_b(a)"};
    }
};

int main(int argc, char** argv) { return VectorsDemo{}.run(argc, argv); }
