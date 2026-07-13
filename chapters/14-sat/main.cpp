// ===========================================================================
//  Chapter 14 — Narrowphase II: SAT & Manifolds
// ---------------------------------------------------------------------------
//  Goal: collide two ORIENTED boxes. The Separating-Axis Theorem gives the
//  contact normal (the axis of shallowest overlap among 15 candidates), and
//  clipping the two touching faces gives a MANIFOLD — several contact points, not
//  just one. A manifold is what lets a box rest flat instead of teetering on a
//  single point (crucial for the stable stacking of Part V).
//
//  Scenes:
//    1  Two boxes, one sliding through the other. Watch the manifold's points
//       appear, slide, and vanish, and the normal flip to the shallowest axis.
//    2  A little stack settling on the floor (box-plane + box-box), with the
//       manifolds drawn — again only crude positional separation, not Part V.
//    3  A tilted box lowered corner-first onto a flat one: the edge-edge case,
//       where SAT's separating axis is a cross-product of edges, not a face.
//
//  Engine: physics/sat.hpp, physics/narrowphase.hpp. Companion page:
//  docs/chapters/14-sat.html
// ===========================================================================
#include "common/app.hpp"
#include "common/render/debug_draw.hpp"
#include "physics/sat.hpp"
#include "physics/narrowphase.hpp"

#include <vector>

using namespace p3d;

struct Box { Vec3 pos, vel; Quat rot; Vec3 half; Real invMass; };

struct SatDemo : App {
    std::vector<Box> boxes;
    std::vector<Contact> contacts;
    Real t = 0;
    Mesh unit = mesh::box({1, 1, 1});

    SatDemo() {
        camera.target   = {0, 1.2f, 0};
        camera.distance = 10;
        camera.pitch    = radians(14);
    }

    std::string caption() const override { return "Ch14: SAT & Manifolds"; }

    static Box make(const Vec3& p, const Vec3& h, const Quat& r = Quat::identity(), Real invMass = 1) {
        return {p, Vec3{}, r, h, invMass};
    }

    void onReset() override {
        boxes.clear(); contacts.clear(); t = 0;
        const int v = sceneVariant();
        if (v == 2) {
            // A stack: a big static base, then a few axis-aligned boxes to settle.
            boxes.push_back(make({0, 0.4f, 0}, {2.0f, 0.4f, 2.0f}, Quat::identity(), 0));
            boxes.push_back(make({-0.2f, 1.6f, 0.1f}, {0.7f, 0.5f, 0.7f}));
            boxes.push_back(make({0.15f, 2.9f, -0.1f}, {0.6f, 0.5f, 0.6f}));
            boxes.push_back(make({0.0f, 4.1f, 0.05f}, {0.5f, 0.45f, 0.5f}));
        } else {
            // Scenes 1 & 3 are kinematic: two boxes we drive by hand and inspect.
            const Quat tilt = (v == 3) ? Quat::fromAxisAngle({0, 0, 1}, radians(45)) : Quat::identity();
            boxes.push_back(make({0, 0.9f, 0}, {1.0f, 0.9f, 1.0f}, Quat::identity(), 0));  // fixed target
            boxes.push_back(make({0, 0.9f, 0}, {0.7f, 0.7f, 0.7f}, tilt, 0));               // driven
        }
    }

    void fixedUpdate(Real dt) override {
        t += dt;
        const int v = sceneVariant();

        if (v == 1) {                 // slide box[1] left-right through box[0]
            boxes[1].pos = {2.4f * std::sin(t * 0.6f), 0.9f, 0};
        } else if (v == 3) {          // lower the tilted box corner-first onto box[0]
            boxes[1].pos = {0, 2.6f + 1.4f * std::sin(t * 0.5f), 0};
        } else {                      // scene 2: gravity + crude positional settle
            for (Box& b : boxes) if (b.invMass > 0) { b.vel.y -= 9.81f * dt; b.pos += b.vel * dt; }
        }

        collectContacts();
        if (v == 2)
            for (int pass = 0; pass < 6; ++pass) { collectContacts(); resolve(); }
    }

    void collectContacts() {
        contacts.clear();
        const int n = static_cast<int>(boxes.size());
        Contact m[8];
        // Box vs box (SAT) for every pair.
        for (int i = 0; i < n; ++i)
            for (int j = i + 1; j < n; ++j) {
                const int k = collideBoxBox(boxes[i].pos, boxes[i].rot, boxes[i].half,
                                            boxes[j].pos, boxes[j].rot, boxes[j].half, m, 8);
                for (int c = 0; c < k; ++c) { m[c].a = i; m[c].b = j; contacts.push_back(m[c]); }
            }
        // Box vs floor plane (scene 2 only; scenes 1/3 have a static base box).
        if (sceneVariant() == 2)
            for (int i = 0; i < n; ++i) if (boxes[i].invMass > 0) {
                const int k = collideBoxPlane(boxes[i].pos, boxes[i].rot, boxes[i].half, {0, 1, 0}, 0, m);
                for (int c = 0; c < k; ++c) { m[c].a = i; m[c].b = -1; contacts.push_back(m[c]); }
            }
    }

    void resolve() {
        for (const Contact& c : contacts) {
            Box& a = boxes[c.a];
            if (c.b < 0) {
                if (a.invMass <= 0) continue;
                a.pos += c.normal * c.penetration;
                if (dot(a.vel, c.normal) < 0) a.vel -= c.normal * dot(a.vel, c.normal);
            } else {
                Box& b = boxes[c.b];
                const Real wsum = a.invMass + b.invMass;
                if (wsum <= 0) continue;
                const Vec3 corr = c.normal * (c.penetration / wsum * 0.6f);   // relax a bit
                a.pos += corr * a.invMass;
                b.pos -= corr * b.invMass;
            }
        }
    }

    void render(Renderer3D& r, Real) override {
        debug::grid(r, 5, 10);
        const Color cols[2] = {palette::teal, palette::amber};
        for (size_t i = 0; i < boxes.size(); ++i) {
            const Box& b = boxes[i];
            r.drawMesh(unit, translation(b.pos) * toMat4(b.rot) * scaling(b.half),
                       (b.invMass == 0 && sceneVariant() != 2) ? cols[i % 2]
                       : (i == 0 ? palette::slate : cols[i % 2]));
        }

        // Draw the manifold: a dot per contact point, plus one big normal arrow
        // at the manifold's centre so the SAT axis is obvious.
        if (!contacts.empty()) {
            Vec3 centre{};
            for (const Contact& c : contacts) { centre += c.point; debug::point(r, c.point, 0.06f, palette::coral); }
            centre = centre * (Real(1) / contacts.size());
            debug::arrow(r, centre, centre + contacts[0].normal * 1.0f, palette::lime);
        }
    }

    std::vector<std::string> hudLines() const override {
        char buf[80];
        std::snprintf(buf, sizeof(buf), "contact points in manifold: %d", int(contacts.size()));
        return {buf, "coral dots = manifold, green arrow = SAT normal",
                "1 slide-through   2 stack   3 tilted (edge-edge)"};
    }
};

int main(int argc, char** argv) { return SatDemo{}.run(argc, argv); }
