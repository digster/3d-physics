// ===========================================================================
//  Chapter 13 — Narrowphase I: Spheres & Planes
// ---------------------------------------------------------------------------
//  Goal: turn a "these might touch" candidate pair into an exact CONTACT — a
//  point, a normal, and a penetration depth. Those three numbers are what every
//  collision response in Part V will consume. Here we generate them for the
//  easy shapes: spheres against each other, and spheres against planes.
//
//  Balls rain into an open box. Broadphase (Chapter 12) narrows the sphere-sphere
//  pairs; then the narrowphase tests below produce contacts, which we DRAW (a dot
//  at the point, an arrow along the normal). To keep the pile watchable we also
//  do the crudest possible fix — shove overlapping shapes apart and cancel the
//  velocity heading into the surface. That is NOT the real solver: bounciness,
//  friction, and stable stacking are Part V. This chapter is about FINDING the
//  contact, not resolving it properly.
//
//  Engine: physics/narrowphase.hpp, physics/broadphase.hpp, physics/particle.hpp.
//  Companion page: docs/chapters/13-narrowphase.html
// ===========================================================================
#include "common/app.hpp"
#include "common/render/debug_draw.hpp"
#include "physics/particle.hpp"
#include "physics/forces.hpp"
#include "physics/narrowphase.hpp"
#include "physics/broadphase.hpp"

#include <random>
#include <vector>

using namespace p3d;

struct NarrowphaseDemo : App {
    struct Plane { Vec3 n; Real offset; };

    std::vector<Particle> balls;
    std::vector<Real>     radius;
    std::vector<Plane>    planes;         // container walls (floor + 4 sides)
    std::vector<Contact>  contacts;       // this frame's contacts, for drawing
    std::mt19937 rng{7};
    Real spawnTimer = 0;
    static constexpr Real kW = 3.2f;      // container half-width
    Real bigRadius = 0;                   // a static obstacle sphere (scene 3)
    Vec3 bigPos{0, 1.4f, 0};

    Mesh ball = mesh::sphere(1.0f, 8, 12); // unit sphere, scaled per body
    Mesh obstacle = mesh::sphere(1.0f, 14, 20);

    NarrowphaseDemo() {
        camera.target   = {0, 1.5f, 0};
        camera.distance = 11;
        camera.pitch    = radians(16);
    }

    std::string caption() const override { return "Ch13: Narrowphase — Spheres & Planes"; }

    void onReset() override {
        balls.clear(); radius.clear(); contacts.clear(); spawnTimer = 0;
        // Container: floor + four inward-facing walls.
        planes = {
            {{0, 1, 0}, 0.0f},
            {{-1, 0, 0}, -kW}, {{1, 0, 0}, -kW},
            {{0, 0, -1}, -kW}, {{0, 0, 1}, -kW},
        };
        bigRadius = (sceneVariant() == 3) ? 1.1f : 0.0f;
    }

    void spawnBall() {
        std::uniform_real_distribution<Real> px(-kW + 0.6f, kW - 0.6f);
        std::uniform_real_distribution<Real> rr(0.28f, 0.5f);
        Particle p; p.setMass(1.0f);
        p.position = {px(rng), 4.5f, px(rng)};
        p.velocity = {0, -1.0f, 0};
        balls.push_back(p);
        radius.push_back(rr(rng));
    }

    void fixedUpdate(Real dt) override {
        const int cap = (sceneVariant() == 2) ? 90 : 55;
        spawnTimer += dt;
        if (spawnTimer > 0.10f && static_cast<int>(balls.size()) < cap) { spawnTimer = 0; spawnBall(); }

        const Vec3 g{0, -9.81f, 0};
        for (Particle& b : balls) { b.clearForces(); force::gravity(b, g); b.integrateForces(dt); }

        // --- Detect all contacts, then apply the crude positional fix ---------
        contacts.clear();
        const int n = static_cast<int>(balls.size());

        // Sphere vs each container plane.
        for (int i = 0; i < n; ++i) {
            Contact c;
            for (const Plane& pl : planes)
                if (collideSpherePlane(balls[i].position, radius[i], pl.n, pl.offset, c)) {
                    c.a = i; c.b = -1; contacts.push_back(c);
                }
            if (bigRadius > 0) {   // scene 3: a big static sphere in the middle
                if (collideSphereSphere(balls[i].position, radius[i], bigPos, bigRadius, c)) {
                    c.a = i; c.b = -2; contacts.push_back(c);
                }
            }
        }
        // Sphere vs sphere, but only on broadphase candidate pairs.
        std::vector<AABB> boxes(n);
        for (int i = 0; i < n; ++i) boxes[i] = aabbOfSphere(balls[i].position, radius[i]);
        for (const BroadPair& pr : broadphaseGrid(boxes, 1.2f)) {
            Contact c;
            if (collideSphereSphere(balls[pr.first].position, radius[pr.first],
                                    balls[pr.second].position, radius[pr.second], c)) {
                c.a = pr.first; c.b = pr.second; contacts.push_back(c);
            }
        }

        // Crude resolution: separate along the normal, cancel the inward velocity.
        // (No restitution/friction — that is Part V.) A few passes for stability.
        for (int pass = 0; pass < 4; ++pass)
            for (const Contact& c : contacts) resolve(c);
    }

    void resolve(const Contact& c) {
        Particle& a = balls[c.a];
        if (c.b < 0) {   // vs a static plane (-1) or the static obstacle (-2)
            a.position += c.normal * c.penetration;
            const Real vn = dot(a.velocity, c.normal);
            if (vn < 0) a.velocity -= c.normal * vn;   // remove the inward part
        } else {
            Particle& b = balls[c.b];
            const Real wsum = a.invMass + b.invMass;
            if (wsum <= 0) return;
            const Vec3 corr = c.normal * (c.penetration / wsum);
            a.position += corr * a.invMass;
            b.position -= corr * b.invMass;
            const Real vn = dot(a.velocity - b.velocity, c.normal);
            if (vn < 0) {
                const Vec3 j = c.normal * (-vn / wsum);
                a.velocity += j * a.invMass;
                b.velocity -= j * b.invMass;
            }
        }
    }

    void render(Renderer3D& r, Real) override {
        debug::grid(r, 5, 10);
        // Draw the container walls as a wire box (open top).
        debug::aabb(r, {-kW, 0, -kW}, {kW, 5.5f, kW}, Color::bytes(70, 76, 92));
        if (bigRadius > 0) r.drawMesh(obstacle, translation(bigPos) * scaling(Vec3(bigRadius)), palette::slate);

        for (size_t i = 0; i < balls.size(); ++i)
            r.drawMesh(ball, translation(balls[i].position) * scaling(Vec3(radius[i])), palette::sky);

        // The contacts: a coral dot at each point, a short arrow along the normal.
        for (const Contact& c : contacts) {
            debug::point(r, c.point, 0.05f, palette::coral);
            debug::arrow(r, c.point, c.point + c.normal * 0.4f, palette::amber);
        }
    }

    std::vector<std::string> hudLines() const override {
        char buf[96];
        std::snprintf(buf, sizeof(buf), "balls=%d   contacts this frame=%d",
                      int(balls.size()), int(contacts.size()));
        return {buf, "coral dot = contact point, amber arrow = normal",
                "1 box   2 more balls   3 + static obstacle   (crude settle; real solver = Part V)"};
    }
};

int main(int argc, char** argv) { return NarrowphaseDemo{}.run(argc, argv); }
