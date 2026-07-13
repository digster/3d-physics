// ===========================================================================
//  Chapter 15 — GJK & EPA
// ---------------------------------------------------------------------------
//  Goal: detect collisions between ARBITRARY convex shapes with one algorithm.
//  SAT needed a shape-specific axis list; GJK needs only a "support function"
//  (farthest point in a direction), so the same code handles a cube, an
//  octahedron, a tetrahedron — anything convex. GJK asks "do they overlap?" by
//  hunting for the origin inside their Minkowski difference; EPA then digs out
//  the penetration normal and depth.
//
//  One shape drifts through another. When they overlap we colour them coral and
//  draw EPA's penetration vector — the shortest push that would separate them;
//  otherwise they stay teal. It's the same Contact (normal + depth) the box
//  tests produced, but now for shapes SAT never knew about.
//
//  Engine: physics/gjk.hpp. Companion page: docs/chapters/15-gjk.html
// ===========================================================================
#include "common/app.hpp"
#include "common/render/debug_draw.hpp"
#include "physics/gjk.hpp"

#include <vector>

using namespace p3d;

// A convex shape: its vertices in local space, plus a mesh to draw it.
struct Hull { std::vector<Vec3> local; Mesh mesh; };

static Hull makeOctahedron(Real r) {
    Hull h;
    h.local = {{r,0,0},{-r,0,0},{0,r,0},{0,-r,0},{0,0,r},{0,0,-r}};
    // 8 faces (winding irrelevant — we draw it double-sided).
    const Vec3* v = h.local.data();
    auto tri = [&](int a, int b, int c) {
        int base = static_cast<int>(h.mesh.positions.size());
        h.mesh.positions.push_back(v[a]); h.mesh.positions.push_back(v[b]); h.mesh.positions.push_back(v[c]);
        h.mesh.indices.push_back(base); h.mesh.indices.push_back(base+1); h.mesh.indices.push_back(base+2);
    };
    tri(2,0,4); tri(2,4,1); tri(2,1,5); tri(2,5,0);
    tri(3,4,0); tri(3,1,4); tri(3,5,1); tri(3,0,5);
    return h;
}
static Hull makeCube(Real s) {
    Hull h;
    for (int x=-1;x<=1;x+=2) for (int y=-1;y<=1;y+=2) for (int z=-1;z<=1;z+=2)
        h.local.push_back({x*s, y*s, z*s});
    h.mesh = mesh::box({s, s, s});
    return h;
}
static Hull makeTetra(Real r) {
    Hull h;
    h.local = {{r,r,r},{r,-r,-r},{-r,r,-r},{-r,-r,r}};
    const Vec3* v = h.local.data();
    auto tri = [&](int a,int b,int c){ int base=(int)h.mesh.positions.size();
        h.mesh.positions.push_back(v[a]); h.mesh.positions.push_back(v[b]); h.mesh.positions.push_back(v[c]);
        h.mesh.indices.push_back(base); h.mesh.indices.push_back(base+1); h.mesh.indices.push_back(base+2); };
    tri(0,1,2); tri(0,2,3); tri(0,3,1); tri(1,3,2);
    return h;
}

struct GjkDemo : App {
    Hull oct = makeOctahedron(1.1f);
    Hull cube = makeCube(0.8f);
    Hull tet = makeTetra(1.2f);
    Real t = 0;

    GjkDemo() {
        camera.target   = {0, 0, 0};
        camera.distance = 8;
        camera.pitch    = radians(14);
    }

    std::string caption() const override { return "Ch15: GJK & EPA"; }
    void onReset() override { t = 0; }
    void fixedUpdate(Real dt) override { t += dt; }

    const Hull& shapeA() const { return (sceneVariant() == 3) ? tet : oct; }
    const Hull& shapeB() const { return (sceneVariant() == 2) ? oct : cube; }

    static std::vector<Vec3> toWorld(const Hull& h, const Vec3& pos, const Quat& rot) {
        std::vector<Vec3> w; w.reserve(h.local.size());
        for (const Vec3& v : h.local) w.push_back(pos + rotate(rot, v));
        return w;
    }

    void render(Renderer3D& r, Real) override {
        debug::grid(r, 5, 10);

        // Shape A sits near the origin, slowly rotating. Shape B drifts through it.
        const Vec3 posA{0, 0, 0};
        const Quat rotA = Quat::fromAxisAngle(normalize(Vec3{0.3f, 1, 0.2f}), t * 0.5f);
        const Vec3 posB{2.6f * std::sin(t * 0.5f), 0.4f * std::cos(t * 0.9f), 0};
        const Quat rotB = Quat::fromAxisAngle(normalize(Vec3{1, 0.4f, 0.6f}), t * 0.8f);

        const Hull& A = shapeA();
        const Hull& B = shapeB();
        const std::vector<Vec3> wa = toWorld(A, posA, rotA);
        const std::vector<Vec3> wb = toWorld(B, posB, rotB);

        const Penetration pen = gjkEpa(wa, wb);
        const Color c = pen.hit ? palette::coral : palette::teal;

        r.drawMesh(A.mesh, translation(posA) * toMat4(rotA), c, /*doubleSided=*/true);
        r.drawMesh(B.mesh, translation(posB) * toMat4(rotB), c, /*doubleSided=*/true);

        if (pen.hit) {
            // EPA's penetration vector: the shortest push (normal · depth) that
            // separates the two shapes. Drawn from the contact point.
            debug::point(r, pen.point, 0.08f, palette::white);
            debug::arrow(r, pen.point, pen.point + pen.normal * pen.depth, palette::lime);
            lastDepth_ = pen.depth;
        } else {
            lastDepth_ = 0;
        }
        lastHit_ = pen.hit;
    }

    std::vector<std::string> hudLines() const override {
        char buf[96];
        std::snprintf(buf, sizeof(buf), "%s   penetration depth = %.3f",
                      lastHit_ ? "OVERLAPPING (GJK yes -> EPA)" : "separate (GJK no)", double(lastDepth_));
        return {buf, "green arrow = EPA push-out vector (normal x depth)",
                "1 cube+octahedron   2 two octahedra   3 tetra+cube"};
    }

    bool lastHit_ = false;
    Real lastDepth_ = 0;
};

int main(int argc, char** argv) { return GjkDemo{}.run(argc, argv); }
