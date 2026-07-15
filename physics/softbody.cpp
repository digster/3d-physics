// ===========================================================================
//  softbody.cpp — building an XPBD cube and stepping it
// ===========================================================================
#include "softbody.hpp"

#include <cmath>

namespace p3d {

void SoftBody::addSpring(int a, int b, Real compliance) {
    springs.push_back({a, b, distance(particles[a].position, particles[b].position), compliance, 0});
}

void SoftBody::buildCube(const Vec3& center, Real size, int n, Real compliance, Real totalMass) {
    particles.clear(); springs.clear();
    nx = ny = nz = n;
    const Real spacing = size / Real(n - 1);
    const Vec3 origin = center - Vec3(size * Real(0.5));
    const Real perMass = totalMass / Real(n * n * n);

    // The particle lattice.
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            for (int k = 0; k < n; ++k) {
                Particle p;
                p.position = origin + Vec3{spacing * i, spacing * j, spacing * k};
                p.setMass(perMass);
                particles.push_back(p);
            }

    // Weave constraints. Each particle connects to neighbours at these offsets;
    // we only add a spring when the neighbour's index is larger, so each pair is
    // counted once. Structural (axes) + shear (face diagonals) + volume (body
    // diagonals) together stop the cube from collapsing or shearing flat.
    static const int offsets[][3] = {
        {1,0,0}, {0,1,0}, {0,0,1},                                   // structural
        {1,1,0}, {1,-1,0}, {1,0,1}, {1,0,-1}, {0,1,1}, {0,1,-1},     // shear (face diagonals)
        {1,1,1}, {1,1,-1}, {1,-1,1}, {1,-1,-1},                      // volume (body diagonals)
    };
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            for (int k = 0; k < n; ++k) {
                const int a = index(i, j, k);
                for (auto& o : offsets) {
                    const int ni = i + o[0], nj = j + o[1], nk = k + o[2];
                    if (ni < 0 || ni >= n || nj < 0 || nj >= n || nk < 0 || nk >= n) continue;
                    const int b = index(ni, nj, nk);
                    if (b > a) addSpring(a, b, compliance);
                }
            }

    buildSurface();
    updateMesh();
}

// Build the triangle list for the six outer faces of the lattice. The soft body
// is drawn double-sided, so winding doesn't matter — we just need the surface
// quads. Indices point into the full particle array; positions are refreshed in
// updateMesh().
void SoftBody::buildSurface() {
    surfaceMesh.positions.assign(particles.size(), Vec3{});
    surfaceMesh.indices.clear();
    auto quad = [&](int a, int b, int c, int d) {
        surfaceMesh.indices.insert(surfaceMesh.indices.end(), {a, b, c, a, c, d});
    };
    for (int i = 0; i < nx - 1; ++i)
        for (int j = 0; j < ny - 1; ++j) {
            quad(index(i,j,0), index(i+1,j,0), index(i+1,j+1,0), index(i,j+1,0));               // -z
            quad(index(i,j,nz-1), index(i+1,j,nz-1), index(i+1,j+1,nz-1), index(i,j+1,nz-1));   // +z
        }
    for (int i = 0; i < nx - 1; ++i)
        for (int k = 0; k < nz - 1; ++k) {
            quad(index(i,0,k), index(i+1,0,k), index(i+1,0,k+1), index(i,0,k+1));               // -y
            quad(index(i,ny-1,k), index(i+1,ny-1,k), index(i+1,ny-1,k+1), index(i,ny-1,k+1));   // +y
        }
    for (int j = 0; j < ny - 1; ++j)
        for (int k = 0; k < nz - 1; ++k) {
            quad(index(0,j,k), index(0,j+1,k), index(0,j+1,k+1), index(0,j,k+1));               // -x
            quad(index(nx-1,j,k), index(nx-1,j+1,k), index(nx-1,j+1,k+1), index(nx-1,j,k+1));   // +x
        }
}

void SoftBody::updateMesh() {
    for (size_t i = 0; i < particles.size(); ++i)
        surfaceMesh.positions[i] = particles[i].position;
}

void SoftBody::step(Real dt, int substeps, const Vec3& gravity, Real groundY) {
    if (substeps < 1) substeps = 1;
    const Real h = dt / Real(substeps);

    for (int s = 0; s < substeps; ++s) {
        // --- Predict: move each particle freely for one tiny step ------------
        for (Particle& p : particles) {
            if (p.isStatic()) continue;
            p.prevPosition = p.position;
            p.velocity += gravity * h;
            p.position += p.velocity * h;
        }

        // --- Solve each compliant constraint once (the XPBD step) ------------
        const Real invH2 = Real(1) / (h * h);
        for (Spring& c : springs) {
            c.lambda = 0;   // XPBD resets the multiplier each substep
            Particle& pa = particles[c.a];
            Particle& pb = particles[c.b];
            const Real w = pa.invMass + pb.invMass;
            if (w < kEpsilon) continue;

            Vec3 d = pa.position - pb.position;
            const Real dist = length(d);
            if (dist < kEpsilon) continue;
            const Vec3 n = d * (Real(1) / dist);
            const Real C = dist - c.restLength;

            // α̃ = compliance / h². dλ = (−C − α̃·λ) / (w + α̃); positions move by
            // w_i · ∇C_i · dλ. With λ starting at 0 this reduces to the familiar
            // stiffness-scaled projection, but α gives a real, timestep-independent
            // springiness.
            const Real alphaTilde = c.compliance * invH2;
            const Real dLambda = (-C - alphaTilde * c.lambda) / (w + alphaTilde);
            c.lambda += dLambda;
            const Vec3 corr = n * dLambda;
            pa.position += corr * pa.invMass;
            pb.position -= corr * pb.invMass;
        }

        // --- Collide with the floor, with a touch of friction ----------------
        for (Particle& p : particles) {
            if (p.isStatic()) continue;
            if (p.position.y < groundY) {
                p.position.y = groundY;
                // Friction: pull the horizontal position part-way back toward
                // where it was, bleeding off sideways sliding.
                p.position.x += (p.prevPosition.x - p.position.x) * 0.3f;
                p.position.z += (p.prevPosition.z - p.position.z) * 0.3f;
            }
        }

        // --- Set velocity from the actual movement this substep --------------
        for (Particle& p : particles) {
            if (p.isStatic()) continue;
            p.velocity = (p.position - p.prevPosition) * (Real(1) / h);
        }
    }

    updateMesh();
}

}  // namespace p3d
