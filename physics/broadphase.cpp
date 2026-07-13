// ===========================================================================
//  broadphase.cpp — the three broadphase strategies
// ===========================================================================
#include "broadphase.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace p3d {

// Normalise a pair so the smaller index is first (makes pairs comparable, so we
// can sort and de-duplicate them).
static BroadPair ordered(int i, int j) { return (i < j) ? BroadPair{i, j} : BroadPair{j, i}; }

// ---------------------------------------------------------------------------
//  Brute force — the correct, slow O(N²) baseline.
// ---------------------------------------------------------------------------
std::vector<BroadPair> broadphaseBrute(const std::vector<AABB>& boxes, Real margin) {
    std::vector<BroadPair> pairs;
    const int n = static_cast<int>(boxes.size());
    for (int i = 0; i < n; ++i)
        for (int j = i + 1; j < n; ++j)
            if (boxes[i].overlaps(boxes[j], margin))
                pairs.push_back({i, j});
    return pairs;
}

// ---------------------------------------------------------------------------
//  Sweep-and-prune along x.
//  Sort boxes by their minimum x. Sweep left to right; keep an "active" list of
//  boxes whose x-span we might still be inside. When a box's max.x falls behind
//  the current box's min.x, it can never overlap anything further right, so we
//  drop it. Everything still active is a candidate (confirm with a full AABB
//  test, since sharing an x-span doesn't guarantee y/z overlap).
// ---------------------------------------------------------------------------
std::vector<BroadPair> broadphaseSweep(const std::vector<AABB>& boxes, Real margin) {
    const int n = static_cast<int>(boxes.size());
    std::vector<int> order(n);
    for (int i = 0; i < n; ++i) order[i] = i;
    std::sort(order.begin(), order.end(),
              [&](int a, int b) { return boxes[a].min.x < boxes[b].min.x; });

    std::vector<BroadPair> pairs;
    std::vector<int> active;
    for (int oi = 0; oi < n; ++oi) {
        const int cur = order[oi];
        const Real curMinX = boxes[cur].min.x - margin;
        // Retire actives whose x-span ends before the current box begins.
        active.erase(std::remove_if(active.begin(), active.end(),
                     [&](int a) { return boxes[a].max.x + margin < curMinX; }),
                     active.end());
        // Everything still active overlaps in x; check the full AABB.
        for (int a : active)
            if (boxes[a].overlaps(boxes[cur], margin))
                pairs.push_back(ordered(a, cur));
        active.push_back(cur);
    }
    return pairs;
}

// ---------------------------------------------------------------------------
//  Uniform grid.
//  Bucket every box into all the integer cells its AABB covers. Two boxes can
//  only touch if they share at least one cell, so we only compare within-cell
//  boxes. A box spanning several cells lands in each, which can produce the same
//  pair more than once, so we de-duplicate at the end.
// ---------------------------------------------------------------------------
namespace {
struct Cell { int x, y, z; bool operator==(const Cell& o) const { return x==o.x && y==o.y && z==o.z; } };
struct CellHash {
    size_t operator()(const Cell& c) const {
        // A simple hash-combine of the three cell coordinates.
        size_t h = static_cast<size_t>(c.x) * 73856093u;
        h ^= static_cast<size_t>(c.y) * 19349663u;
        h ^= static_cast<size_t>(c.z) * 83492791u;
        return h;
    }
};
inline int cellCoord(Real v, Real inv) { return static_cast<int>(std::floor(v * inv)); }
}  // namespace

std::vector<BroadPair> broadphaseGrid(const std::vector<AABB>& boxes, Real cellSize, Real margin) {
    const int n = static_cast<int>(boxes.size());
    const Real inv = Real(1) / cellSize;

    std::unordered_map<Cell, std::vector<int>, CellHash> grid;
    grid.reserve(static_cast<size_t>(n) * 2);
    for (int i = 0; i < n; ++i) {
        const AABB& b = boxes[i];
        const int x0 = cellCoord(b.min.x - margin, inv), x1 = cellCoord(b.max.x + margin, inv);
        const int y0 = cellCoord(b.min.y - margin, inv), y1 = cellCoord(b.max.y + margin, inv);
        const int z0 = cellCoord(b.min.z - margin, inv), z1 = cellCoord(b.max.z + margin, inv);
        for (int x = x0; x <= x1; ++x)
            for (int y = y0; y <= y1; ++y)
                for (int z = z0; z <= z1; ++z)
                    grid[{x, y, z}].push_back(i);
    }

    std::vector<BroadPair> pairs;
    for (auto& [cell, members] : grid) {
        const int m = static_cast<int>(members.size());
        for (int i = 0; i < m; ++i)
            for (int j = i + 1; j < m; ++j)
                if (boxes[members[i]].overlaps(boxes[members[j]], margin))
                    pairs.push_back(ordered(members[i], members[j]));
    }
    // De-duplicate pairs that shared more than one cell.
    std::sort(pairs.begin(), pairs.end());
    pairs.erase(std::unique(pairs.begin(), pairs.end()), pairs.end());
    return pairs;
}

}  // namespace p3d
