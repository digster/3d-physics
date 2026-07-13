// ===========================================================================
//  broadphase.hpp — quickly find pairs that MIGHT be colliding
// ---------------------------------------------------------------------------
//  Introduced in: Chapter 12 (Broadphase)
//
//  With N bodies there are N·(N−1)/2 possible pairs — testing all of them is
//  O(N²) and dies for large N (1000 bodies → half a million tests, every frame).
//  Broadphase cuts that down: using only cheap AABBs, it rejects the vast
//  majority of pairs that are nowhere near each other, and hands the few
//  survivors ("candidate pairs") to the expensive narrowphase.
//
//  We provide three classic strategies so you can compare them:
//    * brute force        — the O(N²) baseline (correct, slow; the yardstick).
//    * sweep-and-prune    — sort by one axis, sweep a line across; only nearby
//                           boxes are ever compared. Great when motion is small.
//    * uniform grid       — drop boxes into a grid of cells; only boxes sharing a
//                           cell can be paired. Great when bodies are evenly sized.
//  All three return the SAME set of candidate pairs (the tests below check that).
// ===========================================================================
#pragma once

#include "aabb.hpp"
#include <vector>
#include <utility>

namespace p3d {

// A candidate pair, stored with first < second so pairs are comparable/sortable.
using BroadPair = std::pair<int, int>;

// The O(N²) baseline: test every pair's AABBs directly.
std::vector<BroadPair> broadphaseBrute(const std::vector<AABB>& boxes, Real margin = 0);

// Sweep-and-prune along the x axis: sort by each box's minimum x, then sweep,
// keeping only an "active" set whose x-spans still overlap the current box.
std::vector<BroadPair> broadphaseSweep(const std::vector<AABB>& boxes, Real margin = 0);

// Uniform spatial grid: hash each box into the cells it covers; pair up boxes
// that share a cell. `cellSize` should be roughly the size of a typical body.
std::vector<BroadPair> broadphaseGrid(const std::vector<AABB>& boxes, Real cellSize,
                                      Real margin = 0);

}  // namespace p3d
