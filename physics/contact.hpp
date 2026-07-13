// ===========================================================================
//  contact.hpp — the Contact: what the whole of Part IV produces
// ---------------------------------------------------------------------------
//  Introduced in: Chapter 13 (Narrowphase)
//
//  Collision DETECTION has one job: when two shapes overlap, describe the
//  overlap precisely enough that something else can fix it. That description is
//  a Contact. Every narrowphase test in this part fills one (or a few) of these,
//  and every solver in Part V will consume them. Getting the fields — and
//  especially the normal's DIRECTION — pinned down once saves endless sign bugs.
// ===========================================================================
#pragma once

#include "../common/math/vec3.hpp"

namespace p3d {

struct Contact {
    int  a{-1};          // index of the first body
    int  b{-1};          // index of the second body (-1 for a static plane/world)

    Vec3 point{};        // a representative contact point, in world space
    Vec3 normal{};       // unit contact normal — see the convention below
    Real penetration{0}; // how deep the overlap is (>0 when overlapping)

    // CONVENTION: `normal` points FROM B TOWARD A — that is, it is the direction
    // you would push body A to pull the two apart. To separate them, move
    //     A along +normal   and   B along -normal,
    // sharing the move by inverse mass. Keeping this fixed everywhere means the
    // solver never has to guess which way "out" is.
};

}  // namespace p3d
