// ===========================================================================
//  color.hpp — a tiny RGBA color, in the 0..1 float space SDL wants
// ---------------------------------------------------------------------------
//  Introduced in: Chapter 4 (Camera & Projection / the software renderer)
//
//  SDL_RenderGeometry takes per-vertex colors as SDL_FColor (four floats in
//  0..1). Rather than sprinkle SDL types through the math-y parts of the
//  engine, we carry our own Color and convert at the boundary. Keeping colors
//  as floats also makes shading trivial: multiplying by a brightness factor is
//  just scaling the channels.
// ===========================================================================
#pragma once

#include "../math/scalar.hpp"

namespace p3d {

struct Color {
    Real r{1}, g{1}, b{1}, a{1};   // opaque white by default

    constexpr Color() = default;
    constexpr Color(Real r_, Real g_, Real b_, Real a_ = 1) : r(r_), g(g_), b(b_), a(a_) {}

    // Build from 0..255 byte values, the way colors are usually quoted.
    static constexpr Color bytes(int r_, int g_, int b_, int a_ = 255) {
        return {r_ / Real(255), g_ / Real(255), b_ / Real(255), a_ / Real(255)};
    }

    // Multiply the RGB channels by a brightness factor (alpha untouched). This
    // is exactly what flat shading does with a Lambert term.
    constexpr Color shaded(Real k) const { return {r * k, g * k, b * k, a}; }
};

// A small, readable palette so scene code says `palette::sky` not raw floats.
namespace palette {
inline constexpr Color white  = Color::bytes(240, 240, 245);
inline constexpr Color slate  = Color::bytes(110, 120, 135);
inline constexpr Color sky    = Color::bytes( 90, 160, 235);
inline constexpr Color teal   = Color::bytes( 40, 200, 190);
inline constexpr Color amber  = Color::bytes(245, 175,  70);
inline constexpr Color coral  = Color::bytes(240, 100,  95);
inline constexpr Color violet = Color::bytes(165, 120, 235);
inline constexpr Color lime   = Color::bytes(150, 210,  90);
inline constexpr Color ground = Color::bytes( 55,  60,  72);
}  // namespace palette

}  // namespace p3d
