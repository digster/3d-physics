// ===========================================================================
//  math.hpp — one include for the whole math toolkit
// ---------------------------------------------------------------------------
//  Pull this in and you get scalars, vectors, matrices, and quaternions. Each
//  piece is documented in its own header; the tutorial introduces them in the
//  order below:
//     scalar.hpp  (Ch 2)  constants, angle conversion, clamp/lerp
//     vec3.hpp    (Ch 2)  the 3D vector and its products
//     vec4.hpp    (Ch 4)  homogeneous coordinates for projection
//     mat4.hpp    (Ch 3)  transforms, camera, projection
//     quat.hpp    (Ch 10) gimbal-lock-free rotation
// ===========================================================================
#pragma once

#include "scalar.hpp"
#include "vec3.hpp"
#include "vec4.hpp"
#include "mat4.hpp"
#include "quat.hpp"
