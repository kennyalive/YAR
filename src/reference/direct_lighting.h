#pragma once

#include "lib/material.h"
#include "lib/random.h"
#include "lib/vector.h"

struct Local_Geometry;
struct Render_Context;

ColorRGB compute_direct_lighting(
    const Render_Context& ctx,
    const Local_Geometry& local_geom,
    const Vector3& wo,
    Material_Handle material,
    pcg32_random_t* rng);
