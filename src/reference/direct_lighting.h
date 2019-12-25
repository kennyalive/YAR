#pragma once

#include "lib/color.h"
#include "lib/random.h"

struct Shading_Context;
struct Render_Context;

ColorRGB compute_direct_lighting(const Render_Context& ctx, const Shading_Context& sc, pcg32_random_t* rng);
