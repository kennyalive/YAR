#pragma once

#include "lib/color.h"
#include "lib/random.h"
#include "lib/vector.h"

struct Shading_Context;
struct Render_Context;
struct Thread_Context;

ColorRGB estimate_direct_lighting(const Render_Context& ctx, Thread_Context& thread_ctx, const Shading_Context& sc, pcg32_random_t* rng, int max_specular_depth = 10);
ColorRGB sample_environment_map_radiance(const Render_Context& ctx, const Vector3& world_direction);
ColorRGB sample_environment_map_radiance(const Render_Context& ctx, Vector2 uv);
