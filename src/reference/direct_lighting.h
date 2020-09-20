#pragma once

#include "lib/color.h"
#include "lib/random.h"
#include "lib/vector.h"

struct Shading_Context;
struct Scene_Context;
struct Thread_Context;

ColorRGB estimate_direct_lighting(const Scene_Context& scene_ctx, Thread_Context& thread_ctx, const Shading_Context& shading_ctx, int max_specular_depth = 10);
