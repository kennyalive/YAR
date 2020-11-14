#pragma once

#include "lib/color.h"

struct Scene_Context;
struct Thread_Context;
struct Shading_Context;

ColorRGB estimate_path_contribution(const Scene_Context& scene_ctx, Thread_Context& thread_ctx, const Shading_Context& shading_ctx);
