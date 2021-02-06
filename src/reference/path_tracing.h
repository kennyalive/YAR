#pragma once

#include "lib/color.h"

struct Auxilary_Rays;
struct Ray;
struct Scene_Context;
struct Thread_Context;

ColorRGB estimate_path_contribution(
    const Scene_Context& scene_ctx, Thread_Context& thread_ctx,
    const Ray& ray, const Auxilary_Rays& auxilary_rays);
