#pragma once

#include "lib/color.h"
#include "lib/vector.h"

struct Auxilary_Rays;
struct Ray;
struct Thread_Context;

ColorRGB estimate_direct_lighting(Thread_Context& thread_ctx, const Ray& ray, const Auxilary_Rays& auxilary_rays,
    int max_specular_depth = 10);

ColorRGB estimate_direct_lighting_from_single_sample(const Thread_Context& thread_ctx,
    float u_light_selector, Vector2 u_light, Vector2 u_bsdf);
