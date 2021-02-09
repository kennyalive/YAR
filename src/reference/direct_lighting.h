#pragma once

#include "lib/color.h"
#include "lib/vector.h"

struct Auxilary_Rays;
struct Ray;
struct Scene_Context;
struct Shading_Context;
struct Thread_Context;

bool trace_ray(const Scene_Context& scene_ctx, Thread_Context& thread_ctx, const Ray& ray,
    const Auxilary_Rays* auxilary_rays, ColorRGB* specular_attenuation, int max_specular_bounces);

ColorRGB estimate_direct_lighting(const Scene_Context& scene_ctx, Thread_Context& thread_ctx,
    const Ray& ray, const Auxilary_Rays& auxilary_rays, int max_specular_depth = 10);

ColorRGB estimate_direct_lighting_from_single_sample(const Scene_Context& scene_ctx, const Shading_Context& shading_ctx,
    float u_light_selector, Vector2 u_light, Vector2 u_bsdf);
