#pragma once

#include "pixel_sampling.h"
#include "shading_context.h"

#include "lib/utils.h" // Memory_Pool

struct Path_Context {
    int bounce_count = 0; // current number of bounces
    int perfect_specular_bounce_count = 0;
};

struct Thread_Context {
    Memory_Pool memory_pool;
    RNG rng;
    Stratified_Pixel_Sampler pixel_sampler;

    const Scene_Context* scene_context = nullptr;
    Path_Context path_context;
    Shading_Context shading_context;

    // TODO: until we implement proper handling of nested dielectrics we make assumption
    // that we don't have nested dielectrics and after we start tracing inside dielectric
    // the only possible hit can be with the same dielectric material for exit event. Here
    // we track current dielectric material to assert this convention and also to determine
    // if it's enter or exit event.
    Material_Handle current_dielectric_material;
};
