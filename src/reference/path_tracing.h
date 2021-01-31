#pragma once

#include "lib/color.h"

struct Footprint_Tracking_Ray;
struct Scene_Context;
struct Thread_Context;

ColorRGB estimate_path_contribution(const Scene_Context& scene_ctx, Thread_Context& thread_ctx, const Footprint_Tracking_Ray& ray);
