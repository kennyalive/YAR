#pragma once

#include "lib/color.h"
#include "lib/vector.h"

struct Float_Parameter;
struct RGB_Parameter;
struct Scene_Context;
struct Thread_Context;

ColorRGB evaluate_rgb_parameter(const Scene_Context& scene_ctx, Vector2 uv, Vector2 duvdx, Vector2 duvdy, const RGB_Parameter& param);
ColorRGB evaluate_rgb_parameter(const Thread_Context& thread_ctx, const RGB_Parameter& param); // DEPRECATED

float evaluate_float_parameter(const Scene_Context& scene_ctx, Vector2 uv, Vector2 duvdx, Vector2 duvdy, const Float_Parameter& param);
float evaluate_float_parameter(const Thread_Context& thread_ctx, const Float_Parameter& param); // DEPRECATED
