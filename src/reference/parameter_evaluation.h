#pragma once

#include "lib/color.h"

struct RGB_Parameter;
struct Float_Parameter;
struct Scene_Context;
struct Shading_Context;

ColorRGB evaluate_rgb_parameter(const Scene_Context& global_ctx, const Shading_Context& shading_ctx, const RGB_Parameter& param);
float evaluate_float_parameter(const  Scene_Context& global_ctx, const Shading_Context& shading_ctx, const Float_Parameter& param);