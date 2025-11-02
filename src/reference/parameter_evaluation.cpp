#include "std.h"
#include "lib/common.h"
#include "parameter_evaluation.h"

#include "scene_context.h"
#include "thread_context.h"

#include "lib/material_parameter.h"

ColorRGB evaluate_rgb_parameter(const Scene_Context& scene_ctx, Vector2 uv, Vector2 duvdx, Vector2 duvdy, const RGB_Parameter& param)
{
    if (param.eval_mode == EvaluationMode::constant) {
        return param.constant_value;
    }

    ASSERT(param.texture_index >= 0);
    const Image_Texture& texture = scene_ctx.textures[param.texture_index];

    Vector2 uv_scale = Vector2(param.u_scale, param.v_scale);

    uv.x *= param.u_scale;
    uv.y *= param.v_scale;

    // A. nearest
    //return texture.sample_nearest(uv, 0, Wrap_Mode::repeat);

    // B. bilinear
    //return texture.sample_bilinear(uv, 0, Wrap_Mode::repeat);

    // C. trilinear
    //float lod = shading_ctx.compute_texture_lod(int(texture.get_mips().size()), uv_scale);
    //return texture.sample_trilinear(uv, lod, Wrap_Mode::repeat);

    // D. EWA
    duvdx *= uv_scale;
    duvdy *= uv_scale;
    return texture.sample_EWA(uv, duvdx, duvdy, Wrap_Mode::repeat, 32.f);
}

ColorRGB evaluate_rgb_parameter(const Scene_Context& scene_ctx, const Shading_Context& shading_ctx, const RGB_Parameter& param)
{
    Vector2 duvdx = Vector2(shading_ctx.dudx, shading_ctx.dvdx);
    Vector2 duvdy = Vector2(shading_ctx.dudy, shading_ctx.dvdy);
    return evaluate_rgb_parameter(scene_ctx, shading_ctx.uv, duvdx, duvdy, param);
}

// DEPRECATED
ColorRGB evaluate_rgb_parameter(const Thread_Context& thread_ctx, const RGB_Parameter& param)
{
    const Shading_Context& shading_ctx = thread_ctx.shading_context;
    Vector2 duvdx = Vector2(shading_ctx.dudx, shading_ctx.dvdx);
    Vector2 duvdy = Vector2(shading_ctx.dudy, shading_ctx.dvdy);
    return evaluate_rgb_parameter(thread_ctx.scene_context, shading_ctx.uv, duvdx, duvdy, param);
}

float evaluate_float_parameter(const Scene_Context& scene_ctx, Vector2 uv, Vector2 duvdx, Vector2 duvdy, const Float_Parameter& param)
{
    if (param.eval_mode == EvaluationMode::constant) {
        return param.constant_value;
    }

    ASSERT(param.texture_index >= 0);
    const Image_Texture& texture = scene_ctx.textures[param.texture_index];

    Vector2 uv_scale = Vector2(param.u_scale, param.v_scale);

    uv.x *= param.u_scale;
    uv.y *= param.v_scale;

    // A. nearest
    //return texture.sample_nearest(uv, 0, Wrap_Mode::repeat);

    // B. bilinear
    //return texture.sample_bilinear(uv, 0, Wrap_Mode::repeat);

    // C. trilinear
    //float lod = shading_ctx.compute_texture_lod(int(texture.get_mips().size()), uv_scale);
    //return texture.sample_trilinear(uv, lod, Wrap_Mode::repeat);

    // D. EWA
    duvdx *= uv_scale;
    duvdy *= uv_scale;
    ColorRGB rgb = texture.sample_EWA(uv, duvdx, duvdy, Wrap_Mode::repeat, 32.f);
    float luminance = rgb.luminance();
    return luminance;
}

float evaluate_float_parameter(const Scene_Context& scene_ctx, const Shading_Context& shading_ctx, const Float_Parameter& param)
{
    Vector2 duvdx = Vector2(shading_ctx.dudx, shading_ctx.dvdx);
    Vector2 duvdy = Vector2(shading_ctx.dudy, shading_ctx.dvdy);
    return evaluate_float_parameter(scene_ctx, shading_ctx.uv, duvdx, duvdy, param);
}

// DEPRECATED
float evaluate_float_parameter(const Thread_Context& thread_ctx, const Float_Parameter& param)
{
    const Shading_Context& shading_ctx = thread_ctx.shading_context;
    Vector2 duvdx = Vector2(shading_ctx.dudx, shading_ctx.dvdx);
    Vector2 duvdy = Vector2(shading_ctx.dudy, shading_ctx.dvdy);
    return evaluate_float_parameter(thread_ctx.scene_context, shading_ctx.uv, duvdx, duvdy, param);
}
