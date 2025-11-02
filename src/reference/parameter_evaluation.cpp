#include "std.h"
#include "lib/common.h"
#include "parameter_evaluation.h"

#include "scene_context.h"
#include "thread_context.h"

#include "lib/material_parameter.h"

static ColorRGB evaluate_texture_parameter(const Scene_Context& scene_ctx, const TextureParameter& texture_parameter,
    Vector2 uv, Vector2 duvdx, Vector2 duvdy) {
    ASSERT(texture_parameter.texture_index >= 0);
    const Image_Texture& texture = scene_ctx.textures[texture_parameter.texture_index];
    const Vector2 uv_scale(texture_parameter.u_scale, texture_parameter.v_scale);
    uv.x *= texture_parameter.u_scale;
    uv.y *= texture_parameter.v_scale;

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

static ColorRGB evaluate_leaf_parameter_rgb(const Scene_Context& scene_ctx, const LeafParameter& leaf,
    Vector2 uv, Vector2 duvdx, Vector2 duvdy) {
    if (leaf.is_constant) {
        return leaf.constant;
    }
    else {
        return evaluate_texture_parameter(scene_ctx, leaf.texture, uv, duvdx, duvdy);
    }
}

static float evaluate_leaf_parameter_float(const Scene_Context& scene_ctx, const LeafParameter& leaf,
    Vector2 uv, Vector2 duvdx, Vector2 duvdy) {
    if (leaf.is_constant) {
        return leaf.constant.r;
    }
    else {
        ColorRGB color = evaluate_texture_parameter(scene_ctx, leaf.texture, uv, duvdx, duvdy);
        float luminance = color.luminance();
        return luminance;
    }
}

ColorRGB evaluate_rgb_parameter(const Scene_Context& scene_ctx, Vector2 uv, Vector2 duvdx, Vector2 duvdy, const RGB_Parameter& param)
{
    if (param.eval_mode == EvaluationMode::value) {
        return evaluate_leaf_parameter_rgb(scene_ctx, param.value, uv, duvdx, duvdy);
    }
    else if (param.eval_mode == EvaluationMode::scale) {
        const RGB_Parameter& param0 = static_cast<const RGB_Parameter&>(scene_ctx.material_parameters[param.parameter0_index]);
        const RGB_Parameter& param1 = static_cast<const RGB_Parameter&>(scene_ctx.material_parameters[param.parameter1_index]);
        const ColorRGB value0 = evaluate_rgb_parameter(scene_ctx, uv, duvdx, duvdy, param0);
        const ColorRGB value1 = evaluate_rgb_parameter(scene_ctx, uv, duvdx, duvdy, param1);
        const ColorRGB value = value0 * value1;
        return value;
    }
    ASSERT(false);
    return Color_Black;
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
    if (param.eval_mode == EvaluationMode::value) {
        return evaluate_leaf_parameter_float(scene_ctx, param.value, uv, duvdx, duvdy);
    }
    else if (param.eval_mode == EvaluationMode::scale) {
        const Float_Parameter& param0 = static_cast<const Float_Parameter&>(scene_ctx.material_parameters[param.parameter0_index]);
        const Float_Parameter& param1 = static_cast<const Float_Parameter&>(scene_ctx.material_parameters[param.parameter1_index]);
        const float value0 = evaluate_float_parameter(scene_ctx, uv, duvdx, duvdy, param0);
        const float value1 = evaluate_float_parameter(scene_ctx, uv, duvdx, duvdy, param1);
        const float value = value0 * value1;
        return value;
    }
    ASSERT(false);
    return 0.f;
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
