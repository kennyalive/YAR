#include "std.h"
#include "lib/common.h"
#include "parameter_evaluation.h"

#include "context.h"
#include "shading_context.h"

#include "lib/parameter.h"

ColorRGB evaluate_rgb_parameter(const Thread_Context& thread_ctx, const RGB_Parameter& param) {
    if (param.is_constant)
        return param.constant_value;

    const Scene_Context& scene_ctx = *thread_ctx.scene_context;
    const Shading_Context& shading_ctx = thread_ctx.shading_context;

    ASSERT(param.texture_index >= 0);
    const Image_Texture& texture = scene_ctx.textures[param.texture_index];

    Vector2 uv_scale = Vector2(param.u_scale, param.v_scale);

    Vector2 uv = shading_ctx.uv;
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
    float dudx = shading_ctx.dudx * param.u_scale;
    float dvdx = shading_ctx.dvdx * param.v_scale;
    float dudy = shading_ctx.dudy * param.u_scale;
    float dvdy = shading_ctx.dvdy * param.v_scale;
    return texture.sample_EWA(uv, Vector2(dudx, dvdx), Vector2(dudy, dvdy), Wrap_Mode::repeat, 32.f);
}

float evaluate_float_parameter(const Thread_Context& thread_ctx, const Float_Parameter& param) {
    if (param.is_constant)
        return param.constant_value;

    // TODO: implement texture sampling
    ASSERT(false);
    return 0.f;
}
