#include "std.h"
#include "lib/common.h"
#include "parameter_evaluation.h"

#include "context.h"
#include "shading_context.h"

#include "lib/parameter.h"

ColorRGB evaluate_rgb_parameter(const Scene_Context& global_ctx, const Shading_Context& shading_ctx, const RGB_Parameter& param) {
    if (param.is_constant)
        return param.constant_value;

    ASSERT(param.texture_index >= 0);
    const Image_Texture& texture = global_ctx.textures[param.texture_index];

    Vector2 uv_scale = Vector2(param.u_scale, param.v_scale);

    float lod = shading_ctx.compute_texture_lod(int(texture.get_mips().size()), uv_scale);

    Vector2 uv = shading_ctx.UV;
    uv.x *= param.u_scale;
    uv.y *= param.v_scale;
    //return texture.sample_nearest(uv, 0, Wrap_Mode::repeat);
    //return texture.sample_bilinear(uv, 0, Wrap_Mode::repeat);
    //return texture.sample_trilinear(uv, lod, Wrap_Mode::repeat);
    return texture.sample_EWA(uv, shading_ctx.dUVdx * uv_scale, shading_ctx.dUVdy * uv_scale, Wrap_Mode::repeat, 32.f);
}

float evaluate_float_parameter(const  Scene_Context& global_ctx, const Shading_Context& shading_ctx, const Float_Parameter& param) {
    if (param.is_constant)
        return param.constant_value;

    // TODO: implement texture sampling
    ASSERT(false);
    return 0.f;
}
