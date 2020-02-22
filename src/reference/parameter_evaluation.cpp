#include "std.h"
#include "lib/common.h"
#include "parameter_evaluation.h"
#include "render_context.h"
#include "shading_context.h"

#include "lib/parameter.h"

ColorRGB evaluate_rgb_parameter(const Render_Context& global_ctx, const Shading_Context& shading_ctx, const RGB_Parameter& param) {
    if (param.is_constant)
        return param.constant_value;

    ASSERT(param.texture_index >= 0);
    const Texture& texture = global_ctx.textures[param.texture_index];

    Vector2 uv = shading_ctx.UV;
    uv.x *= param.u_scale;
    uv.y *= param.v_scale;

    int x = int(uv.x * texture.width) % texture.width;
    int y = int(uv.y * texture.height) % texture.height;

    int texel_index = y * texture.width + x;
    ColorRGB texel_value = texture.texels[texel_index];
    return texel_value;
}
