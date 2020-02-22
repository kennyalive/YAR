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

    int x = std::clamp(int(shading_ctx.UV.x * texture.width), 0, texture.width - 1);
    int y = std::clamp(int(shading_ctx.UV.y * texture.height), 0, texture.height - 1);

    int texel_index = y * texture.width + x;
    ColorRGB texel_value = texture.texels[texel_index];
    return texel_value;
}
