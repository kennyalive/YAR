import common;
import shared;

[[vk::push_constant]]
Frame_Params frame_params;

[[vk::image_format("rgba16f")]]
[[vk::binding(0, 0)]]
RWTexture2D<float4> tonemapped_image;

[[vk::image_format("rgba8")]]
[[vk::binding(1, 0)]]
RWTexture2D<float4> swapchain_image[];

[numthreads(32, 32, 1)]
void main(uint2 id : SV_DispatchThreadID)
{
    if (id.x < frame_params.viewport_size.x && id.y < frame_params.viewport_size.y)
        swapchain_image[frame_params.swapchain_image_index][id] = float4(srgb_encode(tonemapped_image[id].rgb), 1.0);
}
