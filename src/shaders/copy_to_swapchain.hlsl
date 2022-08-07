struct Constants {
    uint2 viewport_size;
};
[[vk::push_constant]]
Constants constants;

[[vk::binding(0, 0)]]
Texture2D output_image;

[[vk::image_format("rgba8")]]
[[vk::binding(1, 0)]]
RWTexture2D<float4> swapchain_image;

[numthreads(32, 32, 1)]
void main(uint2 id : SV_DispatchThreadID) 
{
    if (id.x < constants.viewport_size.x && id.y < constants.viewport_size.y)
        swapchain_image[id] = output_image[id];
}
