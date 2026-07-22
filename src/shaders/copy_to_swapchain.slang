import common;
import shared;
import resources;

struct Push_Data
{
    Frame_Params frame;
    Copy_To_Swapchain_Params local;
}
[[vk::push_constant]]
Push_Data data;

[numthreads(32, 32, 1)]
void main(uint2 id : SV_DispatchThreadID)
{
    if (id.x < data.frame.viewport_size.x && id.y < data.frame.viewport_size.y) {
        uint image_index = data.local.swapchain_first_image_index + data.frame.swapchain_image_index;
        storage_images_rgba8[image_index][id] = float4(srgb_encode(storage_images_rgba_f16[data.local.tonemap_index][id].rgb), 1.0);
    }
}
