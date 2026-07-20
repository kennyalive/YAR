import shared;
import resources;

struct Push_Data
{
    Frame_Params frame;
    Apply_Tonemap_Params local;
}
[[vk::push_constant]]
Push_Data data;

float get_luminance(float3 color)
{
    return dot(color, float3(0.2126, 0.7152, 0.0722));
}

float tonemap_reinhard(float luminance)
{
    static const float white_point_luminance = 2.0;
    static const float k = 1.0 / (white_point_luminance * white_point_luminance);
    return luminance * (1.0 + k * luminance) / (1.0 + luminance);
}

[numthreads(32, 32, 1)]
void main(uint2 id: SV_DispatchThreadID)
{
    if (id.x < data.frame.viewport_size.x && id.y < data.frame.viewport_size.y) {
        Apply_Tonemap_Params params = data.local;
        float4 color = storage_images_rgba_f16[params.output_image_index][id];
        float luminance_hdr = get_luminance(color.rgb);
        float luminance_ldr = tonemap_reinhard(luminance_hdr);
        float scale = luminance_ldr / luminance_hdr;
        color.rgb *= scale;
        storage_images_rgba_f16[params.tonemap_index][id] = color;
    }
}
