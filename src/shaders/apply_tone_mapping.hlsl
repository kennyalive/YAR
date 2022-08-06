struct Constants {
	uint2 viewport_size;
};
[[vk::push_constant]]
Constants constants;

[[vk::image_format("rgba16f")]]
[[vk::binding(0, 0)]]
RWTexture2D<float4> output_image;

float get_luminance(float3 color) {
    return dot(color, float3(0.2126, 0.7152, 0.0722));
}

float tonemap_reinhard(float luminance) {
    static const float white_point_luminance = 2.0;
    static const float k = 1.0 / (white_point_luminance * white_point_luminance);
    return luminance * (1.0 + k * luminance) / (1.0 + luminance);
}

[numthreads(32, 32, 1)]
void main(uint2 id : SV_DispatchThreadID) {
    if (id.x < constants.viewport_size.x && id.y < constants.viewport_size.y) {
        float4 color = output_image[id];
        float luminance_hdr = get_luminance(color.rgb);
        float luminance_ldr = tonemap_reinhard(luminance_hdr);
        float scale = luminance_ldr / luminance_hdr;
        color.rgb *= scale;
        output_image[id] = color;
    }
}
