#include "shared_main.hlsli"

struct PushConstants {
    uint spp4;
    float tan_fovy_over_2;
    uint z_is_up;
};
[[vk::push_constant]]
PushConstants push_constants;

#define RGEN_SHADER
#include "rt_utils.hlsli"

[[vk::image_format("rgba16f")]]
[[vk::binding(0, KERNEL_SET_0)]]
RWTexture2D<float4> output_image;

[[vk::binding(1, KERNEL_SET_0)]]
RaytracingAccelerationStructure accel;

[[vk::binding(2, KERNEL_SET_0)]]
cbuffer Constants
{
    float3x4 camera_to_world;
};

static const float tmin = 1e-3f;
static const float tmax = 1e+3f;

float3 trace_ray(float2 sample_pos)
{
    Ray_Payload payload;
    Ray ray = generate_ray(camera_to_world, sample_pos);
    payload.rx_dir = ray.rx_dir;
    payload.ry_dir = ray.ry_dir;
    
    RayDesc ray_desc;
    ray_desc.Origin = ray.origin;
    ray_desc.TMin = tmin;
    ray_desc.Direction = ray.dir;
    ray_desc.TMax = tmax;
    
    TraceRay(accel, RAY_FLAG_FORCE_OPAQUE, 0xffu, 0, 0, 0, ray_desc, payload);
    return payload.color;
}

[shader("raygeneration")]
void main()
{
    float2 sample_origin = float2(DispatchRaysIndex().xy);
    
    float3 color;
    if (push_constants.spp4 != 0) {
        color = trace_ray(sample_origin + float2(0.125, 0.375));
        color += trace_ray(sample_origin + float2(0.375, 0.875));
        color += trace_ray(sample_origin + float2(0.625, 0.125));
        color += trace_ray(sample_origin + float2(0.875, 0.625));
        color *= 0.25;
    }
    else {
        color = trace_ray(sample_origin + float2(0.5, 0.5));
    }
    output_image[DispatchRaysIndex().xy] = float4(color, 1);
}
