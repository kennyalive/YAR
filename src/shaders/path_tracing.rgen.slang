import shared_main;
import generate_ray;
import raytrace_types;

struct PushConstants {
    uint spp4;
    float tan_fovy_over_2;
    uint z_is_up;
};
[[vk::push_constant]]
PushConstants push_constants;

struct KernelParams {
    [[vk::image_format("rgba16f")]]
    RWTexture2D<float4> output_image;

    RaytracingAccelerationStructure accel;

    cbuffer Constants
    {
        float3x4 camera_to_world;
    };
}

[[vk::binding(0, KERNEL_SET_0)]]
ParameterBlock<KernelParams> params;

static const float tmin = 1e-3f;
static const float tmax = 1e+3f;

float3 trace_ray(float2 sample_pos, PushConstants push_constants)
{
    Ray ray = generate_ray(params.camera_to_world, sample_pos, push_constants.tan_fovy_over_2, push_constants.z_is_up);
    
    RayDesc ray_desc;
    ray_desc.Origin = ray.origin;
    ray_desc.TMin = tmin;
    ray_desc.Direction = ray.dir;
    ray_desc.TMax = tmax;

    Ray_Payload payload;
    TraceRay(params.accel, RAY_FLAG_FORCE_OPAQUE, 0xffu, 0, 0, 0, ray_desc, payload);
    return payload.color;
}

[shader("raygeneration")]
void main()
{
    float2 sample_origin = float2(DispatchRaysIndex().xy);
    
    float3 color;
    if (push_constants.spp4 != 0) {
        color = trace_ray(sample_origin + float2(0.125, 0.375), push_constants);
        color += trace_ray(sample_origin + float2(0.375, 0.875), push_constants);
        color += trace_ray(sample_origin + float2(0.625, 0.125), push_constants);
        color += trace_ray(sample_origin + float2(0.875, 0.625), push_constants);
        color *= 0.25;
    }
    else {
        color = trace_ray(sample_origin + float2(0.5, 0.5), push_constants);
    }
    params.output_image[DispatchRaysIndex().xy] = float4(color, 1);
}
