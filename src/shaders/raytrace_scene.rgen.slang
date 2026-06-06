import shared;
import generate_ray;
import raytrace_types;

[[vk::push_constant]]
Frame_Params frame_params;

struct KernelParams {
    [[vk::image_format("rgba16f")]]
    RWTexture2D<float4> output_image;

    RaytracingAccelerationStructure accel;
}

[[vk::binding(0, KERNEL_SET_0)]]
ParameterBlock<KernelParams> params;

static const float tmin = 1e-3f;
static const float tmax = 1e+3f;

float3 trace_ray(float2 sample_pos, Frame_Params frame_params)
{
    Ray ray = generate_ray(frame_params.camera_to_world, sample_pos, frame_params.tan_fovy_over_2, frame_params.z_is_up);

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
    if (frame_params.spp4 != 0) {
        color = trace_ray(sample_origin + float2(0.125, 0.375), frame_params);
        color += trace_ray(sample_origin + float2(0.375, 0.875), frame_params);
        color += trace_ray(sample_origin + float2(0.625, 0.125), frame_params);
        color += trace_ray(sample_origin + float2(0.875, 0.625), frame_params);
        color *= 0.25;
    }
    else {
        color = trace_ray(sample_origin + float2(0.5, 0.5), frame_params);
    }
    params.output_image[DispatchRaysIndex().xy] = float4(color, 1);
}
