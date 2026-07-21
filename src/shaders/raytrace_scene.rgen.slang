import shared;
import resources;
import generate_ray;
import raytrace_types;

struct Push_Data
{
    Frame_Params frame;
    Direct_Lighting_Params local;
}
[[vk::push_constant]]
Push_Data data;

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
    TraceRay(accel, RAY_FLAG_FORCE_OPAQUE, 0xffu, 0, 0, 0, ray_desc, payload);
    return payload.color;
}

[shader("raygeneration")]
void main()
{
    float2 sample_origin = float2(DispatchRaysIndex().xy);

    float3 color;
    if (data.frame.spp4 != 0) {
        color = trace_ray(sample_origin + float2(0.125, 0.375), data.frame);
        color += trace_ray(sample_origin + float2(0.375, 0.875), data.frame);
        color += trace_ray(sample_origin + float2(0.625, 0.125), data.frame);
        color += trace_ray(sample_origin + float2(0.875, 0.625), data.frame);
        color *= 0.25;
    }
    else {
        color = trace_ray(sample_origin + float2(0.5, 0.5), data.frame);
    }
    storage_images_rgba_f16[data.local.output_image_index][DispatchRaysIndex().xy] = float4(color, 1);
}
