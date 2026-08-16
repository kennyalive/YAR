import common;
import shared;
import resources;

import generate_ray;
import raytrace_types;
import evaluate_bsdf;

struct Push_Data
{
    Frame_Params frame;
    Path_Tracing_Params local;
}
[[vk::push_constant]]
Push_Data data;

static const float tmin = 1e-3f;
static const float tmax = 1e+3f;
static const float Pi = 3.14159265f;

static const int bounce_count = 3;

float3 sample_hemisphere_cosine(float2 u)
{
    float z = sqrt(1.f - u[0]);
    float r = sqrt(u[0]);
    float phi = 2.f * Pi * u[1];
    float x = r * cos(phi);
    float y = r * sin(phi);
    return float3(x, y, z);
}

[shader("raygeneration")]
void main()
{
    const uint pixel_index = get_current_pixel_index();
    uint rng_state = init_rng(pixel_index, data.frame.frame_index);

    const uint2 sample_pixel = DispatchRaysIndex().xy;
    const float2 sample_offset = rand_float2(rng_state);
    const float2 sample_pos = float2(sample_pixel) + sample_offset;

    float3 path_coeff = float3(1);

    Ray ray = generate_ray(data.frame.camera_to_world, sample_pos, data.frame.tan_fovy_over_2, data.frame.z_is_up);
    float3 L = float3(0);

    for (int i = 0; i < bounce_count; i++) {
        float3 wo = -ray.dir;

        RayDesc ray_desc;
        ray_desc.Origin = ray.origin;
        ray_desc.TMin = tmin;
        ray_desc.Direction = ray.dir;
        ray_desc.TMax = tmax;

        Path_Payload payload;
        TraceRay(accel, RAY_FLAG_FORCE_OPAQUE, 0xffu, 0, 0, 0, ray_desc, payload);

        if (!payload.hit) {
            break;
        }

        L += path_coeff * payload.color;

        float3 normal = payload.normal;
        float3 bitangent = normalize(payload.dpdu);
        float3 tangent = normalize(cross(bitangent, normal));

        float2 u = rand_float2(rng_state);
        float3 local_dir = sample_hemisphere_cosine(u);
        float3 wi = local_dir.x * tangent + local_dir.y * bitangent + local_dir.z * normal;

        ray.origin = ray.origin + ray.dir * payload.t + normal * 0.001f;
        ray.dir = wi;

        float diffuse_bsdf_pdf = dot(normal, wi) / Pi;

        float3 bsdf = evaluate_bsdf(payload.mtl_handle, payload.uv, wi, wo);
        path_coeff *= bsdf * abs(dot(payload.normal, wi)) / diffuse_bsdf_pdf;
    }

    float3 current_L = storage_images_rgba_f16[data.local.output_image_index][DispatchRaysIndex().xy].rgb;
    float k = float(data.local.accumulation_index + 1);
    float3 new_L = current_L * (k - 1) / k + L / k;
    storage_images_rgba_f16[data.local.output_image_index][DispatchRaysIndex().xy] = float4(new_L, 1);

    //params.output_image[DispatchRaysIndex().xy] = float4(L, 1);
}
