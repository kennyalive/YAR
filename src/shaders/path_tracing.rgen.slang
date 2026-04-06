import common;
import shared_main;
import generate_ray;
import raytrace_types;
import evaluate_bsdf;
import shared_material;
import material_resources;

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

[[vk::binding(0, 1)]]
ParameterBlock<Materials> materials;

[[vk::binding(0, KERNEL_SET_0)]]
ParameterBlock<KernelParams> params;

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
    const float2 sample_origin = float2(DispatchRaysIndex().xy);
    const float2 sample_pos = sample_origin + float2(0.5, 0.5);

    float3 path_coeff = float3(1);

    Ray ray = generate_ray(params.camera_to_world, sample_pos, push_constants.tan_fovy_over_2, push_constants.z_is_up);
    float3 L = float3(0);

    uint seed = DispatchRaysIndex().y * 800 + DispatchRaysIndex().x;
    uint rng_state = wang_hash(seed);

    for (int i = 0; i < bounce_count; i++) {
        float3 wo = -ray.dir;

        RayDesc ray_desc;
        ray_desc.Origin = ray.origin;
        ray_desc.TMin = tmin;
        ray_desc.Direction = ray.dir;
        ray_desc.TMax = tmax;

        Path_Payload payload;
        TraceRay(params.accel, RAY_FLAG_FORCE_OPAQUE, 0xffu, 0, 0, 0, ray_desc, payload);

        if (!payload.hit) {
            break;
        }

        L += path_coeff * payload.color;

        float3 normal = payload.normal;
        float3 bitangent = normalize(payload.dpdu);
        float3 tangent = normalize(cross(bitangent, normal));

        float2 u;
        u.x = float(rng_state) * (1.0 / float(0xffffffffu));
        rng_state = rand_xorshift(rng_state);
        u.y = float(rng_state) * (1.0 / float(0xffffffffu));
        rng_state = rand_xorshift(rng_state);

        float3 local_dir = sample_hemisphere_cosine(u);
        float3 wi = local_dir.x * tangent + local_dir.y * bitangent + local_dir.z * normal;

        ray.origin = ray.origin + ray.dir * payload.t + normal * 0.001f;
        ray.dir = wi;

        float diffuse_bsdf_pdf = dot(normal, wi) / Pi;

        float3 bsdf = evaluate_bsdf(materials, payload.mtl_handle, payload.uv, wi, wo);
        path_coeff *= bsdf * abs(dot(payload.normal, wi)) / diffuse_bsdf_pdf;
    }
    params.output_image[DispatchRaysIndex().xy] = float4(L, 1);
}
