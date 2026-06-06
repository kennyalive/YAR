module direct_lighting;

import common;
import shared;
import resources;

import evaluate_bsdf;
import raytrace_types;
import shading_context;

bool trace_shadow_ray(RaytracingAccelerationStructure accel, float3 position, float3 direction, float distance)
{
    Shadow_Ray_Payload shadow_ray_payload;
    shadow_ray_payload.shadow_factor = 1.0f;

    RayDesc ray_desc;
    ray_desc.Origin = position;
    ray_desc.TMin = 0.0;
    ray_desc.Direction = direction;
    ray_desc.TMax = distance;

    TraceRay(accel, RAY_FLAG_FORCE_OPAQUE | RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH, 0xffu, 1, 0, 1, ray_desc, shadow_ray_payload);
    return shadow_ray_payload.shadow_factor != 0.0;
}

public float3 estimate_direct_lighting(Materials materials, Shading_Context sc, RaytracingAccelerationStructure accel, inout uint rng_state)
{
    Material_Handle mtl_handle = instance_infos[InstanceID()].material;
    float3 P = offset_ray(sc.P, sc.Ng);

    float3 L = float3(0, 0, 0);

    const Scene_Info scene_info = get_scene_info();
    const uint point_light_count = scene_info.point_light_count;
    const uint directional_light_count = scene_info.directional_light_count;
    const uint rect_light_count = scene_info.rect_light_count;

    // Point lights
    for (int i = 0; i < point_light_count; i++) {
        float3 light_vec = point_lights[i].position - P;
        float light_dist = length(light_vec);
        float3 light_dir = light_vec / light_dist;

        float n_dot_l = dot(sc.N, light_dir);
        if (n_dot_l <= 0.0)
            continue;

        if (!trace_shadow_ray(accel, P, light_dir, light_dist - Shadow_Epsilon))
            continue;

        float3 bsdf = evaluate_bsdf(materials, mtl_handle, sc.UV, light_dir, sc.Wo);
        float3 irradiance = point_lights[i].intensity * (n_dot_l / (light_dist * light_dist));
        L += irradiance * bsdf;
    }

    // Directional lights
    for (int i = 0; i < directional_light_count; i++) {
        float3 light_dir = directional_lights[i].direction;

        float n_dot_l = dot(sc.N, light_dir);
        if (n_dot_l <= 0.0)
            continue;
        
        if (!trace_shadow_ray(accel, P, light_dir, 1e5 /* compute tmax? */))
            continue;

        float3 bsdf = evaluate_bsdf(materials, mtl_handle, sc.UV, light_dir, sc.Wo);
        L += bsdf * directional_lights[i].irradiance * n_dot_l;
    }

    // Rectangle lights
    for (int i = 0; i < rect_light_count; i++) {
        Rect_Light light = rect_lights[i];
        if (light.shadow_ray_count == 0)
            continue;

        float3 L2 = float3(0, 0, 0);
        for (int k = 0; k < light.shadow_ray_count; k++) {
            float2 u = 2.0 * rand_float2(rng_state) - 1.0;

            float3 local_light_point = float3(light.size.x / 2.0 * u.x, light.size.y / 2.0 * u.y, 0.f);
            float3 light_point = mul(light.light_to_world_transform, float4(local_light_point, 1.0));

            float3 light_vec = light_point - P;
            float light_dist = length(light_vec);
            float3 light_dir = light_vec / light_dist;

            float3 light_normal = float3(
                light.light_to_world_transform[0][2],
                light.light_to_world_transform[1][2],
                light.light_to_world_transform[2][2]
            );
            float light_n_dot_l = dot(light_normal, -light_dir);
            if (light_n_dot_l <= 0.f)
                continue;

            float n_dot_l = dot(sc.N, light_dir);
            if (n_dot_l <= 0.f)
                continue;
            
            if (!trace_shadow_ray(accel, P, light_dir, light_dist - Shadow_Epsilon))
                continue;

            float3 bsdf = evaluate_bsdf(materials, mtl_handle, sc.UV, light_dir, sc.Wo);
            L2 += bsdf * light.area * light.emitted_radiance * (n_dot_l * light_n_dot_l / (light_dist * light_dist));
        }
        L2 /= float(light.shadow_ray_count);
        L += L2;
    }

    return L;
}
