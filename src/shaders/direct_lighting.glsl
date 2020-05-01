#ifndef DIRECT_LIGHTING_GLSL
#define DIRECT_LIGHTING_GLSL

#include "compute_bsdf.glsl"
#include "light_resources.glsl"
#include "shading_context.glsl"

vec3 estimate_direct_lighting(Shading_Context sc, accelerationStructureNV accel, int point_light_count, int directional_light_count, int diffuse_rectangular_light_count)
{
    vec3 L = vec3(0);
    Material_Handle mtl_handle = instance_infos[gl_InstanceCustomIndexNV].material;

    const vec3 P = offset_ray(sc.P, sc.Ng);

    for (int i = 0; i < point_light_count; i++) {
        vec3 light_vec = point_lights[i].position - P;
        float light_dist = length(light_vec);
        vec3 light_dir = light_vec / light_dist;

        float n_dot_l = dot(sc.N, light_dir);
        if (n_dot_l <= 0.0)
            continue;

        // trace shadow ray
        shadow_ray_payload.shadow_factor = 1.0f;
        traceNV(accel, gl_RayFlagsOpaqueNV|gl_RayFlagsTerminateOnFirstHitNV, 0xff, 1, 0, 1, P, 0.0, light_dir, light_dist - Shadow_Epsilon, 1);
        if (shadow_ray_payload.shadow_factor == 0.0)
            continue;

        vec3 bsdf = compute_bsdf(mtl_handle, sc.UV, light_dir, sc.Wo);
        vec3 irradiance = point_lights[i].intensity * (n_dot_l / (light_dist * light_dist));
        L += shadow_ray_payload.shadow_factor * irradiance * bsdf;
    }

    for (int i = 0; i < directional_light_count; i++) {
        vec3 light_dir = directional_lights[i].direction;

        float n_dot_l = dot(sc.N, light_dir);
        if (n_dot_l <= 0.0)
            continue;

        // trace shadow ray
        shadow_ray_payload.shadow_factor = 1.0f;
        traceNV(accel, gl_RayFlagsOpaqueNV|gl_RayFlagsTerminateOnFirstHitNV, 0xff, 1, 0, 1, P, 0.0, light_dir, 1e5 /* compute tmax? */, 1);
        if (shadow_ray_payload.shadow_factor == 0.0)
            continue;

        vec3 bsdf = compute_bsdf(mtl_handle, sc.UV, light_dir, sc.Wo);
        L += bsdf * directional_lights[i].irradiance * n_dot_l;
    }

    uint seed = uint(gl_LaunchIDNV.y)*uint(800) + uint(gl_LaunchIDNV.x);
    uint rng_state = wang_hash(seed);

    for (int i = 0; i < diffuse_rectangular_light_count; i++) {
        Diffuse_Rectangular_Light light = diffuse_rectangular_lights[i];
        if (light.shadow_ray_count == 0)
            continue;
        vec3 L2 = vec3(0);
        for (int k = 0; k < light.shadow_ray_count; k++) {
            vec2 u;
            u.x = float(rng_state) * (1.0/float(0xffffffffu));
            rng_state = rand_xorshift(rng_state);
            u.y = float(rng_state) * (1.0/float(0xffffffffu));
            u = 2.0*u - 1.0;

            vec3 local_light_point = vec3(light.size.x/2.0 * u.x, light.size.y/2.0 * u.y, 0.f);
            vec3 light_point = light.light_to_world_transform * vec4(local_light_point, 1.0);

            vec3 light_vec = light_point - P;
            float light_dist = length(light_vec);
            vec3 light_dir = light_vec / light_dist;

            vec3 light_normal = light.light_to_world_transform[2];
            float light_n_dot_l = dot(light_normal, -light_dir);
            if (light_n_dot_l <= 0.f)
                continue;

            float n_dot_l = dot(sc.N, light_dir);
            if (n_dot_l <= 0.f)
                continue;

            // trace shadow ray
            shadow_ray_payload.shadow_factor = 1.0f;
            traceNV(accel, gl_RayFlagsOpaqueNV|gl_RayFlagsTerminateOnFirstHitNV, 0xff, 1, 0, 1, P, 0.0, light_dir, light_dist - Shadow_Epsilon, 1);
            if (shadow_ray_payload.shadow_factor == 0.0)
                continue;

            vec3 bsdf = compute_bsdf(mtl_handle, sc.UV, light_dir, sc.Wo);
            L2 += shadow_ray_payload.shadow_factor * bsdf * light.area * light.emitted_radiance * (n_dot_l * light_n_dot_l / (light_dist * light_dist));
        }
        L2 /= float(light.shadow_ray_count);
        L += L2;
    }
    return L;
}

#endif // DIRECT_LIGHTING_GLSL
