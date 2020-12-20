#include "std.h"
#include "lib/common.h"
#include "direct_lighting.h"

#include "bsdf.h"
#include "context.h"
#include "light_sampling.h"
#include "sampling.h"
#include "shading_context.h"

#include "lib/light.h"
#include "lib/math.h"

inline float mis_power_heuristic(float pdf1, float pdf2) {
    if (pdf1 == Infinity) {
        ASSERT(pdf2 != Infinity);
        return 1.f;
    }
    return pdf1*pdf1 / (pdf1*pdf1 + pdf2*pdf2);
}

static ColorRGB direct_lighting_from_point_light(const Scene_Context& scene_ctx, const Shading_Context& shading_ctx, const Point_Light& light) {
    const Vector3 light_vec = (light.position - shading_ctx.P);
    const float light_dist = light_vec.length();
    const Vector3 light_dir = light_vec / light_dist;

    float n_dot_l = dot(shading_ctx.N, light_dir);
    if (n_dot_l <= 0.f)
        return Color_Black;

    Ray light_visibility_ray(shading_ctx.P, light_dir);
    bool occluded = scene_ctx.acceleration_structure->intersect_any(light_visibility_ray, light_dist - 1e-4f);
    if (occluded)
        return Color_Black;

    ColorRGB bsdf = shading_ctx.bsdf->evaluate(shading_ctx.Wo, light_dir);
    ColorRGB L = (light.intensity * bsdf)  * (n_dot_l / (light_dist * light_dist));
    return L;
}

static ColorRGB direct_lighting_from_directional_light(const Scene_Context& scene_ctx, const Shading_Context& shading_ctx, const Directional_Light& light) {
    float n_dot_l = dot(shading_ctx.N, light.direction);
    if (n_dot_l <= 0.f)
        return Color_Black;

    Ray light_visibility_ray(shading_ctx.P, light.direction);
    bool occluded = scene_ctx.acceleration_structure->intersect_any(light_visibility_ray, Infinity);
    if (occluded)
        return Color_Black;

    ColorRGB bsdf = shading_ctx.bsdf->evaluate(shading_ctx.Wo, light.direction);
    ColorRGB L = (light.irradiance * bsdf) * n_dot_l;
    return L;
}

static ColorRGB direct_lighting_from_rectangular_light(
    const Scene_Context& scene_ctx, const Shading_Context& shading_ctx,
    Light_Handle light_handle, const Diffuse_Rectangular_Light& light,
    Vector2 u_light, Vector2 u_bsdf)
{
    ASSERT(light_handle.type == Light_Type::diffuse_rectangular);

    const Vector3 light_n = light.light_to_world_transform.get_column(2);
    const Vector3 light_center = light.light_to_world_transform.get_column(3);

    // rectangular light emits light only from the front side
    if (dot(light_n, (shading_ctx.P - light_center).normalized()) <= 0.f)
        return Color_Black;

    ColorRGB L;
    // Light sampling part of MIS.
    {
        Vector3 local_light_point = Vector3{ light.size * (u_light - Vector2(0.5f)), 0.0f };
        Vector3 light_point = transform_point(light.light_to_world_transform, local_light_point);
        light_point = offset_ray_origin(light_point, light_n);

        const Vector3 light_vec = (light_point - shading_ctx.P);
        float distance_to_sample = light_vec.length();
        Vector3 wi = light_vec / distance_to_sample;

        float n_dot_wi = dot(shading_ctx.N, wi);
        bool scattering_possible = n_dot_wi > 0.f && shading_ctx.bsdf->reflection_scattering ||
                                   n_dot_wi < 0.f && shading_ctx.bsdf->transmission_scattering;

        if (scattering_possible) {
            ColorRGB f = shading_ctx.bsdf->evaluate(shading_ctx.Wo, wi);

            if (!f.is_black()) {
                Ray light_visibility_ray(shading_ctx.P, wi);
                bool occluded = scene_ctx.acceleration_structure->intersect_any(light_visibility_ray, distance_to_sample);

                if (!occluded) {
                    // We already checked that the light is oriented towards the surface
                    // but for specific point we still can get different result due fp finite precision.
                    ASSERT(dot(light_n, -wi) > -1e-4f);
                    float light_n_dot_wi = std::max(0.f, dot(light_n, -wi));

                    float light_pdf = (distance_to_sample * distance_to_sample) / (light.size.x * light.size.y * light_n_dot_wi);
                    float bsdf_pdf = shading_ctx.bsdf->pdf(shading_ctx.Wo, wi);
                    float mis_weight = mis_power_heuristic(light_pdf, bsdf_pdf);

                    L += (light.emitted_radiance * f) * (mis_weight * std::abs(n_dot_wi) / light_pdf);
                }
            }
        }
    }
    // BSDF sampling part of MIS.
    {
        Vector3 wi;
        float bsdf_pdf;
        ColorRGB f = shading_ctx.bsdf->sample(u_bsdf, shading_ctx.Wo, &wi, &bsdf_pdf);

        if (!f.is_black()) {
            ASSERT(bsdf_pdf > 0.f);

            Intersection isect;
            Ray light_visibility_ray(shading_ctx.P, wi);
            bool found_isect = scene_ctx.acceleration_structure->intersect(light_visibility_ray, isect);

            if (found_isect && isect.scene_object->area_light == light_handle) {
                ASSERT(isect.geometry_type == Geometry_Type::triangle_mesh);
                const Triangle_Intersection& ti = isect.triangle_intersection;
                Vector3 p = ti.mesh->get_position(ti.triangle_index, ti.b1, ti.b2);
                float d = (p - shading_ctx.P).length();

                // We already checked that the light is oriented towards the surface
                // but for specific point we still can get different result due fp finite precision.
                ASSERT(dot(light_n, -wi) > -1e-4f);
                float light_n_dot_wi = std::max(0.f, dot(light_n, -wi));

                float light_pdf = (d * d) / (light.size.x * light.size.y * light_n_dot_wi);
                float mis_weight = mis_power_heuristic(bsdf_pdf, light_pdf);
                float n_dot_wi = dot(shading_ctx.N, wi);

                L += (light.emitted_radiance * f) * (mis_weight * std::abs(n_dot_wi) / bsdf_pdf);
            }
        }
    }
    return L;
}

static ColorRGB direct_lighting_from_sphere_light(
    const Scene_Context& scene_ctx, const Shading_Context& shading_ctx,
    Light_Handle light_handle, const Diffuse_Sphere_Light_Sampler& light_sampler,
    Vector2 u_light, Vector2 u_bsdf)
{
    ASSERT(light_handle.type == Light_Type::diffuse_sphere);

    ColorRGB L;
    // Light sampling part of MIS.
    {
        Vector3 wi;
        float distance_to_sample = light_sampler.sample(u_light, &wi);

        float n_dot_wi = dot(shading_ctx.N, wi);
        bool scattering_possible = n_dot_wi > 0.f && shading_ctx.bsdf->reflection_scattering ||
                                   n_dot_wi < 0.f && shading_ctx.bsdf->transmission_scattering;

        if (scattering_possible) {
            ColorRGB f = shading_ctx.bsdf->evaluate(shading_ctx.Wo, wi);

            if (!f.is_black()) {
                Ray light_visibility_ray(shading_ctx.P, wi);
                bool occluded = scene_ctx.acceleration_structure->intersect_any(light_visibility_ray, distance_to_sample);

                if (!occluded) {
                    float light_pdf = light_sampler.cone_sampling_pdf;
                    float bsdf_pdf = shading_ctx.bsdf->pdf(shading_ctx.Wo, wi);
                    float mis_weight = mis_power_heuristic(light_pdf, bsdf_pdf);

                    L += (light_sampler.light.emitted_radiance * f) * (mis_weight * std::abs(n_dot_wi) / light_pdf);
                }
            }
        }
    }
    // BSDF sampling part of MIS.
    {
        Vector3 wi;
        float bsdf_pdf;
        ColorRGB f = shading_ctx.bsdf->sample(u_bsdf, shading_ctx.Wo, &wi, &bsdf_pdf);

        if (!f.is_black()) {
            ASSERT(bsdf_pdf > 0.f);

            if (light_sampler.is_direction_inside_light_cone(wi)) {
                Intersection isect;
                Ray light_visibility_ray(shading_ctx.P, wi);
                bool found_isect = scene_ctx.acceleration_structure->intersect(light_visibility_ray, isect);

                if (found_isect && isect.scene_object->area_light == light_handle) {
                    float light_pdf = light_sampler.cone_sampling_pdf;
                    float mis_weight = mis_power_heuristic(bsdf_pdf, light_pdf);

                    L += (light_sampler.light.emitted_radiance * f) * (mis_weight * std::abs(dot(shading_ctx.N, wi)) / bsdf_pdf);
                }
            }
        }
    }
    return L;
}

static ColorRGB direct_lighting_from_environment_light(
    const Scene_Context& scene_ctx, const Shading_Context& shading_ctx,
    Vector2 u_light, Vector2 u_bsdf)
{
    ColorRGB L;
    // Light sampling part of MIS.
    {
        Light_Sample light_sample = scene_ctx.environment_light_sampler.sample(u_light);

        float n_dot_wi = dot(shading_ctx.N, light_sample.Wi);
        bool scattering_possible = n_dot_wi > 0.f && shading_ctx.bsdf->reflection_scattering ||
                                   n_dot_wi < 0.f && shading_ctx.bsdf->transmission_scattering;

        if (scattering_possible) {
            ColorRGB f = shading_ctx.bsdf->evaluate(shading_ctx.Wo, light_sample.Wi);

            if (!f.is_black()) {
                Ray light_visibility_ray(shading_ctx.P, light_sample.Wi);
                bool occluded = scene_ctx.acceleration_structure->intersect_any(light_visibility_ray, Infinity);

                if (!occluded) {
                    float bsdf_pdf = shading_ctx.bsdf->pdf(shading_ctx.Wo, light_sample.Wi);
                    float mis_weight = mis_power_heuristic(light_sample.pdf, bsdf_pdf);

                    L += (light_sample.Le * f) * (mis_weight * std::abs(n_dot_wi) / light_sample.pdf);
                }
            }
        }
    }
    // BSDF sampling part of MIS.
    {
        Vector3 wi;
        float bsdf_pdf;
        ColorRGB f = shading_ctx.bsdf->sample(u_bsdf, shading_ctx.Wo, &wi, &bsdf_pdf);

        if (!f.is_black()) {
            ASSERT(bsdf_pdf > 0.f);
            ColorRGB Le = scene_ctx.environment_light_sampler.get_radiance_for_direction(wi);

            if (!Le.is_black()) {
                Ray light_visibility_ray(shading_ctx.P, wi);
                bool occluded = scene_ctx.acceleration_structure->intersect_any(light_visibility_ray, Infinity);

                if (!occluded) {
                    float light_pdf = scene_ctx.environment_light_sampler.pdf(wi);
                    float mis_weight = mis_power_heuristic(bsdf_pdf, light_pdf);

                    L += (Le * f) * (mis_weight * std::abs(dot(shading_ctx.N, wi)) / bsdf_pdf);
                }
            }
        }
    }
    return L;
}

static ColorRGB reflect_from_mirror_surface(const Scene_Context& ctx, Thread_Context& thread_ctx, const Shading_Context& shading_ctx, int max_specular_depth) {
    Ray reflected_ray;
    reflected_ray.origin = shading_ctx.P;
    reflected_ray.direction = shading_ctx.N * (2.f * dot(shading_ctx.N, shading_ctx.Wo)) - shading_ctx.Wo;

    Intersection isect;
    if (ctx.acceleration_structure->intersect(reflected_ray, isect)) {
        Shading_Point_Rays rays;
        rays.incident_ray = reflected_ray;
        // TODO: generate auxilary rays for mirror?
        rays.auxilary_ray_dx_offset = reflected_ray;
        rays.auxilary_ray_dy_offset = reflected_ray;

        Shading_Context shading_ctx2(ctx, thread_ctx, rays, isect);
        ColorRGB reflected_radiance = estimate_direct_lighting(ctx, thread_ctx, shading_ctx2, max_specular_depth);
        return reflected_radiance * shading_ctx.mirror_reflectance;
    }
    else if (ctx.has_environment_light_sampler) {
        return ctx.environment_light_sampler.get_radiance_for_direction(reflected_ray.direction);
    }
    else {
        return Color_Black;
    }
}

ColorRGB estimate_direct_lighting(const Scene_Context& scene_ctx, Thread_Context& thread_ctx, const Shading_Context& shading_ctx, int max_specular_depth)
{
    ColorRGB L;

    // Intersection with specular surface.
    if (shading_ctx.mirror_surface) {
        if (max_specular_depth > 0) {
            L = reflect_from_mirror_surface(scene_ctx, thread_ctx, shading_ctx, --max_specular_depth);
        }
    }
    // Intersection with area light.
    else if (shading_ctx.area_light != Null_Light) {
        if (shading_ctx.area_light.type == Light_Type::diffuse_rectangular) {
            L = scene_ctx.lights.diffuse_rectangular_lights[shading_ctx.area_light.index].emitted_radiance;
        }
        else {
            ASSERT(shading_ctx.area_light.type == Light_Type::diffuse_sphere);
            L = scene_ctx.lights.diffuse_sphere_lights[shading_ctx.area_light.index].emitted_radiance;
        }
    }
    // Intersection with finite BSDF surface.
    else {
        for (const Point_Light& light : scene_ctx.lights.point_lights) {
            L += direct_lighting_from_point_light(scene_ctx, shading_ctx, light);
        }

        for (const Directional_Light& light : scene_ctx.lights.directional_lights) {
            L += direct_lighting_from_directional_light(scene_ctx, shading_ctx, light);
        }

        for (auto [light_index, light] : enumerate(scene_ctx.lights.diffuse_rectangular_lights)) {
            Light_Handle light_handle = {Light_Type::diffuse_rectangular, (int)light_index};

            const MIS_Array_Info& array_info = scene_ctx.array2d_registry.rectangular_light_arrays[light_index];
            const Vector2* light_samples = thread_ctx.pixel_sampler.get_array2d(array_info.light_array_id);
            const Vector2* bsdf_samples = thread_ctx.pixel_sampler.get_array2d(array_info.bsdf_array_id);

            ColorRGB L2;
            for (int i = 0; i < array_info.array_size; i++) {
                L2 += direct_lighting_from_rectangular_light(scene_ctx, shading_ctx, light_handle, light, light_samples[i], bsdf_samples[i]);
            }
            L2 /= float(array_info.array_size);
            L += L2;
        }

        for (auto [light_index, light] : enumerate(scene_ctx.lights.diffuse_sphere_lights)) {
            Light_Handle light_handle = {Light_Type::diffuse_sphere, (int)light_index};
            Diffuse_Sphere_Light_Sampler sampler(light, shading_ctx.P);

            const MIS_Array_Info& array_info = scene_ctx.array2d_registry.sphere_light_arrays[light_index];
            const Vector2* light_samples = thread_ctx.pixel_sampler.get_array2d(array_info.light_array_id);
            const Vector2* bsdf_samples = thread_ctx.pixel_sampler.get_array2d(array_info.bsdf_array_id);

            ColorRGB L2;
            for (int i = 0; i < array_info.array_size; i++) {
                L2 += direct_lighting_from_sphere_light(scene_ctx, shading_ctx, light_handle, sampler, light_samples[i], bsdf_samples[i]);
            }
            L2 /= float(array_info.array_size);
            L += L2;
        }

        if (scene_ctx.has_environment_light_sampler) {
            ColorRGB L2;
            for (int i = 0; i < scene_ctx.environment_light_sampler.light->sample_count; i++) {
                Vector2 u_light = thread_ctx.rng.get_vector2();
                Vector2 u_bsdf = thread_ctx.rng.get_vector2();
                L2 += direct_lighting_from_environment_light(scene_ctx, shading_ctx, u_light, u_bsdf);
            }
            L2 /= float(scene_ctx.environment_light_sampler.light->sample_count);
            L += L2;
        }
    }

    return L;
}

ColorRGB estimate_direct_lighting_from_single_sample(
    const Scene_Context& scene_ctx, const Shading_Context& shading_ctx,
    float u_light_selector, Vector2 u_light, Vector2 u_bsdf)
{
    int light_index = int(u_light_selector * scene_ctx.lights.total_light_count);
    ASSERT(light_index < scene_ctx.lights.total_light_count);

    if (light_index < scene_ctx.lights.point_lights.size()) {
        const Point_Light& light = scene_ctx.lights.point_lights[light_index];
        return (float)scene_ctx.lights.total_light_count * direct_lighting_from_point_light(scene_ctx, shading_ctx, light);
    }
    light_index -= (int)scene_ctx.lights.point_lights.size();

    if (light_index < scene_ctx.lights.directional_lights.size()) {
        const Directional_Light& light = scene_ctx.lights.directional_lights[light_index];
        return (float)scene_ctx.lights.total_light_count * direct_lighting_from_directional_light(scene_ctx, shading_ctx, light);
    }
    light_index -= (int)scene_ctx.lights.directional_lights.size();

    if (light_index < scene_ctx.lights.diffuse_rectangular_lights.size()) {
        Light_Handle light_handle = {Light_Type::diffuse_rectangular, light_index};
        const Diffuse_Rectangular_Light& light = scene_ctx.lights.diffuse_rectangular_lights[light_index];
        return (float)scene_ctx.lights.total_light_count * direct_lighting_from_rectangular_light(scene_ctx, shading_ctx, light_handle, light, u_light, u_bsdf);
    }
    light_index -= (int)scene_ctx.lights.diffuse_rectangular_lights.size();

    if (light_index < scene_ctx.lights.diffuse_sphere_lights.size()) {
        Light_Handle light_handle = {Light_Type::diffuse_sphere, light_index};
        Diffuse_Sphere_Light_Sampler sampler(scene_ctx.lights.diffuse_sphere_lights[light_index], shading_ctx.P);
        return (float)scene_ctx.lights.total_light_count * direct_lighting_from_sphere_light(scene_ctx, shading_ctx, light_handle, sampler, u_light, u_bsdf);
    }
    light_index -= (int)scene_ctx.lights.diffuse_sphere_lights.size();

    // the only light left is environment light
    ASSERT(light_index == 0);
    ASSERT(scene_ctx.has_environment_light_sampler);
    return (float)scene_ctx.lights.total_light_count * direct_lighting_from_environment_light(scene_ctx, shading_ctx, u_light, u_bsdf);
}
