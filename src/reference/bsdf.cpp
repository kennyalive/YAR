#include "std.h"
#include "lib/common.h"
#include "bsdf.h"

#include "parameter_evaluation.h"
#include "sampling.h"
#include "scattering.h"
#include "shading_context.h"
#include "scene_context.h"
#include "thread_context.h"

#include "lib/math.h"

static bool ggx_sample_visible_normals = false;

inline float calculate_microfacet_wi_pdf(const Vector3& wo, const Vector3& wh, const Vector3& n, float alpha) {
    float wh_pdf;
    if (ggx_sample_visible_normals)
        wh_pdf = GGX_visible_microfacet_normal_pdf(wo, wh, n, alpha);
    else
        wh_pdf = GGX_microfacet_normal_pdf(wh, n, alpha);

    // Converts between probability densities:
    // wi_pdf = wh_pdf * dWh/dWi
    // dWh/dWi = 1/4(wh, wi) = 1/4(wh,wo)
    float wi_pdf = wh_pdf / (4 * dot(wh, wo));
    return wi_pdf;
}

inline float cosine_hemisphere_pdf(float theta_cos)
{
    ASSERT(theta_cos >= 0.f);
    return theta_cos * Pi_Inv;
}

// Mapping from pbrt3.
static float roughness_to_alpha(float roughness)
{
    // 'roughness' is a user-friendly value from [0..1] range and we need to remap it to
    // represent 'alpha' parameter from microfacet distribution.

    // TODO: assert is disabled for now, because some materials violate this.
    // Do we need a warning here?
    //ASSERT(roughness >= 0.f && roughness <= 1.f);

    roughness = std::max(roughness, 1e-3f);
    float x = std::log(roughness);

    float alpha =
        1.621420000f +
        0.819955000f * x +
        0.173400000f * x * x +
        0.017120100f * x * x * x +
        0.000640711f * x * x * x * x;

    return alpha;
}

BSDF::BSDF(const Shading_Context& shading_ctx)
    : normal(shading_ctx.normal)
    , tangent(shading_ctx.tangent)
    , bitangent(shading_ctx.bitangent)
{
}

Vector3 BSDF::local_to_world(const Vector3& local_direction) const
{
    return Vector3{
        tangent.x * local_direction.x + bitangent.x * local_direction.y + normal.x * local_direction.z,
        tangent.y * local_direction.x + bitangent.y * local_direction.y + normal.y * local_direction.z,
        tangent.z * local_direction.x + bitangent.z * local_direction.y + normal.z * local_direction.z
    };
}

Vector3 BSDF::world_to_local(const Vector3& world_direction) const
{
    return Vector3 { 
        dot(world_direction, tangent),
        dot(world_direction, bitangent),
        dot(world_direction, normal)
    };
}

Vector3 BSDF::sample_microfacet_normal(Vector2 u, const Vector3& wo, float alpha) const
{
    Vector3 wh_local;
    if (ggx_sample_visible_normals) {
        Vector3 wo_local = world_to_local(wo);
        wh_local = GGX_sample_visible_microfacet_normal(u, wo_local, alpha, alpha);
    }
    else {
        wh_local = GGX_sample_microfacet_normal(u, alpha);
    }
    Vector3 wh = local_to_world(wh_local);
    ASSERT(dot(wh, normal) >= 0.f);
    return wh;
}

//
// Lambertian BRDF
//
Lambertian_BRDF::Lambertian_BRDF(const Thread_Context& thread_ctx, const Lambertian_Material& material)
    : BSDF(thread_ctx.shading_context)
{
    reflection_scattering = true;
    reflectance = evaluate_rgb_parameter(thread_ctx, material.reflectance);
}

ColorRGB Lambertian_BRDF::evaluate(const Vector3& /*wo*/, const Vector3& /*wi*/) const {
    return Pi_Inv * reflectance;
}

ColorRGB Lambertian_BRDF::sample(Vector2 u, const Vector3& wo, Vector3* wi, float* pdf) const {
    Vector3 local_dir = sample_hemisphere_cosine(u);
    *wi = local_to_world(local_dir);
    *pdf = Lambertian_BRDF::pdf(wo, *wi);
    return Pi_Inv * reflectance;
}

float Lambertian_BRDF::pdf(const Vector3& /*wo*/, const Vector3& wi) const {
    ASSERT(dot(normal, wi) >= 0.f);
    return dot(normal, wi) / Pi; // pdf for cosine-weighted hemisphere sampling
}

//
// Diffuse Transmission BSDF
//
Diffuse_Transmission_BSDF::Diffuse_Transmission_BSDF(const Thread_Context& thread_ctx,
    const Diffuse_Transmission_Material& material)
    : BSDF(thread_ctx.shading_context)
{
    reflection_scattering = true;
    transmission_scattering = true;

    float scale = evaluate_float_parameter(thread_ctx, material.scale);

    reflectance = scale * evaluate_rgb_parameter(thread_ctx, material.reflectance);
    reflectance.clamp_to_unit_range();

    transmittance = scale * evaluate_rgb_parameter(thread_ctx, material.transmittance);
    transmittance.clamp_to_unit_range();
}

ColorRGB Diffuse_Transmission_BSDF::evaluate(const Vector3& wo, const Vector3& wi) const
{
    bool same_hemisphere = dot(wo, normal) * dot(wi, normal) > 0.f;
    if (same_hemisphere)
        return Pi_Inv * reflectance;
    else
        return Pi_Inv * transmittance;
}

ColorRGB Diffuse_Transmission_BSDF::sample(Vector2 u, const Vector3& wo, Vector3* wi, float* pdf) const
{
    float max_r = reflectance.max_component_value();
    float max_t = transmittance.max_component_value();
    float p = max_r / (max_r + max_t);

    Vector3 local_dir;
    if (u[0] < p) { // sample reflectance
        u[0] = std::min(u[0] / p, One_Minus_Epsilon); // remap to [0, 1) range
        local_dir = sample_hemisphere_cosine(u);
    }
    else { // sample tranmittance
        u[0] = std::min((u[0] - p) / (1.f - p), One_Minus_Epsilon); // remap to [0, 1) range
        local_dir = -sample_hemisphere_cosine(u);
    }
    *wi = local_to_world(local_dir);
    *pdf = Diffuse_Transmission_BSDF::pdf(wo, *wi);
    return Diffuse_Transmission_BSDF::evaluate(wo, *wi);
}

float Diffuse_Transmission_BSDF::pdf(const Vector3& wo, const Vector3& wi) const
{
    float max_r = reflectance.max_component_value();
    float max_t = transmittance.max_component_value();

    float cos_theta = std::abs(dot(normal, wi));
    float pdf = cosine_hemisphere_pdf(cos_theta);

    bool same_hemisphere = dot(wo, normal) * dot(wi, normal) > 0.f;
    if (same_hemisphere)
        return (max_r / (max_r + max_t)) * pdf;
    else
        return (max_t / (max_r + max_t)) * pdf;
}

//
// Metal BRDF
//
Metal_BRDF::Metal_BRDF(const Thread_Context& thread_ctx, const Metal_Material& material)
    : BSDF(thread_ctx.shading_context)
{
    reflection_scattering = true;

    float roughness = evaluate_float_parameter(thread_ctx, material.roughness);
    if (material.roughness_is_alpha)
        alpha = roughness;
    else
        alpha = roughness_to_alpha(roughness);

    eta_i = evaluate_float_parameter(thread_ctx, material.eta_i);
    eta_t = evaluate_rgb_parameter(thread_ctx, material.eta);
    k_t = evaluate_rgb_parameter(thread_ctx, material.k);
}

ColorRGB Metal_BRDF::evaluate(const Vector3& wo, const Vector3& wi) const {
    Vector3 wh = (wo + wi).normalized();

    float cos_theta_i = dot(wi, wh);
    ASSERT(cos_theta_i >= 0);

    ColorRGB F = conductor_fresnel(cos_theta_i, eta_i, eta_t, k_t);

    float D = GGX_Distribution::D(wh, normal, alpha);
    float G = GGX_Distribution::G(wi, wo, normal, alpha);
    
    ColorRGB f = (G * D) * F / (4.f * dot(normal, wo) * dot(normal, wi));
    return f;
}

ColorRGB Metal_BRDF::sample(Vector2 u, const Vector3& wo, Vector3* wi, float* pdf) const {
    ASSERT_ZERO_TO_ONE_RANGE_VECTOR2(u);

    Vector3 wh = sample_microfacet_normal(u, wo, alpha);
    *wi = reflect(wo, wh);

    if (dot(normal, *wi) <= 0.f)
        return Color_Black;

    *pdf = calculate_microfacet_wi_pdf(wo, wh, normal, alpha);
    return evaluate(wo, *wi);
}

float Metal_BRDF::pdf(const Vector3& wo, const Vector3& wi) const {
    ASSERT(dot(normal, wi) >= 0.f);
    Vector3 wh = (wo + wi).normalized();
    return calculate_microfacet_wi_pdf(wo, wh, normal, alpha);
}

//
// Plastic BRDF
//
Plastic_BRDF::Plastic_BRDF(const Thread_Context& thread_ctx, const Plastic_Material& params)
    : BSDF(thread_ctx.shading_context)
{
    reflection_scattering = true;

    float roughness = evaluate_float_parameter(thread_ctx, params.roughness);
    if (params.roughness_is_alpha)
        alpha = roughness;
    else
        alpha = roughness_to_alpha(roughness);

    r0 = evaluate_float_parameter(thread_ctx, params.r0);
    diffuse_reflectance = evaluate_rgb_parameter(thread_ctx, params.diffuse_reflectance);
}

ColorRGB Plastic_BRDF::evaluate(const Vector3& wo, const Vector3& wi) const
{
    Vector3 wh = (wo + wi).normalized();

    float cos_theta_i = dot(wi, wh);
    ASSERT(cos_theta_i >= 0);

    ColorRGB F = schlick_fresnel(ColorRGB(0.04f), cos_theta_i);

    float D = GGX_Distribution::D(wh, normal, alpha);
    float G = GGX_Distribution::G(wi, wo, normal, alpha);

    ColorRGB specular_brdf = (G * D) * F * r0 / (4.f * dot(normal, wo) * dot(normal, wi));
    ColorRGB diffuse_brdf = diffuse_reflectance * Pi_Inv;
    return diffuse_brdf + specular_brdf;
}

ColorRGB Plastic_BRDF::sample(Vector2 u, const Vector3& wo, Vector3* wi, float* pdf) const
{
    if (u[0] < 0.5f) { // sample diffuse
        u[0] *= 2.f; // remap u[0] to [0, 1) range

        Vector3 local_dir = sample_hemisphere_cosine(u);
        *wi = local_to_world(local_dir);
    }
    else { // sample specular
        u[0] = (u[0] - 0.5f) * 2.f; // remap u[0] to [0, 1) range
        Vector3 wh = sample_microfacet_normal(u, wo, alpha);
        *wi = reflect(wo, wh);
    }

    if (dot(normal, *wi) <= 0.f)
        return Color_Black;

    *pdf = Plastic_BRDF::pdf(wo, *wi);
    return evaluate(wo, *wi);
}

float Plastic_BRDF::pdf(const Vector3& wo, const Vector3& wi) const
{
    ASSERT(dot(normal, wi) >= 0.f);
    float diffuse_pdf = dot(normal, wi) / Pi;

    Vector3 wh = (wo + wi).normalized();
    float spec_pdf = calculate_microfacet_wi_pdf(wo, wh, normal, alpha);

    float pdf = 0.5f * (diffuse_pdf + spec_pdf);
    return pdf;
}

//
// Ashikhmin_Shirley_Phong_BRDF
//
// BRDF described in "An Anisotropic Phong Light Reflection Model", Michael Ashikhmin, Peter Shirley.
// https://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.18.4504&rep=rep1&type=pdf
//
Ashikhmin_Shirley_Phong_BRDF::Ashikhmin_Shirley_Phong_BRDF(const Thread_Context& thread_ctx, const Coated_Diffuse_Material& params)
    : BSDF(thread_ctx.shading_context)
{
    reflection_scattering = true;

    float roughness = evaluate_float_parameter(thread_ctx, params.roughness);
    if (params.roughness_is_alpha)
        alpha = roughness;
    else
        alpha = roughness_to_alpha(roughness);

    r0 = evaluate_rgb_parameter(thread_ctx, params.r0);
    diffuse_reflectance = evaluate_rgb_parameter(thread_ctx, params.diffuse_reflectance);
}

ColorRGB Ashikhmin_Shirley_Phong_BRDF::evaluate(const Vector3& wo, const Vector3& wi) const {
    Vector3 wh = (wo + wi).normalized();

    float cos_theta_i = dot(wi, wh);
    ASSERT(cos_theta_i >= 0);

    ColorRGB F = schlick_fresnel(r0, cos_theta_i);
    float D = GGX_Distribution::D(wh, normal, alpha);

    ColorRGB specular_brdf = F * (D / (4.f * cos_theta_i * std::max(dot(normal, wo), dot(normal, wi))));

    auto pow5 = [](float v) { return (v * v) * (v * v) * v; };

    ColorRGB diffuse_brdf =
            (diffuse_reflectance * (ColorRGB(1.f) - r0)) *
            (28.f / (23.f*Pi) * (1.f - pow5(1.f - 0.5f * dot(normal, wi))) * (1.f - pow5(1.f - 0.5f * dot(normal, wo))));

    return diffuse_brdf + specular_brdf;
}

ColorRGB Ashikhmin_Shirley_Phong_BRDF::sample(Vector2 u, const Vector3& wo, Vector3* wi, float* pdf) const {
    if (u[0] < 0.5f) { // sample diffuse
        u[0] *= 2.f; // remap u[0] to [0, 1) range

        Vector3 local_dir = sample_hemisphere_cosine(u);
        *wi = local_to_world(local_dir);
    }
    else { // sample specular
        u[0] = (u[0] - 0.5f) * 2.f; // remap u[0] to [0, 1) range
        Vector3 wh = sample_microfacet_normal(u, wo, alpha);
        *wi = reflect(wo, wh);
    }

    if (dot(normal, *wi) <= 0.f)
        return Color_Black;

    *pdf = Ashikhmin_Shirley_Phong_BRDF::pdf(wo, *wi);
    return evaluate(wo, *wi);
}

float Ashikhmin_Shirley_Phong_BRDF::pdf(const Vector3& wo, const Vector3& wi) const {
    ASSERT(dot(normal, wi) >= 0.f);
    float diffuse_pdf = dot(normal, wi) / Pi;

    Vector3 wh = (wo + wi).normalized();
    float spec_pdf = calculate_microfacet_wi_pdf(wo, wh, normal, alpha);

    float pdf = 0.5f * (diffuse_pdf + spec_pdf);
    return pdf;
}

//
// Uber material from PBRT 3
//
Pbrt3_Uber_BRDF::Pbrt3_Uber_BRDF(const Thread_Context& thread_ctx, const Pbrt3_Uber_Material& params)
    : BSDF(thread_ctx.shading_context)
{
    reflection_scattering = true;
    opacity = evaluate_rgb_parameter(thread_ctx, params.opacity);
    diffuse_reflectance = evaluate_rgb_parameter(thread_ctx, params.diffuse_reflectance);
    specular_reflectance = evaluate_rgb_parameter(thread_ctx, params.specular_reflectance);

    float roughness = evaluate_float_parameter(thread_ctx, params.roughness);
    if (params.roughness_is_alpha)
        alpha = roughness;
    else
        alpha = roughness_to_alpha(roughness);

    index_of_refraction = evaluate_float_parameter(thread_ctx, params.index_of_refraction);
}

ColorRGB Pbrt3_Uber_BRDF::evaluate(const Vector3& wo, const Vector3& wi) const
{
    ColorRGB diffuse_brdf = Pi_Inv * diffuse_reflectance * opacity;

    Vector3 wh = (wo + wi).normalized();
    float cos_theta_i = dot(wi, wh);
    ASSERT(cos_theta_i >= 0);
    float F = dielectric_fresnel(cos_theta_i, index_of_refraction);
    float D = GGX_Distribution::D(wh, normal, alpha);
    float G = GGX_Distribution::G(wi, wo, normal, alpha);
    float f = (G * D * F) / (4.f * dot(normal, wo) * dot(normal, wi));
    ColorRGB specular_brdf = f * specular_reflectance * opacity;

    return diffuse_brdf + specular_brdf;
}

ColorRGB Pbrt3_Uber_BRDF::sample(Vector2 u, const Vector3& wo, Vector3* wi, float* pdf) const
{
    if (u[0] < 0.5f) { // sample diffuse
        u[0] *= 2.f; // remap u[0] to [0, 1) range
        Vector3 local_dir = sample_hemisphere_cosine(u);
        *wi = local_to_world(local_dir);
    }
    else { // sample specular
        u[0] = (u[0] - 0.5f) * 2.f; // remap u[0] to [0, 1) range
        Vector3 wh = sample_microfacet_normal(u, wo, alpha);
        *wi = reflect(wo, wh);
    }

    if (dot(normal, *wi) <= 0.f)
        return Color_Black;

    *pdf = Pbrt3_Uber_BRDF::pdf(wo, *wi);
    return Pbrt3_Uber_BRDF::evaluate(wo, *wi);
}

float Pbrt3_Uber_BRDF::pdf(const Vector3& wo, const Vector3& wi) const
{
    ASSERT(dot(normal, wi) >= 0.f);
    float diffuse_pdf = dot(normal, wi) / Pi;

    Vector3 wh = (wo + wi).normalized();
    float specular_pdf = calculate_microfacet_wi_pdf(wo, wh, normal, alpha);

    float pdf = 0.5f * (diffuse_pdf + specular_pdf);
    return pdf;
}

//
// Pbrt3 Plastic BRDF
//
Pbrt3_Plastic_BRDF::Pbrt3_Plastic_BRDF(const Thread_Context& thread_ctx, const Plastic_Material& params)
    : Plastic_BRDF(thread_ctx, params)
{
    original_shading_normal = thread_ctx.shading_context.original_shading_normal_was_flipped ? -normal : normal;
}

ColorRGB Pbrt3_Plastic_BRDF::evaluate(const Vector3& wo, const Vector3& wi) const
{
    Vector3 wh = (wo + wi).normalized();

    float cos_theta_i = dot(wi, wh);
    ASSERT(cos_theta_i >= 0);

    // In pbrt3 eta_t is 1.0 and eta_i is 1.5, which is a bug but it became a feature.
    // We need to do the same to produce pbrt3 output.
    float relative_ior = 1.f / 1.5f;

    bool flip_ior = dot(original_shading_normal, wi) < 0.f;
    if (flip_ior)
        relative_ior = 1.5f / 1.f;

    ColorRGB F = ColorRGB(dielectric_fresnel(cos_theta_i, relative_ior));

    float D = GGX_Distribution::D(wh, normal, alpha);
    float G = GGX_Distribution::G(wi, wo, normal, alpha);

    ColorRGB specular_brdf = (G * D) * F * r0 / (4.f * dot(normal, wo) * dot(normal, wi));
    ColorRGB diffuse_brdf = diffuse_reflectance * Pi_Inv;
    return diffuse_brdf + specular_brdf;
}

//
// Pbrt3 Fourier BRDF
//
Pbrt3_Fourier_BSDF::Pbrt3_Fourier_BSDF(const Thread_Context& thread_ctx, const Pbrt3_Fourier_Material& params)
    : BSDF(thread_ctx.shading_context)
    , data(params)
{
    reflection_scattering = true;
    ASSERT(data.eta == 1.f); // support only reflection
}

ColorRGB Pbrt3_Fourier_BSDF::evaluate(const Vector3& wo, const Vector3& wi) const
{
    // fourier bsdf data uses inversed incident direction comparing to our representation
    // (both incident and outgoing directions point away from the surface)
    float cos_i = std::clamp(dot(normal, -wi), -1.f, 1.f);

    float cos_o = std::clamp(dot(normal, wo), -1.f, 1.f);

    auto find_index = [this](float cos_theta) -> int {
        const auto& cosines = data.zenith_angle_discretization;
        auto it = std::lower_bound(cosines.begin(), cosines.end(), cos_theta);
        ASSERT(it != cosines.end());
        return int(it - cosines.begin());
    };
    int index_i = find_index(cos_i);
    int index_o = find_index(cos_o);

    int index = index_o * (int)data.zenith_angle_discretization.size() + index_i;

    uint32_t coeff_count = data.coeff_count[index];
    const float* coeffs = data.coeffs.data() + data.coeff_offset[index];

    float cos_phi = cos_delta_phi(wo, -wi, tangent, bitangent);
    float phi = std::acos(cos_phi);

    // the fourier series computes bsdf * abs(cos_i), so we need to remove cosine
    float scale = (cos_i != 0.f) ? 1.f / std::abs(cos_i) : 0.f;

    ASSERT(data.channel_count == 1 || data.channel_count == 3);

    if (data.channel_count == 1) {
        float y = 0.f;
        for (uint32_t i = 0; i < coeff_count; i++) {
            float coeff_cos = std::cos(float(i) * phi);
            y += coeffs[i] * coeff_cos;
        }
        ColorRGB f(y * scale);
        return f;
    }
    else {
        float y = 0.f;
        float r = 0.f;
        float b = 0.f;
        for (uint32_t i = 0; i < coeff_count; i++) {
            float coeff_cos = std::cos(float(i) * phi);
            y += coeffs[i + 0 * coeff_count] * coeff_cos;
            r += coeffs[i + 1 * coeff_count] * coeff_cos;
            b += coeffs[i + 2 * coeff_count] * coeff_cos;
        }
        float g = get_green_from_YRB(y, r, b);
        ColorRGB f(r * scale, g * scale, b * scale);
        return f;
    }
}

ColorRGB Pbrt3_Fourier_BSDF::sample(Vector2 u, const Vector3& wo, Vector3* wi, float* pdf) const
{
    Vector3 local_dir = sample_hemisphere_cosine(u);
    *wi = local_to_world(local_dir);
    *pdf = Pbrt3_Fourier_BSDF::pdf(wo, *wi);
    return Pbrt3_Fourier_BSDF::evaluate(wo, *wi);
}

float Pbrt3_Fourier_BSDF::pdf(const Vector3& wo, const Vector3& wi) const
{
    ASSERT(dot(normal, wi) >= 0.f);
    return dot(normal, wi) / Pi; // pdf for cosine-weighted hemisphere sampling
}

const BSDF* create_bsdf(Thread_Context& thread_ctx, Material_Handle material) {
    const Scene_Context& scene_ctx = *thread_ctx.scene_context;
    Shading_Context& shading_ctx = thread_ctx.shading_context;
    switch (material.type) {
    case Material_Type::lambertian:
    {
        const Lambertian_Material& params = scene_ctx.materials.lambertian[material.index];
        shading_ctx.apply_bump_map(scene_ctx, params.bump_map);
        void* bsdf_allocation = thread_ctx.memory_pool.allocate<Lambertian_BRDF>();
        return new (bsdf_allocation) Lambertian_BRDF(thread_ctx, params);
    }
    case Material_Type::diffuse_transmission:
    {
        const Diffuse_Transmission_Material& params = scene_ctx.materials.diffuse_transmission[material.index];
        shading_ctx.apply_bump_map(scene_ctx, params.bump_map);
        void* bsdf_allocation = thread_ctx.memory_pool.allocate<Diffuse_Transmission_BSDF>();
        return new (bsdf_allocation) Diffuse_Transmission_BSDF(thread_ctx, params);
    }
    case Material_Type::metal:
    {
        const Metal_Material& params = scene_ctx.materials.metal[material.index];
        shading_ctx.apply_bump_map(scene_ctx, params.bump_map);
        void* bsdf_allocation = thread_ctx.memory_pool.allocate<Metal_BRDF>();
        return new (bsdf_allocation) Metal_BRDF(thread_ctx, params);
    }
    case Material_Type::plastic:
    {
        const Plastic_Material& params = scene_ctx.materials.plastic[material.index];
        shading_ctx.apply_bump_map(scene_ctx, params.bump_map);
        if (thread_ctx.scene_context->pbrt3_scene) {
            void* bsdf_allocation = thread_ctx.memory_pool.allocate<Pbrt3_Plastic_BRDF>();
            return new (bsdf_allocation) Pbrt3_Plastic_BRDF(thread_ctx, params);
        }
        else {
            void* bsdf_allocation = thread_ctx.memory_pool.allocate<Plastic_BRDF>();
            return new (bsdf_allocation) Plastic_BRDF(thread_ctx, params);
        }
    }
    case Material_Type::coated_diffuse:
    {
        const Coated_Diffuse_Material& params = scene_ctx.materials.coated_diffuse[material.index];
        shading_ctx.apply_bump_map(scene_ctx, params.bump_map);
        void* bsdf_allocation = thread_ctx.memory_pool.allocate<Ashikhmin_Shirley_Phong_BRDF>();
        return new (bsdf_allocation) Ashikhmin_Shirley_Phong_BRDF(thread_ctx, params);
    }
    case Material_Type::pbrt3_uber:
    {
        const Pbrt3_Uber_Material& params = scene_ctx.materials.pbrt3_uber[material.index];
        shading_ctx.apply_bump_map(scene_ctx, params.bump_map);
        void* bsdf_allocation = thread_ctx.memory_pool.allocate<Pbrt3_Uber_BRDF>();
        return new (bsdf_allocation) Pbrt3_Uber_BRDF(thread_ctx, params);
    }
    case Material_Type::pbrt3_fourier:
    {
        const Pbrt3_Fourier_Material& params = scene_ctx.materials.pbrt3_fourier[material.index];
        void* bsdf_allocation = thread_ctx.memory_pool.allocate<Pbrt3_Fourier_BSDF>();
        return new (bsdf_allocation) Pbrt3_Fourier_BSDF(thread_ctx, params);
    }
    default:
    {
        ASSERT(false);
        return nullptr;
    }
    }
}
