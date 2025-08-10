#pragma once

#include "lib/vector.h"

class Image_Texture;
struct RNG;

float cosine_hemisphere_pdf(float theta_cos);

Vector3 sample_sphere_uniform(Vector2 u);
Vector3 sample_hemisphere_uniform(Vector2 u);
Vector3 sample_hemisphere_cosine(Vector2 u);

Vector3 uniform_sample_cone(Vector2 u, float cos_theta_max);
float uniform_cone_pdf(float cos_theta_max);

Vector3 uniform_sample_triangle_baricentrics(Vector2 u);

// Generates n stratified samples over [0, 1) range.
void generate_stratified_sequence_1d(RNG& rng, int n, float* result);
// Generates nx*ny stratified samples over [0, 1)^2 range.
void generate_stratified_sequence_2d(RNG& rng, int nx, int ny, Vector2* result);

// The assumption that CDF is defined over [0, 1] and is a piecewise-linear function.
// If we divide [0, 1] into 'n' intervals then 'cdf' array defines cdf values at the
// end of each interval. We also implicitly define that cdf(0) = 0.0. The last element
// cdf(n-1) must be 1.0.
//
// u - uniformly distributed random variable from [0, 1).
//
// 'pdf' output parameter is a pdf of the drawn sample sample with respect to the length measure.
// It is guaranteed to be greater than zero.
// 
// 'interval_length' output parameter is an index of the CDF interval (between 0 and n-1) into
// which 'u' variable is mapped.
//
// The function returns a sample from [0, 1) distributed according to the provided CDF.
float sample_from_CDF(float u, const float* cdf, int n, float interval_length /* 1/n */, float* pdf,
    int* interval_index = nullptr, float* remapped_u = nullptr);

// Distribution_1D represents PDF (probability density function) defined over [0..1].
// The samples are drawn according to pdf and belong to [0..1).
class Distribution_1D {
public:
    // The pdf function is proportional to the initialization values.
    void initialize(const float* values, int n);

    // Draws a sample from the distribution. The sample belongs to [0..1).
    // 
    // 'u' - uniformly distributed random variable from [0..1).
    // 
    // 'pdf' output parameter is a pdf of the drawn sample with respect to [0..1] linear measure.
    // It is guaranteed to be greater than zero.
    float sample(float u, float* pdf, float* remapped_u = nullptr) const;

    // For the given sample returns its probability density value.
    // The pdf is calculated with respect to [0..1] uniform measure.
    float pdf(float sample) const;

private:
    int n = 0;
    float interval_length = 0.f; // 1/n
    std::vector<float> cdf;
};

// Distribution_2D represents PDF (probability density function) defined over [0..1]^2.
// The samples are drawn according to pdf and belong to [0..1)^2.
class Distribution_2D {
public:
    // The pdf function is proportional to the initialization values.
    void initialize(const float* values, int nx, int ny);
    void initialize_from_latitude_longitude_radiance_map(const Image_Texture& env_map);

    // Draws a sample from the distribution. The sample belongs to [0..1)^2.
    //
    // 'u' - two uniformly distributed random variables from [0..1).
    //
    // 'pdf_uv' output parameter is a pdf of the drawn sample with respect to [0..1]^2 UV measure.
    // It is guaranteed to be greater than zero.
    Vector2 sample(Vector2 u, float *pdf_uv) const;

    // For the given sample returns its probability density value.
    // The pdf is calculated with respect to [0..1]^2 UV measure,
    // it should be converted to solid angle measure if necessary.
    float pdf_uv(Vector2 sample) const;

private:
    int nx = 0;
    int ny = 0;
    float x_interval_length = 0.f; // 1/nx
    float y_interval_length = 0.f; // 1/ny
    std::vector<float> marginal_CDF_y; // cdf from marginal pdf p(y): ny elements
    std::vector<float> CDFs_x; // cdf for each row: nx * ny elements
};

// Importance sampling of GGX microfacet distribution: D(wh) * dot(wh, N)
// NOTE: This function is not used and is left here mostly as an example.
//       The renderer switched to sampling distribution of visible normals.
Vector3 GGX_sample_microfacet_normal(Vector2 u, float alpha);

// Returns PDF of sampled wh direction (microfacet normal).
// NOTE: This function is not used and is left here mostly as an example.
//       The renderer switched to sampling distribution of visible normals.
float GGX_microfacet_normal_pdf(const Vector3& wh, const Vector3& n, float alpha);

// "Sampling the GGX Distribution of Visible Normals"
// by Eric Heitz, 2018: https://jcgt.org/published/0007/04/01/paper.pdf
//
// wo_local - outgoing direction in shading coordinate system.
Vector3 GGX_sample_visible_microfacet_normal(Vector2 u, const Vector3& wo_local, float alpha_x, float alpha_y);

// Returns PDF of sampled wh direction (microfacet normal).
float GGX_visible_microfacet_normal_pdf(const Vector3& wo, const Vector3& wh, const Vector3& n, float alpha);
