#include "std.h"
#include "lib/common.h"
#include "sampling.h"

#include "image_texture.h"
#include "scattering.h"

#include "lib/math.h"
#include "lib/random.h"

float cosine_hemisphere_pdf(float theta_cos)
{
    ASSERT(theta_cos >= 0.f);
    return theta_cos * Pi_Inv;
}

Vector3 sample_sphere_uniform(Vector2 u) {
    ASSERT(u < Vector2(1));
    float z = 1.f - 2.f * u[0];

    ASSERT(1 - z * z >= 0);
    float r = std::sqrt(1.f - z*z);

    float phi = 2.f * Pi * u[1];
    float x = r * std::cos(phi);
    float y = r * std::sin(phi);
    return {x, y, z};
}

Vector3 sample_hemisphere_uniform(Vector2 u) {
    ASSERT(u < Vector2(1));
    float z = u[0];

    ASSERT(1 - z * z >= 0);
    float r = std::sqrt(1.f - z*z);

    float phi = 2.f * Pi * u[1];
    float x = r * std::cos(phi);
    float y = r * std::sin(phi);
    return {x, y, z};
}

Vector3 sample_hemisphere_cosine(Vector2 u) {
    ASSERT(u < Vector2(1));
    float z = std::sqrt(1.f - u[0]);

    float r = std::sqrt(u[0]);

    float phi = 2.f * Pi * u[1];
    float x = r * std::cos(phi);
    float y = r * std::sin(phi);
    return {x, y, z};
}

Vector3 uniform_sample_cone(Vector2 u, float cos_theta_max) {
    ASSERT(u < Vector2(1));
    float cos_theta = (1.f - u[0]) + u[0] * cos_theta_max;
    ASSERT(cos_theta >= 0.f && cos_theta <= 1.f);
    float sin_theta = std::sqrt(1.f - cos_theta * cos_theta);

    float phi = 2.f * Pi * u[1];
    return {sin_theta * std::cos(phi), sin_theta * std::sin(phi), cos_theta};
}

float uniform_cone_pdf(float cos_theta_max) {
    return 1.f / (2.f * Pi * (1.f - cos_theta_max));
}

void generate_stratified_sequence_1d(RNG& rng, int n, float* result) {
    float dx = 1.f / float(n);
    for (int x = 0; x < n; x++) {
        *result++ = std::min((float(x) + rng.get_float()) * dx, One_Minus_Epsilon);
    }
}

void generate_stratified_sequence_2d(RNG& rng, int nx, int ny, Vector2* result) {
    float dx = 1.f / float(nx);
    float dy = 1.f / float(ny);
    float* f = &result->x;
    for (int y = 0; y < ny; y++) {
        for (int x = 0; x < nx; x++) {
            *f++ = std::min((float(x) + rng.get_float()) * dx, One_Minus_Epsilon);
            *f++ = std::min((float(y) + rng.get_float()) * dy, One_Minus_Epsilon);
        }
    }
}

float sample_from_CDF(float u, const float* cdf, int n, float interval_length /* 1/n */, float* pdf, int* interval_index) {
    ASSERT(u >= 0.f && u < 1.f);
    ASSERT(n >= 1);
    ASSERT(cdf[n-1] == 1.f);

    auto it = std::lower_bound(cdf, cdf + n, u);
    ASSERT(it != cdf + n);
    ASSERT(*it >= u); // just for clarity, it's std::lower_bound guarantee

    int k = int(it - cdf);
    float cdf_a = (k == 0) ? 0.f : cdf[k-1];
    float cdf_b = cdf[k];

    // Check for horizontal segment which means pdf == 0. We should never sample from it.
    // The binary search above in general will skip horizontal segments and the only corner
    // case is when horizontal segment is the first one and u == 0.
    if (cdf_a == cdf_b) {
        // Just to detect this special case the first time and do proper debugging on hot data.
        // After that this assert should be removed and the code below should be uncommented.
        ASSERT(false);

        // Make this code part of implementation when the assert above triggers.
        /*
        ASSERT(k == 0);
        ASSERT(u == 0.f);

        do {
            ASSERT(k < n);
            cdf_a = cdf_b;
            cdf_b = cdf[++k];
        } while (cdf_a == cdf_b);
        */
    }

    *pdf = (cdf_b - cdf_a) * n;
    ASSERT(*pdf > 0.f);

    *interval_index = k;

    float x = (float(k) + (u - cdf_a) / (cdf_b - cdf_a)) * interval_length;
    return std::min(x, One_Minus_Epsilon);
}

void Distribution_2D::initialize(const float* values, int nx, int ny) {
    this->nx = nx;
    this->ny = ny;
    x_interval_length = 1.f / nx;
    y_interval_length = 1.f / ny;

    // Compute sum of values from each row and the sum of all values.
    std::vector<float> row_sums(ny);
    float total_sum = 0.f;
    int last_row_with_non_zero_sum = -1;

    for (int y = 0; y < ny; y++) {
        float row_sum = 0.f;
        for (int x = 0; x < nx; x++) {
            row_sum += values[nx*y + x];
        }
        row_sums[y] = row_sum;
        total_sum += row_sum;
        if (row_sum != 0.f)
            last_row_with_non_zero_sum = y;
    }
    ASSERT(last_row_with_non_zero_sum != -1); // should we handle entirely black distribution?

    // Compute CDF for marginal pdf(y).
    marginal_CDF_y.resize(ny);
    for (int y = 0; y < last_row_with_non_zero_sum; y++) {
        marginal_CDF_y[y] = (y == 0 ? 0.f : marginal_CDF_y[y - 1]) + row_sums[y] / total_sum;
    }
    for (int y = last_row_with_non_zero_sum; y < ny; y++) {
        marginal_CDF_y[y] = 1.f;
    }

    // Compute CDFs for conditional pdf(x|y).
    CDFs_x.resize(nx*ny);
    for (int y = 0; y < ny; y++) {
        float* cdf = &CDFs_x[y*nx];

        if (row_sums[y] == 0.f) {
            // Zeroed rows should never be selected by marginal_CDF_y sampling.
            // Initialize correspodning cdf function with invalid distribution.
            memset(cdf, 0, nx * sizeof(float));
            continue;
        }
        
        for (int x = 0; x < nx - 1; x++) {
            cdf[x] = (x == 0 ? 0.f : cdf[x-1]) + values[y*nx + x] / row_sums[y];
        }
        cdf[nx - 1] = 1.f;
    }
}

void Distribution_2D::initialize_from_latitude_longitude_radiance_map(const Image_Texture& env_map) {
    ASSERT(!env_map.get_mips().empty());
    const Image& image = env_map.get_mips()[0];

    std::vector<float> distribution_coeffs(image.width * image.height);
    float* p = distribution_coeffs.data();

    for (int y = 0; y < image.height; y++) {
        float sin_theta = std::sin((y + 0.5f) / image.height * Pi);
        float v = (y + 0.5f) / image.height;

        for (int x = 0; x < image.width; x++, p++) {
            float u = (x + 0.5f) / image.width;
            ColorRGB radiance = env_map.sample_nearest({u, v}, 0, Wrap_Mode::clamp);
            // Modify luminance-based pdf by multiplying by sin_theta to take into
            // account that sphere slices have area that is proportional to sin_theta.
            // Without this we will oversample when we move towards poles (still the result
            // is correct but with larger variance).
            *p = radiance.luminance() * sin_theta;
        }
    }

    initialize(distribution_coeffs.data(), image.width, image.height);
}

Vector2 Distribution_2D::sample(Vector2 u, float* pdf_uv) const {
    ASSERT(u >= Vector2(0.f) && u < Vector2(1.f));

    float pdf_y;
    int y;
    float ky = sample_from_CDF(u[0], marginal_CDF_y.data(), int(marginal_CDF_y.size()), y_interval_length, &pdf_y, &y);
    ASSERT(pdf_y > 0.f);
    ASSERT(y < ny);

    float pdf_x_given_y;
    int temp_x;
    float kx = sample_from_CDF(u[1], &CDFs_x[y * nx], nx, x_interval_length, &pdf_x_given_y, &temp_x);
    ASSERT(pdf_x_given_y > 0.f);

    *pdf_uv = pdf_y * pdf_x_given_y;
    return {kx, ky};
}

float Distribution_2D::pdf_uv(Vector2 sample) const {
    ASSERT(sample >= Vector2(0.f) && sample < Vector2(1.f));

    int x = int(sample.x * nx);
    ASSERT(x < nx);
    int y = int(sample.y * ny);
    ASSERT(y < ny);

    float pdf_y = (marginal_CDF_y[y] - (y == 0 ? 0.f : marginal_CDF_y[y - 1])) * ny;
    float pdf_x_given_y = (CDFs_x[y*nx + x] - (x == 0 ? 0.f : CDFs_x[y*nx + x - 1])) * nx;
    
    float pdf = pdf_y * pdf_x_given_y;
    return pdf;
}

Vector3 GGX_sample_microfacet_normal(Vector2 u, float alpha) {
    float theta = std::atan(alpha * std::sqrt(u[0] / (1 - u[0])));
    float phi = 2.f * Pi * u[1];
    ASSERT(theta >= 0.f && theta <= Pi_Over_2 + 1e-4f);

    Vector3 wh_local = get_direction_from_spherical_coordinates(theta, phi);
    return wh_local;
}

float GGX_microfacet_normal_pdf(const Vector3& wh, const Vector3& n, float alpha) {
    ASSERT(dot(wh, n) >= 0.f);

    float D = GGX_Distribution::D(wh, n, alpha);
    return D * dot(wh, n);
}

static void sample_visible_slope(Vector2 u, float cos_theta, float* slope_x, float* slope_y) {
    ASSERT(cos_theta >= -1.f && cos_theta <= 1.f);

    // special case (normal incidence)
    if (cos_theta > 0.9999f) {
        float r = std::sqrt(u[0]/(1.f - u[0]));
        float phi = Pi2 * u[1];
        *slope_x = r * std::cos(phi);
        *slope_y = r * std::cos(phi);
        return;
    }

    float sin_theta = std::sqrt(1.f - cos_theta * cos_theta);
    float tan_theta = sin_theta / cos_theta;
    float a = 1.f / tan_theta;
    float G1 = 2.f / (1.f + std::sqrt(1.f + 1.f/(a*a)));

    // sample slope_x
    float A = (2.f * u[0] / G1) - 1.f;
    float tmp = 1.f / (A*A - 1.f);
    if (tmp > 1e10)
        tmp = 1e10;
    float B = tan_theta;
    float D = std::sqrt(std::max(0.f, B*B*tmp*tmp - (A*A - B*B)*tmp));
    float slope_x_1 = B*tmp - D;
    float slope_x_2 = B*tmp + D;
    *slope_x = (A < 0 || slope_x_2 > 1.f/tan_theta) ? slope_x_1 : slope_x_2;

    // sample slope_y
    float S;
    float u2 = u[1];
    if (u2 > 0.5f) {
        S = 1.f;
        u2 = 2.f * (u2 - 0.5f);
    }
    else {
        S = -1.f;
        u2 = 2.f * (0.5f - u2);
    }
    float z = (u2 * (u2 * (u2 * 0.27385f - 0.73369f) + 0.46341f)) /
              (u2 * (u2 * (u2 * 0.093073f + 0.309420f) - 1.f) + 0.597999f);
    *slope_y = S * z * std::sqrt(1.f + (*slope_x) * (*slope_x));
}

Vector3 GGX_sample_visible_microfacet_normal(Vector2 u, const Vector3& wo_local, float alpha_x, float alpha_y) {
    // Stretch Wo (alpha_x/y are also stretched implicitly by the inverse values,
    // so we get isotropic distribution).
    Vector3 wo = Vector3(alpha_x * wo_local.x, alpha_y * wo_local.y, wo_local.z);
    wo.normalize();

    Direction_Info wo_info(wo);

    // Sample P22_wo (for isotropic distribution and we can assume phi=0).
    float slope_x, slope_y;
    sample_visible_slope(u, wo_info.cos_theta, &slope_x, &slope_y);

    // Rotate from default phi=0 frame to phi defined by wo.
    float tmp = wo_info.cos_phi * slope_x - wo_info.sin_phi * slope_y;
    slope_y   = wo_info.sin_phi * slope_x + wo_info.cos_phi * slope_y;
    slope_x = tmp;

    // Unstretch slope.
    slope_x = alpha_x * slope_x;
    slope_y = alpha_y * slope_y;

    // Compute normal.
    Vector3 wh_local = Vector3(-slope_x, -slope_y, 1.f).normalized();
    return wh_local;
}

float GGX_visible_microfacet_normal_pdf(const Vector3& wo, const Vector3& wh, const Vector3& n, float alpha) {
    ASSERT(dot(wh, n) >= 0.f);
    ASSERT(dot(wo, wh) >= 0.f);
    ASSERT(dot(wo, n) >= 0.f);

    float G1 = GGX_Distribution::G1(wo, n, alpha);
    float D = GGX_Distribution::D(wh, n, alpha);

    float wh_pdf = G1 * D * dot(wo, wh) / dot(wo, n);
    return wh_pdf;
}
