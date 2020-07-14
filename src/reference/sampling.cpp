#include "std.h"
#include "lib/common.h"
#include "sampling.h"

#include "image_texture.h"

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

float sample_from_CDF(float u, const float* cdf, int n, float interval_length /* 1/n */, float* pdf) {
    ASSERT(u >= 0.f && u < 1.f);
    ASSERT(n >= 1);
    ASSERT(cdf[n-1] == 1.f);

    auto it = std::lower_bound(cdf, cdf + n, u);
    ASSERT(it != cdf + n);

    int k = int(it - cdf);
    float cdf_a = (k == 0) ? 0.f : cdf[k-1];
    float cdf_b = cdf[k];

    if (pdf) {
        *pdf = (cdf_b - cdf_a) * n;
        ASSERT(*pdf > 0.f); // it can be shown this can't be zero
    }

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

    for (int y = 0; y < ny; y++) {
        float row_sum = 0.f;
        for (int x = 0; x < nx; x++) {
            row_sum += values[nx*y + x];
        }
        row_sums[y] = row_sum;
        total_sum += row_sum;
    }

    // Compute CDF for marginal pdf(y).
    marginal_CDF_y.resize(ny);
    for (int y = 0; y < ny - 1; y++) {
        marginal_CDF_y[y] = (y == 0 ? 0.f : marginal_CDF_y[y - 1]) + row_sums[y] / total_sum;
    }
    marginal_CDF_y[ny - 1] = 1.f;

    // Compute CDFs for conditional pdf(x|y).
    CDFs_x.resize(nx*ny);
    for (int y = 0; y < ny; y++) {
        float* cdf = &CDFs_x[y*nx];
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

        // We compute (u, v) texture coordinates for positions in the middle between texels
        // (the texels are positioned at half-integer grid), thus blurring four neighbors.
        // The full explanation can be found in pbrt 3 book, section 14.2.4 Infinite Area Lights:
        // http://www.pbr-book.org/3ed-2018/Light_Transport_I_Surface_Reflection/Sampling_Light_Sources.html#InfiniteAreaLights
        // The idea is that for black texels that have non-black neighbors we have to ensure that
        // resulting pdf is not zero for corresponding region.
        float v = float(y) / image.height;

        for (int x = 0; x < image.width; x++, p++) {
            float u = float(x) / image.width;

            ColorRGB radiance = env_map.sample_bilinear({u, v}, 0, Wrap_Mode::clamp);

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
    float ky = sample_from_CDF(u[0], marginal_CDF_y.data(), int(marginal_CDF_y.size()), y_interval_length, &pdf_y);
    float y = ky * ny;
    ASSERT(y < float(ny));

    float pdf_x_given_y;
    float kx = sample_from_CDF(u[1], &CDFs_x[int(y) * nx], nx, x_interval_length, &pdf_x_given_y);

    if (pdf_uv) {
        *pdf_uv = pdf_y * pdf_x_given_y;
    }

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
