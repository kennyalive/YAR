#include "std.h"
#include "lib/common.h"
#include "sampling.h"

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

float sample_from_CDF(float u, const float* cdf, int n, float interval_length /* 1/n */) {
    ASSERT(u >= 0.f && u < 1.f);
    ASSERT(n >= 1);
    ASSERT(cdf[n-1] == 1.f);

    auto it = std::lower_bound(cdf, cdf + n, u);
    ASSERT(it != cdf + n);

    int k = int(it - cdf);
    float cdf_a = (k == 0) ? 0.f : cdf[k-1];
    float cdf_b = cdf[k];

    float x = (float(k) + (u - cdf_a) / (cdf_b - cdf_a)) * interval_length;
    return std::min(x, One_Minus_Epsilon);
}

void Distribution_2D_Sampling::initialize(const float* values, int nx, int ny) {
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

Vector2 Distribution_2D_Sampling::sample(Vector2 u) const {
    ASSERT(u < Vector2(1));

    float ky = sample_from_CDF(u[0], marginal_CDF_y.data(), int(marginal_CDF_y.size()), y_interval_length);
    float y = ky * ny;
    ASSERT(y < float(ny));

    float kx = sample_from_CDF(u[1], &CDFs_x[int(y) * nx], nx, x_interval_length);

    return {kx, ky};
}
