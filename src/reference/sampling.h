#pragma once

#include "lib/vector.h"

class Image_Texture;

Vector3 sample_sphere_uniform(Vector2 u);
Vector3 sample_hemisphere_uniform(Vector2 u);
Vector3 sample_hemisphere_cosine(Vector2 u);

// The assumption that CDF is defined over [0, 1] and is a piecewise-linear function.
// If we divide [0, 1] into 'n' intervals then 'cdf' array defines cdf values at the
// end of each interval. We also implicitly define that cdf(0) = 0.0. The last element
// cdf(n-1) must be 1.0.
//
// u - uniformly distributed random variable from [0, 1).
//
// The function generates samples from [0, 1) distributed according to provided CDF.
// The optional 'pdf' return value is a pdf of the selected sample with respect to length measure.
float sample_from_CDF(float u, const float* cdf, int n, float interval_length /* 1/n */, float* sample_pdf = nullptr);

class Distribution_2D_Sampling {
public:
    void initialize(const float* values, int nx, int ny);
    void initialize_from_latitude_longitude_radiance_map(const Image_Texture& env_map);

    // The function generates values from [0..1)^2 distributed proportionally to the
    // values specified during initialization.
    // The optional 'pdf' return value is a pdf of the selected sample with respect to solid angle measure.
    Vector2 sample(Vector2 u, float *pdf = nullptr) const;

private:
    int nx = 0;
    int ny = 0;
    float x_interval_length = 0.f; // 1/nx
    float y_interval_length = 0.f; // 1/ny
    std::vector<float> marginal_CDF_y; // cdf from marginal pdf p(y): ny elements
    std::vector<float> CDFs_x; // cdf for each row: nx * ny elements
};
