#pragma once

#include "lib/vector.h"

Vector3 sample_sphere_uniform(Vector2 u);
Vector3 sample_hemisphere_uniform(Vector2 u);
Vector3 sample_hemisphere_cosine(Vector2 u);

// The assumption that CDF is defined over [0, 1] and is a piecewise-linear function.
// If we divide [0, 1] into 'n' intervals then 'cdf' array defines cdf values at the
// end of each interval and we implicitly define that cdf(0) = 0.0. The last element
// cdf(n-1) must be 1.0.
//
// u - uniformly distributed random variable from [0, 1).
//
// The function generates values from [0, 1) distributed according to provided CDF.
float sample_from_CDF(float u, const float* cdf, int n, float interval_length /* 1/n */);

class Distribution_2D_Sampling {
public:
    void initialize(const float* values, int nx, int ny);

    // The function generates values from [0..1)[0..1) distributed proportionally to the
    // values provided to initialization routine.
    Vector2 sample(Vector2 u) const;

private:
    int nx = 0;
    int ny = 0;
    float x_interval_length = 0.f; // 1/nx
    float y_interval_length = 0.f; // 1/ny
    std::vector<float> marginal_CDF_y; // cdf from marginal pdf p(y): ny elements
    std::vector<float> CDFs_x; // cdf for each row: nx * ny elements
};
