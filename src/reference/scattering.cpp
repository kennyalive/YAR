#include "std.h"
#include "lib/common.h"
#include "scattering.h"

#include "lib/vector.h"

ColorRGB schlick_fresnel(const ColorRGB& R0, float cos_theta) {
    float k = std::max(0.f, 1.f - cos_theta);
    float k2 = k * k;
    float k4 = k2 * k2;
    float k5 = k4 * k;
    return R0 + (Color_White - R0) * k5;
}

float GGX_Distribution::D(const Vector3& wh, const Vector3& n, float alpha) {
    float cos_theta = dot(wh, n);
    if (cos_theta <= 0.f)
        return 0.f;

    float cos2_theta = cos_theta * cos_theta;
    float alpha2 = alpha * alpha;

    // The formula as specified in "Microfacet Models for Refraction through Rough Surfaces".
    // https://www.cs.cornell.edu/~srm/publications/EGSR07-btdf.pdf
    /*
    float cos4_theta = cos2_theta * cos2_theta;
    float tan2_theta = (1.f - cos2_theta) / cos2_theta;
    float k = alpha2 + tan2_theta;
    float D = alpha2 / (Pi * cos4_theta * k * k);
    */

    // Algebraic transformation of the above code which saves 1 div and 1 mul.
    float k = cos2_theta * (alpha2 - 1) + 1;
    float D = alpha2 / (Pi * k *k);

    return D;
}

inline float GGX_lambda(const Vector3& v, const Vector3& n, float alpha) {
    float cos_theta = dot(v, n);
    float cos2_theta = cos_theta * cos_theta;
    float tan2_theta = (1.f - cos2_theta) / cos2_theta; // could be Infinity, that's fine

    float alpha2 = alpha * alpha;

    float lambda = 0.5f * (-1.f + std::sqrt(1.f + alpha2 * tan2_theta));
    return lambda;
}

float GGX_Distribution::G(const Vector3& wi, const Vector3& wo, const Vector3& n, float alpha) {
    return 1.f / (1.f + GGX_lambda(wi, n, alpha) + GGX_lambda(wo, n, alpha));
}
