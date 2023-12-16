#pragma once

#include "color.h"
#include "parameter.h"

enum class Material_Type : uint32_t {
    perfect_reflector,
    perfect_refractor,
    diffuse,
    diffuse_transmission,
    metal,
    plastic,
    coated_diffuse,
    glass,
    pbrt3_uber,
    pbrt3_fourier,
    count,
    null_material = std::numeric_limits<uint32_t>::max()
};

struct Material_Handle {
    Material_Type type = Material_Type::null_material;
    int index = -1;

    bool operator==(const Material_Handle& other) const {
        return type == other.type && index == other.index;
    }
    bool operator!=(const Material_Handle& other) const {
        return !(*this == other);
    }
};

static_assert(sizeof(Material_Handle) == 8);
constexpr Material_Handle Null_Material = { Material_Type::null_material, -1 };
constexpr int Material_Type_Count = static_cast<int>(Material_Type::count);

struct Perfect_Reflector_Material {
    Float_Parameter bump_map;
    RGB_Parameter reflectance;
    bool operator==(const Perfect_Reflector_Material&) const = default;
};

struct Perfect_Refractor_Material {
    Float_Parameter bump_map;
    Float_Parameter index_of_refraction;
};

struct Diffuse_Material {
    Float_Parameter bump_map;
    RGB_Parameter reflectance;
    bool operator==(const Diffuse_Material&) const = default;
};

struct Diffuse_Transmission_Material {
    Float_Parameter bump_map;
    RGB_Parameter reflectance;
    RGB_Parameter transmittance;
    RGB_Parameter scale;
    bool operator==(const Diffuse_Transmission_Material&) const = default;
};

struct Metal_Material {
    Float_Parameter bump_map;

    Float_Parameter roughness;

    // If true, roughness represents a microfacet alpha parameter.
    // Otherwise it's a roughness material property with values in the range [0..1].
    bool roughness_is_alpha = false;

    // If r0 is defined then we use schlick approximation to compute fresnel,
    // otherwise eta/k are used to evaluate full fresnel equations.
    bool is_r0_defined = false;

    // Reflectance at normal incident angle.
    RGB_Parameter r0;

    // If input defines eta/k as _spectral_ data then we use Spectrum -> XYZ -> RGB transformation.
    // This is not a physically-based conversion because eta/k are not perceptual quantities that
    // play nicely with color matching functions. The engineering justification to do this is the 
    // hypothesis that the result of the following computation:
    //
    //      a) Spectrum -> XYZ -> sRGB conversion -> use rgb eta/k to compute rgb fresnel value.
    //
    // will be similar to:
    //
    //      b) compute spectral fresnel based on spectral eta/k and then convert spectral F to
    //      rgb F using color matching functions (this conversion has more connection to reality
    //      because reflectance (F) can be seen as object color).
    //
    // I did the measurements and the results are close. The largest error is for the gold (2-4%)
    // but gold is a bit special in regard that correctly computed rgb reflectance is out of sRGB gamut.
    // For the most tested metals the error is within 1% for normal incident direction and decreases as the angle increases.
    RGB_Parameter eta;
    RGB_Parameter k;
    Float_Parameter eta_i; // IOR of the dielectric that contacts the metal
    bool operator==(const Metal_Material&) const = default;
};

struct Plastic_Material {
    Float_Parameter bump_map;

    Float_Parameter roughness;

    // If true, roughness represents a microfacet alpha parameter.
    // Otherwise it's a roughness material property with values in the range [0..1].
    bool roughness_is_alpha = false;

    Float_Parameter r0; // reflectance at normal incident angle
    RGB_Parameter diffuse_reflectance; // SSS reflectance inside plastic
    bool operator==(const Plastic_Material&) const = default;
};

struct Coated_Diffuse_Material {
    Float_Parameter bump_map;

    Float_Parameter roughness; // roughness of the glossy layer

    // If true, roughness represents a microfacet alpha parameter.
    // Otherwise it's a roughness material property with values in the range [0..1].
    bool roughness_is_alpha = false;

    RGB_Parameter r0; // reflectance of the glossy layer at normal incident angle
    RGB_Parameter diffuse_reflectance; // reflectance of the diffuse layer
    bool operator==(const Coated_Diffuse_Material&) const = default;
};

struct Glass_Material {
    Float_Parameter bump_map;
    RGB_Parameter reflectance;
    RGB_Parameter transmittance;
    Float_Parameter index_of_refraction;

    Float_Parameter roughness;

    // If true, roughness represents a microfacet alpha parameter.
    // Otherwise it's a roughness material property with values in the range [0..1].
    bool roughness_is_alpha = false;

    bool operator==(const Glass_Material&) const = default;
};

struct Pbrt3_Uber_Material {
    Float_Parameter bump_map;
    RGB_Parameter diffuse_reflectance;
    RGB_Parameter specular_reflectance;
    RGB_Parameter delta_reflectance;
    RGB_Parameter delta_transmission;

    // Opacity allows the light to go through the surface without being scattered.
    // Opacity it's one more type of delta scattering.
    //
    // If incoming radiance is L_incoming then the amount of passthrough radiance is:
    //      L_passthrough = (White - Opacity) * L_incoming.
    // The amount of radiance that's being scattered according to other parameters is:
    //      L_before_scattering = Opacity * L_incoming.
    RGB_Parameter opacity;

    Float_Parameter roughness;

    // If true, roughness represents a microfacet alpha parameter.
    // Otherwise it's a roughness material property with values in the range [0..1].
    bool roughness_is_alpha = false;

    Float_Parameter index_of_refraction;

    // NOTE: the following are the derived fields. It's not mandatory to store them
    // as part of material definition. Currently they are used only by the reference
    // renderer. We might also store them in parallel data structure owned by the 
    // reference renderer.
    enum Component_Type : uint8_t {
        DIFFUSE,
        SPECULAR,
        DELTA_REFLECTION,
        DELTA_TRANSMISSION,
        OPACITY
    };
    Component_Type components[5] = {}; // component_count elements
    uint8_t component_count = 0;

    bool operator==(const Pbrt3_Uber_Material&) const = default;
};

// https://pbr-book.org/3ed-2018/Reflection_Models/Fourier_Basis_BSDFs
struct Pbrt3_Fourier_Material {
    bool load_bsdf_file();
    bool operator==(const Pbrt3_Fourier_Material& other) const {
        return bsdf_file == other.bsdf_file;
    }

    std::string bsdf_file;

    // Bounds the number of coefficients in the fourier series.
    uint32_t max_order = 0;

    // 1 for monochromatic BSDF, 3 for RBG (stores luminance, red and blue).
    uint32_t channel_count = 0;

    // Relative IOR: eta(bottom) / eta(top).
    float eta = 1.f;

    // Zenith angle cosines for sampled directions.
    std::vector<float> zenith_angle_discretization;

    std::vector<float> cdf;

    // Coefficients of fourier series.
    std::vector<float> coeffs;

    // The first coefficient for each pair of zenith directions.
    std::vector<float> first_coeffs;

    // Offsets that define the start positions in the coeffs array of the
    // series of fourier coefficients for each pair of zenith directions.
    std::vector<uint32_t> coeff_offset;

    // The number of coefficients in the fourier series for each pair of zenith direction.
    std::vector<uint32_t> coeff_count;
};

struct Materials {
    std::vector<Diffuse_Material> diffuse;
    std::vector<Diffuse_Transmission_Material> diffuse_transmission;
    std::vector<Perfect_Reflector_Material> perfect_reflector;
    std::vector<Perfect_Refractor_Material> perfect_refractor;
    std::vector<Metal_Material> metal;
    std::vector<Plastic_Material> plastic;
    std::vector<Coated_Diffuse_Material> coated_diffuse;
    std::vector<Glass_Material> glass;
    std::vector<Pbrt3_Uber_Material> pbrt3_uber;
    std::vector<Pbrt3_Fourier_Material> pbrt3_fourier;
};
