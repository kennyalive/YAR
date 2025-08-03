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
    mix,
    pbrt3_uber,
    pbrt3_translucent,
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
    Float_Parameter u_roughness;
    Float_Parameter v_roughness;

    // If true, roughness represents a microfacet alpha parameter.
    // Otherwise it's a roughness material property with values in the range [0..1].
    bool roughness_is_alpha = false;

    // If r0 is defined then we use schlick approximation to compute fresnel,
    // otherwise eta/k are used to evaluate full fresnel equations.
    bool is_r0_defined = false;

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

struct Mix_Material {
    Material_Handle material1;
    Material_Handle material2;
    RGB_Parameter mix_amount;

    bool operator==(const Mix_Material&) const = default;
};

#include "material_pbrt.h"

struct Materials {
    std::vector<Diffuse_Material> diffuse;
    std::vector<Diffuse_Transmission_Material> diffuse_transmission;
    std::vector<Perfect_Reflector_Material> perfect_reflector;
    std::vector<Perfect_Refractor_Material> perfect_refractor;
    std::vector<Metal_Material> metal;
    std::vector<Plastic_Material> plastic;
    std::vector<Coated_Diffuse_Material> coated_diffuse;
    std::vector<Glass_Material> glass;
    std::vector<Mix_Material> mix;
    std::vector<Pbrt3_Uber_Material> pbrt3_uber;
    std::vector<Pbrt3_Translucent_Material> pbrt3_translucent;
    std::vector<Pbrt3_Fourier_Material> pbrt3_fourier;
};
