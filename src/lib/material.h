#pragma once

#include "color.h"
#include "parameter.h"

enum class Material_Type : uint32_t {
    lambertian,
    perfect_reflector,
    perfect_refractor,
    metal,
    plastic,
    coated_diffuse,
    glass,
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

struct Lambertian_Material {
    RGB_Parameter reflectance;
    bool operator==(const Lambertian_Material&) const = default;
};

struct Perfect_Reflector_Material {
    RGB_Parameter reflectance;
    bool operator==(const Perfect_Reflector_Material&) const = default;
};

struct Perfect_Refractor_Material {
    Float_Parameter index_of_refraction;
};

struct Metal_Material {
    Float_Parameter roughness;

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
    Float_Parameter roughness;
    Float_Parameter r0; // reflectance at normal incident angle
    RGB_Parameter diffuse_reflectance; // SSS reflectance inside plastic
    bool operator==(const Plastic_Material&) const = default;
};

struct Coated_Diffuse_Material {
    Float_Parameter roughness; // roughness of the glossy layer
    RGB_Parameter r0; // reflectance of the glossy layer at normal incident angle
    RGB_Parameter diffuse_reflectance; // reflectance of the diffuse layer
    bool operator==(const Coated_Diffuse_Material&) const = default;
};

struct Glass_Material {
    RGB_Parameter reflectance;
    RGB_Parameter transmittance;
    Float_Parameter index_of_refraction;
    bool operator==(const Glass_Material&) const = default;
};

struct Materials {
    std::vector<Lambertian_Material> lambertian;
    std::vector<Perfect_Reflector_Material> perfect_reflector;
    std::vector<Perfect_Refractor_Material> perfect_refractor;
    std::vector<Metal_Material> metal;
    std::vector<Plastic_Material> plastic;
    std::vector<Coated_Diffuse_Material> coated_diffuse;
    std::vector<Glass_Material> glass;
};
