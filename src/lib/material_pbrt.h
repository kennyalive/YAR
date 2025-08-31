#pragma once

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

struct Pbrt3_Translucent_Material {
    RGB_Parameter reflectance;
    RGB_Parameter transmittance;
    RGB_Parameter diffuse;
    RGB_Parameter specular;
    Float_Parameter roughness;
    Float_Parameter bump_map;

    bool operator==(const Pbrt3_Translucent_Material&) const = default;
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

    Float_Parameter u_roughness;
    Float_Parameter v_roughness;
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
