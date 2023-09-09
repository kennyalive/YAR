#include "std.h"
#include "common.h"
#include "material.h"

// https://github.com/wjakob/layerlab/blob/master/include/layer/storage.h
struct Fourier_Bsdf_Header {
    uint8_t     identifier[7];          // Set to 'SCATFUN'
    uint8_t     version;                // Currently version is 1
    uint32_t    flags;                  // 0x01: file contains a BSDF, 0x02: uses harmonic extrapolation
    uint32_t    node_count;             // Number of samples in the elevational discretization
    uint32_t    coeff_count;            // Total number of Fourier series coefficients stored in the file
    uint32_t    max_order;              // Coeff. count for the longest series occurring in the file
    uint32_t    channel_count;          // Number of color channels (usually 1 or 3)
    uint32_t    basis_count;            // Number of BSDF basis functions (relevant for texturing)
    uint32_t    metadata_bytes;         // Size of descriptive metadata that follows the BSDF data
    uint32_t    parameter_count;        // Number of textured material parameters
    uint32_t    parameter_values_count; // Total number of BSDF samples for all textured parameters
    float       eta;                    // Relative IOR through the material (eta(bottom) / eta(top))
    float       alpha[2];               // Beckmann-equiv. roughness on the top (0) and bottom(1) side
    float       unused[2];              // Unused fields to pad the header to 64 bytes
};
static_assert(sizeof(Fourier_Bsdf_Header) == 64);

bool Pbrt3_Fourier_Material::load_bsdf_file()
{
    static_assert(std::endian::native == std::endian::little,
        "fourier bsdf loader assumes little endian byte order");
    const std::array magic = { 'S', 'C', 'A', 'T', 'F', 'U', 'N' };

    const std::vector<uint8_t> data = read_binary_file(bsdf_file);
    const uint8_t* ptr = data.data();

    auto read_floats = [&data, &ptr](uint32_t float_count, std::vector<float>& output) -> bool {
        auto end = ptr + float_count * sizeof(float);
        if (end > data.data() + data.size()) {
            return false;
        }
        output.resize(float_count);
        memcpy(output.data(), ptr, float_count * sizeof(float));
        ptr = end;
        return true;
    };
    auto read_uints = [&data, &ptr](uint32_t uint_count, std::vector<uint32_t>& output) -> bool {
        auto end = ptr + uint_count * sizeof(uint32_t);
        if (end > data.data() + data.size()) {
            return false;
        }
        output.resize(uint_count);
        memcpy(output.data(), ptr, uint_count * sizeof(uint32_t));
        ptr = end;
        return true;
    };

    if (data.size() < sizeof(Fourier_Bsdf_Header)) {
        return false;
    }
    const auto& header = *reinterpret_cast<const Fourier_Bsdf_Header*>(ptr);
    ptr += sizeof(Fourier_Bsdf_Header);

    if (memcmp(header.identifier, magic.data(), 7) != 0) {
        return false;
    }
    if (header.version != 1) {
        return false;
    }
    // Only a subset of BSDF files are supported for simplicity, in particular:
    // monochromatic and RGB files with uniform (i.e. non-textured) material properties
    if (header.flags != 1 ||
        (header.channel_count != 1 && header.channel_count != 3) ||
        header.basis_count != 1 || header.parameter_count != 0 ||
        header.parameter_values_count != 0) {
        return false;
    }
    max_order = header.max_order;
    channel_count = header.channel_count;
    eta = header.eta;

    if (!read_floats(header.node_count, zenith_angle_discretization)) {
        return false;
    }
    if (!read_floats(header.node_count * header.node_count, cdf)) {
        return false;
    }
    std::vector<uint32_t> offset_table;
    if (!read_uints(header.node_count * header.node_count * 2, offset_table)) {
        return false;
    }
    if (!read_floats(header.coeff_count, coeffs)) {
        return false;
    }

    coeff_offset.resize(header.node_count * header.node_count);
    coeff_count.resize(header.node_count * header.node_count);
    first_coeffs.resize(header.node_count * header.node_count);
    for (uint32_t i = 0; i < header.node_count * header.node_count; i++) {
        uint32_t offset = offset_table[2 * i + 0];
        uint32_t count = offset_table[2 * i + 1];
        coeff_offset[i] = offset;
        coeff_count[i] = count;
        first_coeffs[i] = count > 0 ? coeffs[offset] : 0.f;
    }
    return true;
}
