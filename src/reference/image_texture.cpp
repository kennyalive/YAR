#include "std.h"
#include "lib/common.h"
#include "image_texture.h"

#include "lib/math.h"
#include "lib/vector.h"

static float lanczos_reconstruction_filter(float x, float radius) {
    x = std::abs(x); // abs here is only to simplify comparisons, the function
                     // itself is symmetrical
    if (x < 1e-5f) 
        return 1.f;
    if (x >= radius)
        return 0.f;
    x *= Pi;
    return radius * std::sin(x) * std::sin(x/radius) / (x*x);
}

// In this implementation we are using 'pre_aliasing_filter' suffix with a
// filter name to denote that the filter is used to remove frequencies higher
// than half of the intended sampling rate, so it can be adequately sampled with
// a given sampling frequency. Here the filter removes frequencies higher than
// sampling_frequency/2.
static float lanczos_pre_aliasing_filter(float x, float radius, float sampling_frequency) {

    // Take into account sampling frequency. Unparameterized sync(x) filter (or
    // sinc(1*x)) removes frequencies higher than 0.5 and allows to sample a
    // signal with a frequency 0.5*2 = 1. It's easy to show (just check
    // frequency space representation) that arbitrary sampling frequency can be
    // taken into account if we replace sinc(x) with sinc(sampling_frequency *
    // x). For example, the 1st mip level is sampled with frequency 0.5, so we
    // use sinc(0.5 * x) - regular sinc stretched 2x along x axis.
    x *= sampling_frequency;

    return lanczos_reconstruction_filter(x, radius);
}

static float mitchell_pre_aliasing_filter(float x, float B, float C, float sampling_frequency) {
    x = std::abs(x);
    x *= sampling_frequency;
    if (x < 1) {
        return (1.f/6.f) * ((12 - 9*B - 6*C) * x*x*x + (-18 + 12*B + 6*C) * x*x + (6 - 2*B));
    }
    else if (x < 2) {
        return (1.f/6.f) * ((-B - 6*C)* x*x*x + (6*B + 30*C)*x*x + (-12*B - 48*C)*x + (8*B + 24*C));
    }
    else { // x >= 2.0
        return 0.f;
    }
}

// x: [-1.0, 1.0] - relative sample position, should be 0 at the center of the
// filter's footprint and -1/+1 on the edges.
static float kaiser_window(float x, float alpha) {
    auto bessel_I0 = [](float k) {
        const double relative_epsilon = 1e-10;
        const double half_x = double(k) / 2.0;
        double sum = 1.0;
        double n = 1.0;
        double t = 1.0;

        while (true) {
            t *= half_x / n++;
            double term_k = t*t;
            sum += term_k;
            if (term_k < sum * relative_epsilon)
                break;
        }
        return float(sum);
    };
    ASSERT(x >= -1.f && x <= 1.f);
    return bessel_I0(alpha * std::sqrt(1.f - x * x)) / bessel_I0(alpha);
}

static float kaiser_pre_aliasing_filter(float x, float radius, float alpha, float sampling_frequency) {
    x = std::abs(x);
    x *= sampling_frequency;

    if (x < 1e-5f)
        return 1.f;
    if (x >= radius) 
        return 0.f;

    float sinc_value = std::sin(Pi*x) / (Pi*x);
    float window_value = kaiser_window(x/radius, alpha);
    return sinc_value * window_value;
}

static const char* get_filter_name(Filter_Type filter) {
    switch (filter) {
    case Filter_Type::box:
        return "box";
    case Filter_Type::lanczos2:
        return "lanczos2";
    case Filter_Type::lanczos3:
        return "lanczos3";
    case Filter_Type::kaiser2_alpha_4:
        return "kaiser2_alpha_4";
    case Filter_Type::kaiser3_alpha_4:
        return "kaiser3_alpha_4";
    case Filter_Type::mitchell_B_1_3_C_1_3:
        return "mitchell";
    default:
        ASSERT(false);
        return nullptr;
    }
}

static float get_filter_radius(Filter_Type filter) {
    switch (filter) {
    case Filter_Type::lanczos2:
        return 2;
    case Filter_Type::lanczos3:
        return 3;
    case Filter_Type::kaiser2_alpha_4:
        return 2;
    case Filter_Type::kaiser3_alpha_4:
        return 3;
    case Filter_Type::mitchell_B_1_3_C_1_3:
        return 2;
    default:
        ASSERT(false);
        return 0.f;
    }
}

static inline float evaluate_pre_aliasing_filter(Filter_Type filter, float x, float sampling_frequency) {
    switch (filter) {
    case Filter_Type::lanczos2:
        return lanczos_pre_aliasing_filter(x, 2.f, sampling_frequency);
    case Filter_Type::lanczos3:
        return lanczos_pre_aliasing_filter(x, 3.f, sampling_frequency);
    case Filter_Type::kaiser2_alpha_4:
        return kaiser_pre_aliasing_filter(x, 2.f, 4.f, sampling_frequency);
    case Filter_Type::kaiser3_alpha_4:
        return kaiser_pre_aliasing_filter(x, 3.f, 4.f, sampling_frequency);
    case Filter_Type::mitchell_B_1_3_C_1_3:
        return mitchell_pre_aliasing_filter(x, 1.f/3.f, 1.f/3.f, sampling_frequency);
    default:
        ASSERT(false);
        return 0.f;
    }
}

// Wrap mode note: For filters with radius > 1.5 mipmap generation algorithm
// should take into account texture wrap mode. The drawback of such correct
// implementation is a dependency between the texture's content and the texture
// addressing mode. Current implementation assumes clamp to edge texture
// addressing mode and will produce slightly incorrect pixels on the edges for
// samplers that use non-clamp addressing.
static Image generate_mip_level_with_separable_filter(const Image& base_image, int mip_level_to_generate, Filter_Type filter)
{
    // for box filter each mip is generated directly from the previous one
    ASSERT(filter != Filter_Type::box);

    ASSERT(mip_level_to_generate >= 1);
    const int mip_width = std::max(1, base_image.width >> mip_level_to_generate);
    const int mip_height = std::max(1, base_image.height >> mip_level_to_generate);

    // Filter's pixel footprint is computed based on the fact that the texels
    // from mip level >= 1 are mapped to integer coordinates of the base mip
    // (and base level texels are on the half-integer grid)
    const int filter_pixel_count = 2 * int(get_filter_radius(filter) + 0.5f) * (1 << (mip_level_to_generate - 1));

    std::vector<float> weights(filter_pixel_count);
    {
        float sampling_frequency = std::pow(0.5f, float(mip_level_to_generate));
        float sum = 0.f;
        float x = float(-filter_pixel_count/2) + 0.5f;
        for (int i = 0; i < filter_pixel_count; i++) {
            weights[i] = evaluate_pre_aliasing_filter(filter, x, sampling_frequency);
            sum += weights[i];
            x += 1.f;
        }
        for (float& w : weights) w /= sum; // normalize weights
    }

    // Downsample in horizontal direction.
    std::vector<ColorRGB> temp(mip_width * base_image.height);
    {
        const int width_ratio = base_image.width / mip_width;
        for (int y = 0; y < base_image.height; y++) {
            int filter_start_x = width_ratio / 2 - filter_pixel_count / 2;
            for (int x = 0; x < mip_width; x++, filter_start_x += width_ratio) {
                ColorRGB& t = temp[y*mip_width + x];
                for (int k = 0; k < filter_pixel_count; k++)
                    t += weights[k] * base_image.data[y*base_image.width + std::clamp(filter_start_x + k, 0, base_image.width - 1)];
            }
        }
    }
    // Downsample in vertical direction.
    Image result(mip_width, mip_height);
    {
        const int height_ratio = base_image.height / mip_height;
        int filter_start_y = height_ratio / 2 - filter_pixel_count / 2;
        for (int y = 0; y < mip_height; y++, filter_start_y += height_ratio) {
            for (int x = 0; x < mip_width; x++) {
                ColorRGB& t = result.data[y*mip_width + x];
                for (int k = 0; k < filter_pixel_count; k++)
                    t += weights[k] * temp[std::clamp(filter_start_y + k, 0, base_image.height - 1)*mip_width + x];
            }
        }
    }

    for (ColorRGB& p : result.data) {
        p.r = std::clamp(p.r, 0.f, 1.f);
        p.g = std::clamp(p.g, 0.f, 1.f);
        p.b = std::clamp(p.b, 0.f, 1.f);
    }
    return result;
}

static Image generate_next_mip_level_with_box_filter(const Image& image) {
    Image result(
        std::max(1, image.width >> 1),
        std::max(1, image.height >> 1)
    );
    if (image.width == 1 || image.height == 1) {
        for (int i = 0; i < int(image.data.size()); i += 2) {
            result.data[i/2] = 0.5f * (image.data[i] + image.data[i+1]);
        }
    }
    else {
        ColorRGB* p = result.data.data();
        for (int y = 0; y < image.height; y += 2) {
            const ColorRGB* row0 = &image.data[y * image.width];
            const ColorRGB* row0_end = row0 + image.width;
            const ColorRGB* row1 = row0_end;
            while (row0 != row0_end) {
                *p++ = 0.25f * (row0[0] + row0[1] + row1[0] + row1[1]);
                row0 += 2;
                row1 += 2;
            }
        }
    }
    return result;
}

void Image_Texture::initialize_from_file(const std::string& image_path, const Image_Texture::Init_Params& params) {
    // Load base mip level.
    bool is_hdr_image;
    Image base_mip;
    if (!base_mip.load_from_file(image_path, params.decode_srgb, &is_hdr_image))
        error("failed to load image file: %s", image_path.c_str());

    // Allocate mip array.
    int mip_count = 1;
    if (params.generate_mips) {
        uint32_t max_size = uint32_t(std::max(base_mip.width, base_mip.height));
        mip_count = log2_int(round_up_to_power_of_2(max_size)) + 1;
    }
    mips.resize(mip_count);
    mips[0] = std::move(base_mip);

    // Ensure that base mip level has power of two resolution.
    if (!is_power_of_2(mips[0].width) || !is_power_of_2(mips[0].height))
        upsample_base_level_to_power_of_two_resolution(!is_hdr_image);

    if (params.generate_mips) {
        generate_mips(params.mip_filter);
        // DEBUG
        #if 0
        for (int i = 0; i < int(mips.size()); i++) {
            const std::string image_name = fs::path(image_path).filename().replace_extension("").string();
            mips[i].write_tga(image_name + "_" + get_filter_name(params.mip_filter) + "_mip_" + std::to_string(i) + ".tga");
        }
        #endif
    }
}

void Image_Texture::upsample_base_level_to_power_of_two_resolution(bool clamp_color_values) {
    struct Resample_Weight {
        int first_pixel;
        float pixel_weight[4]; // weights for 4 consecutive pixels covered by the filter kernel
    };
    auto compute_resample_weights = [](int old_resolution, int new_resolution) {
        std::vector<Resample_Weight> rw(new_resolution);

        const float filter_radius = 2.0f;
        const float new_to_old_sample_pos = float(old_resolution) / float(new_resolution);

        // Determine which pixels from the original image contribute to which pixels of the
        // resampled image and compute corresponding weights.
        for (int i = 0; i < new_resolution; i++) {
            float filter_center = (float(i) + 0.5f) * new_to_old_sample_pos;
            rw[i].first_pixel = int(filter_center - filter_radius + 0.5f);
            for (int k = 0; k < 4; k++) {
                float pixel_pos = rw[i].first_pixel + k + 0.5f;
                rw[i].pixel_weight[k] = lanczos_reconstruction_filter(pixel_pos - filter_center, filter_radius);
            }
            // Normalize weights.
            float weight_sum = 0.f;
            for (float w : rw[i].pixel_weight)
                weight_sum += w;
            float inv_weight_sum = 1.f / weight_sum;
            for (float& w : rw[i].pixel_weight)
                w *= inv_weight_sum;
        }
        return rw;
    };

    // Resample in horizontal direction.
    if (!is_power_of_2(mips[0].width)) {
        int new_width = int(round_up_to_power_of_2(uint32_t(mips[0].width)));
        const std::vector<Resample_Weight> rw = compute_resample_weights(mips[0].width, new_width);
        std::vector<ColorRGB> texels(mips[0].height * new_width);
        for (int y = 0; y < mips[0].height; y++) {
            for (int x = 0; x < new_width; x++) {
                ColorRGB& t = texels[y*new_width + x];
                for (int k = 0; k < 4; k++) {
                    int src_pixel_x = std::clamp(rw[x].first_pixel + k, 0, mips[0].width - 1);
                    t += rw[x].pixel_weight[k] * mips[0].data[y * mips[0].width + src_pixel_x];
                }
                // Filters with negative regions can produce negative color components.
                t.clamp_to_zero_negative_components();
            }
        }
        mips[0].width = new_width;
        mips[0].data.swap(texels);
    }

    // Resample in vertical direction.
    if (!is_power_of_2(mips[0].height)) {
        int new_height = int(round_up_to_power_of_2(uint32_t(mips[0].height)));
        const std::vector<Resample_Weight> rw = compute_resample_weights(mips[0].height, new_height);
        std::vector<ColorRGB> texels(new_height * mips[0].width);
        for (int x = 0; x < mips[0].width; x++) {
            for (int y = 0; y < new_height; y++) {
                ColorRGB& t = texels[y*mips[0].width + x];
                for (int k = 0; k < 4; k++) {
                    int src_pixel_y = std::clamp(rw[y].first_pixel + k, 0, mips[0].height - 1);
                    t += rw[y].pixel_weight[k] * mips[0].data[src_pixel_y * mips[0].width + x];
                }
                // Filters with negative regions can produce negative color components.
                t.clamp_to_zero_negative_components();
            }
        }
        mips[0].height = new_height;
        mips[0].data.swap(texels);
    }

    if (clamp_color_values) {
        for (ColorRGB& t : mips[0].data) {
            t.r = std::clamp(t.r, 0.f, 1.f);
            t.g = std::clamp(t.g, 0.f, 1.f);
            t.b = std::clamp(t.b, 0.f, 1.f);
        }
    }
}

void Image_Texture::generate_mips(Filter_Type filter) {
    for (int i = 1; i < int(mips.size()); i++) {
        if (filter == Filter_Type::box) {
            mips[i] = generate_next_mip_level_with_box_filter(mips[i - 1]);
        }
        else {
            int src_mip = (i > 4) ? 4 : 0; // optimization: only top few mips are generated directly from the base mip.
            mips[i] = generate_mip_level_with_separable_filter(mips[src_mip], i - src_mip, filter);
        }
    }
}

inline ColorRGB get_texel_repeat(const Image& image, int x, int y) {
    ASSERT(is_power_of_2(image.width));
    ASSERT(is_power_of_2(image.height));

    // Simple and correct implementation.
    /*
    x %= image.width;
    x = (x < 0) ? x + image.width : x;
    y %= image.height;
    y = (y < 0) ? y + image.height : y;
    */

    // TODO: can we get rid of % in else blocks?
    if (x >= 0) {
        x &= (image.width - 1);
    }
    else {
        x %= image.width;
        x += (x >> 31) & image.width;
    }
    if (y >= 0) {
        y &= (image.height - 1);
    }
    else {
        y %= image.height;
        y += (y >> 31) & image.height;
    }
    return image.data[y * image.width + x];
}

inline ColorRGB get_texel_clamp(const Image& image, int x, int y) {
    x = std::clamp(x, 0, image.width - 1);
    y = std::clamp(y, 0, image.height - 1);
    return image.data[y * image.width + x];
}

ColorRGB Image_Texture::sample_nearest(const Vector2& uv, int mip_level, Wrap_Mode wrap_mode) const {
    ASSERT(mip_level >= 0 && mip_level < mips.size());
    const Image& image = mips[mip_level];

    int x = int(uv[0] * float(image.width));
    int y = int(uv[1] * float(image.height));

    ColorRGB nearest_texel;
    if (wrap_mode == Wrap_Mode::repeat) {
        nearest_texel = get_texel_repeat(image, x, y);
    }
    else {
        ASSERT(wrap_mode == Wrap_Mode::clamp);
        nearest_texel = get_texel_clamp(image, x, y);
    }
    return nearest_texel;
}

ColorRGB Image_Texture::sample_bilinear(const Vector2& uv, int mip_level, Wrap_Mode wrap_mode) const {
    ASSERT(mip_level >= 0 && mip_level < mips.size());
    const Image& image = mips[mip_level];

    float x = uv.x * float(image.width) - 0.5f;
    float y = uv.y * float(image.height) - 0.5f;

    float x_floor;
    float wx = std::modf(x, &x_floor);

    float y_floor;
    float wy = std::modf(y, &y_floor);

    int x0 = int(x_floor);
    int y0 = int(y_floor);
    int x1 = x0 + 1;
    int y1 = y0 + 1;

    ColorRGB texel00, texel01, texel10, texel11;
    if (wrap_mode == Wrap_Mode::repeat) {
        texel00 = get_texel_repeat(image, x0, y0);
        texel01 = get_texel_repeat(image, x1, y0);
        texel10 = get_texel_repeat(image, x0, y1);
        texel11 = get_texel_repeat(image, x1, y1);
    }
    else {
        ASSERT(wrap_mode == Wrap_Mode::clamp);
        texel00 = get_texel_clamp(image, x0, y0);
        texel01 = get_texel_clamp(image, x1, y0);
        texel10 = get_texel_clamp(image, x0, y1);
        texel11 = get_texel_clamp(image, x1, y1);
    }

    ColorRGB bilinear_sample =
        texel00 * ((1 - wy) * (1 - wx)) +
        texel01 * ((1 - wy) * wx) +
        texel10 * (wy * (1 - wx)) +
        texel11 * (wy * wx);

    return bilinear_sample;
}

ColorRGB Image_Texture::sample_trilinear(const Vector2& uv, float lod, Wrap_Mode wrap_mode) const {
    lod = std::clamp(lod, 0.f, float(mips.size() - 1));

    float lod_floor;
    float t = std::modf(lod, &lod_floor);

    int level0 = int(lod_floor);
    int level1 = std::min(level0 + 1, int(mips.size() - 1));

    ColorRGB mip0_sample = sample_bilinear(uv, level0, wrap_mode);
    ColorRGB mip1_sample = sample_bilinear(uv, level1, wrap_mode);
    ColorRGB trilinear_sample = lerp(mip0_sample, mip1_sample, t);
    return trilinear_sample;
}

static std::vector<float> EWA_filter_weights;

void initalize_EWA_filter_weights(int table_size, float alpha) {
    EWA_filter_weights.resize(table_size);

    for (int i = 0; i < int(EWA_filter_weights.size()); i++) {
        float x = float(i) / float(table_size - 1);
        EWA_filter_weights[i] = std::exp(-alpha * x);
    }
}

// The theory and the algorithm for EWA filter is provided in:
// "Fundamentals of Texture Mapping and Image Warping", thesis by Paul S. Heckbert, 1989
// PBRT book also implements this algorithm.
static ColorRGB do_EWA(const Image& image, Vector2 uv,
    float Ux, float Vx, float Uy, float Vy, Wrap_Mode wrap_mode)
{
    // Transition from UV space to texture space with texels placed at integer coordinates.
    uv[0] = uv[0] * image.width - 0.5f;
    uv[1] = uv[1] * image.height - 0.5f;
    Ux *= image.width;
    Vx *= image.height;
    Uy *= image.width;
    Vy *= image.height;

    // Compute ellipse parameters (quadratic form).
    float A = Vx*Vx + Vy*Vy + 1;
    float B = -2 * (Ux*Vx + Uy*Vy);
    float C = Ux*Ux + Uy*Uy + 1;
    float F = A*C - 0.25f * B*B;

    // (A*C - 0.25*B*B) determines conic section type. For ellipse it's always positive.
    ASSERT(F > 0.f); 

    float inv_F = 1.f / F;
    A *= inv_F;
    B *= inv_F;
    C *= inv_F;

    // It's easy to show that after normalization the inverse determinant 1/(4*A*C - B*B)
    // is equal to 0.25*F where F is from the original equation.
    //float inv_det = 1.f / (4 * (A*C) - B * B);
    float inv_det = 0.25f * F;

    // Determine ellipse bounding box.
    float u_delta = 2 * std::sqrt(C * inv_det); // or std::sqrt(C * F)
    ASSERT(u_delta < 256.f); // filtering range sanity check
    float x0 = std::ceil(uv[0] - u_delta);
    float x1 = std::floor(uv[0] + u_delta);

    float v_delta = 2 * std::sqrt(A * inv_det); // or std::sqrt(A * F)
    ASSERT(v_delta < 256.f); // filtering range sanity check
    float y0 = std::ceil(uv[1] - v_delta);
    float y1 = std::floor(uv[1] + v_delta);

    // Apply pixel's filter (projected into ellipse) to the texels inside the ellipse.
    ColorRGB sum;
    float weight_sum = 0.f;
    for (float y = y0; y <= y1; y++) {
        float yy = y - uv[1];
        for (float x = x0; x <= x1; x++) {
            float xx = x - uv[0];
            float r2 = A * xx*xx + B * xx*yy + C * yy*yy;
            ASSERT(r2 >= 0.f);
            if (r2 < 1.f) {
                int weight_index = int(r2 * EWA_filter_weights.size());
                ASSERT(weight_index < EWA_filter_weights.size());
                float weight = EWA_filter_weights[weight_index];
                if (wrap_mode == Wrap_Mode::repeat) {
                    sum += weight * get_texel_repeat(image, int(x), int(y));
                }
                else {
                    ASSERT(wrap_mode == Wrap_Mode::clamp);
                    sum += weight * get_texel_clamp(image, int(x), int(y));
                }
                weight_sum += weight;
            }
        }
    }
    ASSERT(weight_sum != 0.f);
    return sum / weight_sum;
}

ColorRGB Image_Texture::sample_EWA(Vector2 uv, Vector2 UVx, Vector2 UVy,
    Wrap_Mode wrap_mode, float max_anisotropy) const
{
    if (UVx.length_squared() < UVy.length_squared())
        std::swap(UVx, UVy);

    float major_length = UVx.length();
    float minor_length = UVy.length();

    if (minor_length < 1e-6f)
        return sample_bilinear(uv, 0, wrap_mode);

    if (minor_length * max_anisotropy < major_length) {
        float scale = major_length / (minor_length * max_anisotropy);
        minor_length *= scale;
        UVy *= scale;
    }

    const float lod = std::max(0.f, int(mips.size()) - 1 + std::log2(minor_length));
    float lod_floor;
    float t = std::modf(lod, &lod_floor);

    int level0 = int(lod_floor);
    int level1 = level0 + 1;

    if (level0 >= int(mips.size()) - 1)
        return mips.back().data[0];

    ColorRGB mip0_sample = do_EWA(mips[level0], uv, UVx[0], UVx[1], UVy[0], UVy[1], wrap_mode);
    ColorRGB mip1_sample = do_EWA(mips[level1], uv, UVx[0], UVx[1], UVy[0], UVy[1], wrap_mode);
    ColorRGB final_sample = lerp(mip0_sample, mip1_sample, t);
    return final_sample;
}
