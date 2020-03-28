#include "std.h"
#include "lib/common.h"
#include "image_texture.h"

#include "lib/vector.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb/stb_image.h"

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
    case Filter_Type::box:
        return "box";
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
    // Load image data from file.
    stbi_uc* rgba_texels = nullptr;
    {
        // we need lock because stbi_set_flip_vertically_on_load is not thread safe
        static std::mutex stb_load_mutex;
        std::scoped_lock<std::mutex> lock(stb_load_mutex);

        stbi_set_flip_vertically_on_load(params.flip_vertically);
        int component_count;
        rgba_texels = stbi_load(image_path.c_str(), &base_width, &base_height, &component_count, STBI_rgb_alpha);
    }
    if (rgba_texels == nullptr)
        error("failed to load image file: %s", image_path.c_str());

    // Allocate array for mip images.
    int mip_count = 1;
    if (params.generate_mips) {
        uint32_t max_size = uint32_t(std::max(base_width, base_height));
        mip_count = log2_int(round_up_to_power_of_2(max_size)) + 1;
    }
    mips.resize(mip_count);

    // Convert image data to floating point representation.
    mips[0] = Image(base_width, base_height);
    {
        std::vector<ColorRGB>& dst = mips[0].data;
        const stbi_uc* src = rgba_texels;
        for (int i = 0; i < base_width * base_height; i++, src += 4) {
            dst[i] = ColorRGB(src[0], src[1], src[2]) * (1.f / 255.f);
            if (params.decode_srgb) {
                dst[i].r = srgb_decode(dst[i].r);
                dst[i].g = srgb_decode(dst[i].g);
                dst[i].b = srgb_decode(dst[i].b);
            }
        }
    }
    stbi_image_free(rgba_texels);

    // Ensure that base mip level has power of two resolution.
    if (!is_power_of_2(base_width) || !is_power_of_2(base_height))
        upsample_base_level_to_power_of_two_resolution();

    if (params.generate_mips) {
        {
            Timestamp t;
            for (int i = 0; i < 100; i++)
                generate_mips(params.mip_filter);
            printf("mip gen time: %" PRId64 " ms\n", elapsed_milliseconds(t));
        }
        // DEBUG
        #if 1
        for (int i = 0; i < int(mips.size()); i++) {
            const std::string image_name = fs::path(image_path).filename().replace_extension("").string();
            mips[i].write_tga(image_name + "_" + get_filter_name(params.mip_filter) + "_mip_" + std::to_string(i) + ".tga");
        }
        #endif
        //exit(0);
    }
}

void Image_Texture::upsample_base_level_to_power_of_two_resolution() {
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
    if (!is_power_of_2(base_width)) {
        int new_width = int(round_up_to_power_of_2(uint32_t(base_width)));
        const std::vector<Resample_Weight> rw = compute_resample_weights(base_width, new_width);
        std::vector<ColorRGB> texels(base_height * new_width);
        for (int y = 0; y < base_height; y++) {
            for (int x = 0; x < new_width; x++) {
                ColorRGB& t = texels[y*new_width + x];
                for (int k = 0; k < 4; k++) {
                    int src_pixel_x = std::clamp(rw[x].first_pixel + k, 0, base_width - 1);
                    t += rw[x].pixel_weight[k] * mips[0].data[y * base_width + src_pixel_x];
                }
            }
        }
        base_width = new_width;
        mips[0].data.swap(texels);
    }

    // Resample in vertical direction.
    if (!is_power_of_2(base_height)) {
        int new_height = int(round_up_to_power_of_2(uint32_t(base_height)));
        const std::vector<Resample_Weight> rw = compute_resample_weights(base_height, new_height);
        std::vector<ColorRGB> texels(new_height * base_width);
        for (int x = 0; x < base_width; x++) {
            for (int y = 0; y < new_height; y++) {
                ColorRGB& t = texels[y*base_width + x];
                for (int k = 0; k < 4; k++) {
                    int src_pixel_y = std::clamp(rw[y].first_pixel + k, 0, base_height - 1);
                    t += rw[y].pixel_weight[k] * mips[0].data[src_pixel_y * base_width + x];
                }
            }
        }
        base_height = new_height;
        mips[0].data.swap(texels);
    }

    for (ColorRGB& t : mips[0].data) {
        t.r = std::clamp(t.r, 0.f, 1.f);
        t.g = std::clamp(t.g, 0.f, 1.f);
        t.b = std::clamp(t.b, 0.f, 1.f);
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

ColorRGB Image_Texture::sample_nearest(const Vector2& uv, int mip_level, Wrap_Mode wrap_mode) const {
    ASSERT(mip_level >= 0 && mip_level < mips.size());
    const Image& image = mips[mip_level];

    int i = int(uv.x * image.width);
    int j = int(uv.y * image.height);

    if (wrap_mode == Wrap_Mode::repeat) {
        i %= image.width;
        j %= image.height;
    }
    else {
        ASSERT(wrap_mode == Wrap_Mode::clamp);
        i = std::clamp(i, 0, image.width-1);
        j = std::clamp(j, 0, image.height-1);
    }

    ColorRGB nearest_texel = image.data[j * image.width + i];
    return nearest_texel;
}

ColorRGB Image_Texture::sample_bilinear(const Vector2& uv, int mip_level, Wrap_Mode wrap_mode) const {
    ASSERT(mip_level >= 0 && mip_level < mips.size());
    const Image& image = mips[mip_level];

    float a = uv.x * float(image.width) - 0.5f;
    float b = uv.y * float(image.height) - 0.5f;

    float a_floor = std::floor(a);
    float b_floor = std::floor(b);

    int i0 = int(a_floor);
    int j0 = int(b_floor);
    int i1 = i0 + 1;
    int j1 = j0 + 1;

    if (wrap_mode == Wrap_Mode::repeat) {
        // The integer coordinate is additionally incremented before taking remainder
        // in order to handle the case when coordinate's value is -1.
        i0 = (i0 + image.width) % image.width;
        j0 = (j0 + image.height) % image.height;
        i1 = (i1 + image.width) % image.width;
        j1 = (j1 + image.height) % image.height;
    }
    else {
        ASSERT(wrap_mode == Wrap_Mode::clamp);
        i0 = std::clamp(i0, 0, image.width - 1);
        j0 = std::clamp(j0, 0, image.height - 1);
        i1 = std::clamp(i1, 0, image.width - 1);
        j1 = std::clamp(j1, 0, image.height - 1);
    }

    const std::vector<ColorRGB>& texels = image.data;
    ColorRGB texel00 = texels[j0 * image.width + i0];
    ColorRGB texel01 = texels[j0 * image.width + i1];
    ColorRGB texel10 = texels[j1 * image.width + i0];
    ColorRGB texel11 = texels[j1 * image.width + i1];

    float alpha = a - a_floor;
    float beta = b - b_floor;

    float w_i0 = 1.f - alpha;
    float w_i1 = alpha;
    float w_j0 = 1.f - beta;
    float w_j1 = beta;

    ColorRGB bilinear_texel =
        texel00 * (w_j0 * w_i0) +
        texel01 * (w_j0 * w_i1) +
        texel10 * (w_j1 * w_i0) +
        texel11 * (w_j1 * w_i1);

    return bilinear_texel;
}
