#include "std.h"
#include "lib/common.h"
#include "film.h"

constexpr int Tile_Size = 64;

static Film_Pixel& get_tile_pixel(Film_Tile& tile, Vector2i p) {
    ASSERT(is_inside_bounds(tile.pixel_bounds, p));
    int offset = (p.y - tile.pixel_bounds.p0.y) * tile.pixel_bounds.size().x + (p.x - tile.pixel_bounds.p0.x);
    ASSERT(offset < tile.pixels.size());
    return tile.pixels[offset];
}

static Film_Pixel& get_film_pixel(Film& film, Vector2i p) {
    ASSERT(is_inside_bounds(film.render_region, p));
    int offset = (p.y - film.render_region.p0.y) * film.render_region.size().x + (p.x - film.render_region.p0.x);
    ASSERT(offset < film.pixels.size());
    return film.pixels[offset];
}

Film_Tile::Film_Tile(Bounds2i pixel_bounds, Film_Filter filter, float max_rgb_component_value) {
    this->pixel_bounds = pixel_bounds;
    this->filter = filter;
    this->max_rgb_component_value = max_rgb_component_value;
    pixels.resize(pixel_bounds.area());
    memset(pixels.data(), 0, pixels.size() * sizeof(Film_Pixel));
}

void Film_Tile::add_sample(Vector2 film_pos, ColorRGB color) {
    float max_component = std::max(color.r, std::max(color.g, color.b));
    if (max_component > max_rgb_component_value) {
        color *= (max_rgb_component_value / max_component);
    }

    // find pixels that are affected by the sample
    Bounds2i region;
    region.p0.x = (int)std::ceil(film_pos.x - filter.radius - 0.5f);
    region.p0.y = (int)std::ceil(film_pos.y - filter.radius - 0.5f);
    region.p1.x = (int)std::floor(film_pos.x + filter.radius - 0.5f) + 1;
    region.p1.y = (int)std::floor(film_pos.y + filter.radius - 0.5f) + 1;

    region = intersect_bounds(region, pixel_bounds);

    // add sample contribution to each pixel
    for (int y = region.p0.y; y < region.p1.y; y++) {
        for (int x = region.p0.x; x < region.p1.x; x++) {
            Vector2 pixel_pos { x + 0.5f, y + 0.5f };
            Vector2 filter_point = film_pos - pixel_pos;

            float weight = filter.func(filter_point);

            Film_Pixel& pixel = get_tile_pixel(*this, Vector2i{x, y});
            pixel.color_sum += weight * color;
            pixel.weight_sum += weight;
        }
    }
}

Film::Film(Bounds2i render_region, Film_Filter filter) {
    this->render_region = render_region;
    this->filter = filter;

    sample_region = Bounds2i {
        Vector2i {
            (int32_t)std::floor(render_region.p0.x + 0.5f - filter.radius),
            (int32_t)std::floor(render_region.p0.y + 0.5f - filter.radius)
        },
        Vector2i {
            (int32_t)std::ceil(render_region.p1.x-1 + 0.5f + filter.radius),
            (int32_t)std::ceil(render_region.p1.y-1 + 0.5f + filter.radius)
        }
    };

    tile_grid_size = (sample_region.size() + (Tile_Size - 1)) / Tile_Size;

    pixels.resize(render_region.area());
    memset(pixels.data(), 0, pixels.size() * sizeof(Film_Pixel));
}

void Film::get_tile_bounds(int tile_index, Bounds2i& tile_sample_bounds, Bounds2i& tile_pixel_bounds) const {
    ASSERT(tile_index < get_tile_count());
    int tile_x_pos = tile_index % tile_grid_size.x;
    int tile_y_pos = tile_index / tile_grid_size.x;

    tile_sample_bounds.p0 = sample_region.p0 + Vector2i{ tile_x_pos * Tile_Size, tile_y_pos * Tile_Size };
    tile_sample_bounds.p1.x = std::min(tile_sample_bounds.p0.x + Tile_Size, sample_region.p1.x);
    tile_sample_bounds.p1.y = std::min(tile_sample_bounds.p0.y + Tile_Size, sample_region.p1.y);

    tile_pixel_bounds.p0.x = std::max((int)std::ceil(tile_sample_bounds.p0.x - filter.radius - 0.5f), render_region.p0.x);
    tile_pixel_bounds.p0.y = std::max((int)std::ceil(tile_sample_bounds.p0.y - filter.radius - 0.5f), render_region.p0.y);
    tile_pixel_bounds.p1.x = std::min((int)std::floor(tile_sample_bounds.p1.x + filter.radius - 0.5f) + 1, render_region.p1.x);
    tile_pixel_bounds.p1.y = std::min((int)std::floor(tile_sample_bounds.p1.y + filter.radius - 0.5f) + 1, render_region.p1.y);
}

void Film::merge_tile(const Film_Tile& tile) {
    // The tile merge operation should be mutually exclusive because border pixels from
    // neighbor tiles can overlap. The sample space is divided into sample tiles and
    // they do _not_ overlap. A Film_Tile does not store the sample tile pixels but rather
    // the pixels that are affected by the samples from sample tile - it means Film_Tile
    // can be larger than sample tile if filter width is large enough (> 0.5). Because of
    // this Film_Tiles can overlap which requires synchronization when merging tile pixels
    // into film but sample tiles never overlap which allows to render them in parallel.
    std::lock_guard<std::mutex> lock(pixels_mutex);

    for (int y = tile.pixel_bounds.p0.y; y < tile.pixel_bounds.p1.y; y++) {
        for (int x = tile.pixel_bounds.p0.x; x < tile.pixel_bounds.p1.x; x++) {
            Vector2i p {x, y};
            
            Film_Pixel& film_pixel = get_film_pixel(*this, p);
            const Film_Pixel& tile_pixel = get_tile_pixel(const_cast<Film_Tile&>(tile), p);

            film_pixel.color_sum += tile_pixel.color_sum;
            film_pixel.weight_sum += tile_pixel.weight_sum;
        }
    }

    // Update progress.
    {
        finished_tile_count++;
        int previousPercentage = 100 * (finished_tile_count - 1) / get_tile_count();
        int currentPercentage = 100 * finished_tile_count / get_tile_count();

        if (currentPercentage > previousPercentage)
            printf("\rRendering: %d%%", currentPercentage);
        if (finished_tile_count == get_tile_count())
            printf("\n");
    }
}

Image Film::get_image() const {
    Image image(render_region.size().x, render_region.size().y);
    ColorRGB* image_pixel = image.data.data();

    for (const Film_Pixel& film_pixel : pixels) {
        ColorRGB resolved_color = (film_pixel.weight_sum == 0.f) ?
            Color_Black : film_pixel.color_sum / film_pixel.weight_sum;

        // handle out-of-gamut values
        resolved_color.r = std::max(0.f, resolved_color.r);
        resolved_color.g = std::max(0.f, resolved_color.g);
        resolved_color.b = std::max(0.f, resolved_color.b);

        *image_pixel++ = resolved_color;
    }
    return image;
}

//
// Filters.
//
Film_Filter get_box_filter(float radius) {
    auto box_filter = [](Vector2 p) { return 1.f; };
    return Film_Filter{ box_filter, radius };
}

Film_Filter get_gaussian_filter(float radius, float alpha) {
    float zero_level = std::exp(-alpha*radius*radius);
    
    auto gaussian_filter = [alpha, zero_level](Vector2 p) {
        return std::max(0.f, std::exp(-alpha * p.length_squared()) - zero_level);
    };
    return Film_Filter{ gaussian_filter, radius };
}

Film_Filter get_triangle_filter(float radius) {
    auto triangle_filter = [radius](Vector2 p) {
        return std::max(0.f, radius - std::abs(p.x)) * std::max(0.f, radius - std::abs(p.y));
    };
    return Film_Filter{ triangle_filter, radius };
}
