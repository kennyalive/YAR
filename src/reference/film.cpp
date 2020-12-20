#include "std.h"
#include "lib/common.h"
#include "film.h"

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

Film_Tile::Film_Tile(Bounds2i pixel_bounds, Film_Filter filter) {
    this->pixel_bounds = pixel_bounds;
    this->filter = filter;
    pixels.resize(pixel_bounds.area());
    memset(pixels.data(), 0, pixels.size() * sizeof(Film_Pixel));
}

void Film_Tile::add_sample(Vector2 film_pos, ColorRGB color) {
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

Film::Film(Vector2i image_resolution, Bounds2i render_region, Film_Filter filter) {
    this->render_region = render_region;
    this->filter = filter;
    pixels.resize(image_resolution.x * image_resolution.y);
    memset(pixels.data(), 0, pixels.size() * sizeof(Film_Pixel));
}

void Film::merge_tile(const Film_Tile& tile) {
    std::lock_guard<std::mutex> lock(pixels_mutex);

    for (int y = tile.pixel_bounds.p0.y; y < tile.pixel_bounds.p1.y; y++)
        for (int x = tile.pixel_bounds.p0.x; x < tile.pixel_bounds.p1.x; x++) {
            Vector2i p {x, y};
            
            Film_Pixel& film_pixel = get_film_pixel(*this, p);
            const Film_Pixel& tile_pixel = get_tile_pixel(const_cast<Film_Tile&>(tile), p);

            film_pixel.color_sum += tile_pixel.color_sum;
            film_pixel.weight_sum += tile_pixel.weight_sum;
        }
}

std::vector<ColorRGB> Film::get_image() const {
    std::vector<ColorRGB> image(pixels.size());
    for (size_t i = 0; i < pixels.size(); i++) {
        const Film_Pixel& pixel = pixels[i];
        
        ColorRGB color = pixel.color_sum / pixel.weight_sum;

        // handle out-of-gamut values
        color.r = std::max(0.f, color.r);
        color.g = std::max(0.f, color.g);
        color.b = std::max(0.f, color.b);

        image[i] = color;
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
