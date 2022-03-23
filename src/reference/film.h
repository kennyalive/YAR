#pragma once

#include "lib/bounding_box.h"
#include "lib/color.h"
#include "lib/image.h"
#include "lib/math.h" // Infinity constant
#include "lib/vector.h"

// ---- Difference between film tiles and sample tiles ----
//
// The sample space is divided into sample tiles.
// Sample tiles do not overlap, so we can render them in parallel.
//
// Film_Tile stores pixels that are affected by the samples from a sample
// tile - it means Film_Tile can be larger than sample tile if filter width
// is large enough (> 0.5).
//
// Also it means that Film_Tiles can overlap. In order to have deterministic
// rendering we need to merge Film_Tiles into the Film in deterministic order.
// (we order tiles according to tile index value).

struct Film_Filter {
    std::function<float(Vector2)> func;
    float radius = 0.f;
};

struct Film_Pixel {
    ColorRGB color_sum; // sum(Weight * Color)
    float weight_sum = 0.f; // sum(Weight)
};

struct Film_Tile {
    Bounds2i pixel_bounds;
    Film_Filter filter;
    float max_rgb_component_value = Infinity;
    std::vector<Film_Pixel> pixels;

    Film_Tile() = default;
    Film_Tile(Bounds2i pixel_bounds, Film_Filter filter, float max_rgb_component_value);
    void add_sample(Vector2 film_pos, ColorRGB color);
};

struct Film {
    Bounds2i render_region;
    Film_Filter filter;

    Bounds2i sample_region;
    Vector2i tile_grid_size;

    std::vector<Film_Pixel> pixels; // has render_region dimensions

    Film(Bounds2i render_region, Film_Filter filter);
    int get_tile_count() const { return tile_grid_size.x * tile_grid_size.y; }
    void get_tile_bounds(int tile_index, Bounds2i& tile_sample_bounds, Bounds2i& tile_pixel_bounds) const;
    void merge_tile(const Film_Tile& tile);
    Image get_image() const;
};

Film_Filter get_box_filter(float radius);
Film_Filter get_gaussian_filter(float radius, float alpha);
Film_Filter get_triangle_filter(float radius);
