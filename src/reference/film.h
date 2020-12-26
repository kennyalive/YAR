#pragma once

#include "lib/color.h"
#include "lib/geometry.h"
#include "lib/vector.h"

#include <functional>
#include <mutex>
#include <vector>

struct Film_Filter {
    std::function<float(Vector2)> func;
    float radius;
};

struct Film_Pixel {
    ColorRGB color_sum; // sum(w*c)
    float weight_sum; // sum(w)
};

struct Film_Tile {
    Bounds2i pixel_bounds;
    Film_Filter filter;
    std::vector<Film_Pixel> pixels;

    Film_Tile(Bounds2i pixel_bounds, Film_Filter filter);
    void add_sample(Vector2 film_pos, ColorRGB color);
};

struct Film {
    Bounds2i render_region;
    Film_Filter filter;

    Bounds2i sample_region;
    Vector2i tile_grid_size;

    std::mutex pixels_mutex;
    std::vector<Film_Pixel> pixels;

    Film(Bounds2i render_region, Film_Filter filter);
    int get_tile_count() const { return tile_grid_size.x * tile_grid_size.y; }
    void get_tile_bounds(int tile_index, Bounds2i& tile_sample_bounds, Bounds2i& tile_pixel_bounds) const;
    void merge_tile(const Film_Tile& tile);
    std::vector<ColorRGB> get_image() const;
};

Film_Filter get_box_filter(float radius);
Film_Filter get_gaussian_filter(float radius, float alpha);
Film_Filter get_triangle_filter(float radius);
