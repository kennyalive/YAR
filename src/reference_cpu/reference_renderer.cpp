#include "camera.h"
#include "film.h"
#include "intersection.h"
#include "kdtree.h"
#include "kdtree_builder.h"
#include "light.h"
#include "reference_renderer.h"
#include "triangle_mesh.h"

#include "io/io.h"
#include "lib/common.h"
#include "lib/geometry.h"
#include "lib/mesh.h"
#include "lib/vector.h"

#include <vector>

namespace {
struct Render_Context {
    Bounds2i sample_bounds;
    const Camera* camera;
    const TwoLevel_KdTree* acceleration_structure;
    Lights lights;
};
} // namespace

constexpr int Tile_Size = 64;

void render_tile(const Render_Context& ctx, Bounds2i sample_bounds, Bounds2i pixel_bounds, Film& film) {
    Film_Tile tile(pixel_bounds, film.filter);

    for (int y = sample_bounds.p0.y; y < sample_bounds.p1.y; y++) {
        for (int x = sample_bounds.p0.x; x < sample_bounds.p1.x; x++) {
            Vector2 film_pos = Vector2(x + 0.5f, y + 0.5f);

            Ray ray = ctx.camera->generate_ray(film_pos);

            Local_Geometry local_geom;
            if (ctx.acceleration_structure->intersect(ray, local_geom) == Infinity)
                continue;

            Vector3 wo = (ray.origin - local_geom.position).normalized();
            ColorRGB radiance = compute_direct_lighting(local_geom, ctx.acceleration_structure, ctx.lights, wo, local_geom.material);

            tile.add_sample(film_pos, radiance);
        }
    }
    film.merge_tile(tile);
}

void render_reference_image(const Render_Reference_Image_Params& params, bool* active) {
    Matrix3x4 camera_to_world = params.camera_to_world_vk;
    for (int i = 0; i < 3; i++) {
        float tmp = camera_to_world.a[i][1];
        camera_to_world.a[i][1] = -camera_to_world.a[i][2];
        camera_to_world.a[i][2] = tmp;
    }

    Camera camera(camera_to_world, Vector2(params.image_resolution), 60.f);

    std::vector<Mesh_KdTree> kdtrees(params.scene_data->meshes.size());
    std::vector<Triangle_Mesh> meshes(params.scene_data->meshes.size());

    for (size_t i = 0; i < params.scene_data->meshes.size(); i++) {
        Timestamp t;
        Material_Handle material_handle = register_material(params.scene_data->materials[i]);
        meshes[i] = Triangle_Mesh::from_mesh_data(params.scene_data->meshes[i], material_handle);
        kdtrees[i] = build_kdtree(meshes[i]);
        int time = int(elapsed_milliseconds(t));
        printf("KdTree %zd build time = %dms\n", i, time);

    }

    TwoLevel_KdTree kdtree = build_kdtree(kdtrees);
    printf("two-level tree created\n");

    Lights lights;
    {
        for (const RGB_Point_Light_Data& point_light_data : params.scene_data->rgb_point_lights) {
            Point_Light light;
            light.position = point_light_data.position;
            light.intensity = point_light_data.intensity;
            lights.point_lights.push_back(light);
        }
    }

    ASSERT(params.render_region.p0 >= Vector2i{});
    ASSERT(params.render_region.p1 <= params.image_resolution);
    ASSERT(params.render_region.p0 < params.render_region.p1);

    const float filter_radius = 0.5f;

    Film film(params.render_region.size(), params.render_region, get_box_filter(filter_radius));

    Bounds2i sample_region {
        Vector2i {
            (int32_t)std::ceil(params.render_region.p0.x + 0.5f - filter_radius),
            (int32_t)std::ceil(params.render_region.p0.y + 0.5f - filter_radius)
        },
        Vector2i {
            (int32_t)std::ceil(params.render_region.p1.x-1 + 0.5f + filter_radius),
            (int32_t)std::ceil(params.render_region.p1.y-1 + 0.5f + filter_radius)
        } + Vector2i{1, 1}
    };

    Vector2i sample_region_size = sample_region.p1 - sample_region.p0;

    const int x_tile_count = (sample_region_size.x + Tile_Size - 1) / Tile_Size;
    const int y_tile_count = (sample_region_size.y + Tile_Size - 1) / Tile_Size;

    Render_Context ctx;
    ctx.camera = &camera;
    ctx.acceleration_structure = &kdtree;
    ctx.lights = lights;

    Timestamp t;

    for (int y_tile = 0; y_tile < y_tile_count; y_tile++) {
        for (int x_tile = 0; x_tile < x_tile_count; x_tile++) {
            Bounds2i tile_sample_bounds;
            tile_sample_bounds.p0 = sample_region.p0 + Vector2i{x_tile * Tile_Size, y_tile * Tile_Size};
            tile_sample_bounds.p1.x = std::min(tile_sample_bounds.p0.x + Tile_Size, sample_region.p1.x);
            tile_sample_bounds.p1.y = std::min(tile_sample_bounds.p0.y + Tile_Size, sample_region.p1.y);

            Bounds2i tile_pixel_bounds;
            tile_pixel_bounds.p0.x = std::max((int)std::ceil(tile_sample_bounds.p0.x + 0.5f - filter_radius), params.render_region.p0.x);
            tile_pixel_bounds.p0.y = std::max((int)std::ceil(tile_sample_bounds.p0.y + 0.5f - filter_radius), params.render_region.p0.y);
            tile_pixel_bounds.p1.x = std::min((int)std::ceil(tile_sample_bounds.p1.x-1 + 0.5f + filter_radius) + 1, params.render_region.p1.x);
            tile_pixel_bounds.p1.y = std::min((int)std::ceil(tile_sample_bounds.p1.y-1 + 0.5f + filter_radius) + 1, params.render_region.p1.y);

            render_tile(ctx, tile_sample_bounds, tile_pixel_bounds, film);
        }
    }

    std::vector<ColorRGB> image = film.get_image();

    int time = int(elapsed_milliseconds(t));
    printf("image rendered in %d ms\n", time);

    write_exr_image("image.exr",  image.data(), (int)params.render_region.size().x, (int)params.render_region.size().y);
    *active = false;
}
