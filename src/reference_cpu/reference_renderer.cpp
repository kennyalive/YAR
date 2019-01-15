#include "camera.h"
#include "colorimetry.h"
#include "intersection.h"
#include "kdtree.h"
#include "kdtree_builder.h"
#include "light.h"
#include "reference_renderer.h"
#include "spectrum.h"
#include "triangle_mesh.h"

#include "io/io.h"
#include "lib/common.h"
#include "lib/mesh.h"
#include "lib/vector.h"

#include <vector>

namespace {
struct Render_Tile_Params {
    int x, y, w, h;
    const Camera* camera;
    const TwoLevel_KdTree* acceleration_structure;
    Lights lights;
};
} // namespace

constexpr int Tile_Size = 64;

static std::vector<RGB> render_tile(const Render_Tile_Params& tile) {
    std::vector<RGB> radiance_values(tile.w * tile.h, RGB{});
    RGB* radiance = radiance_values.data();

    for (int y = tile.y; y < tile.y + tile.h; y++) {
        for (int x = tile.x; x < tile.x + tile.w; x++, radiance++) {
            Ray ray = tile.camera->generate_ray(Vector2(x + 0.5f, y + 0.5f));

            Local_Geometry local_geom;
            if (tile.acceleration_structure->intersect(ray, local_geom) == Infinity)
                continue;

            Vector3 wo = (ray.origin - local_geom.position).normalized();
            *radiance = compute_direct_lighting(local_geom, tile.acceleration_structure, tile.lights, wo, local_geom.material);
        }
    }
    return radiance_values;
}

void render_reference_image(const Render_Reference_Image_Params& params, bool* active) {
    Matrix3x4 camera_to_world = params.camera_to_world_vk;
    for (int i = 0; i < 3; i++) {
        float tmp = camera_to_world.a[i][1];
        camera_to_world.a[i][1] = -camera_to_world.a[i][2];
        camera_to_world.a[i][2] = tmp;
    }

    Camera camera(camera_to_world, Vector2(float(params.film_w), float(params.film_h)), 60.f);

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

    const bool crop_image = params.crop_x != 0 || params.crop_y != 0 || params.crop_w != 0 || params.crop_h != 0;

    int image_start_x, image_start_y;
    int image_width, image_height;

    if (crop_image) {
        assert(params.crop_w > 0 && params.crop_h > 0);
        assert(params.crop_x + params.crop_w <= params.film_w);
        assert(params.crop_y + params.crop_h <= params.film_h);

        image_start_x = params.crop_x;
        image_start_y = params.crop_y;
        image_width = params.crop_w;
        image_height = params.crop_h;
    } else {
        image_start_x = 0;
        image_start_y = 0;
        image_width = params.film_w;
        image_height = params.film_h;
    }

    std::vector<RGB> image;
    image.resize(image_width * image_height);

    auto merge_tile_into_image = [&image, image_width](const std::vector<RGB>& tile_radiance, int x, int y, int w, int h) {
        assert(tile_radiance.size() == w * h);
        for (int i = 0; i < h; i++) {
            const RGB* src = tile_radiance.data() + i * w;
            RGB* dst = image.data() + (y + i) * image_width + x;
            memcpy(dst, src, w * sizeof(RGB));
        }
    };

    const int x_tile_count = (image_width + Tile_Size - 1) / Tile_Size;
    const int y_tile_count = (image_height + Tile_Size - 1) / Tile_Size;

    Render_Tile_Params tile;
    tile.camera = &camera;
    tile.acceleration_structure = &kdtree;
    tile.lights = lights;

    Timestamp t;

    for (int y_tile = 0; y_tile < y_tile_count; y_tile++) {
        for (int x_tile = 0; x_tile < x_tile_count; x_tile++) {
            tile.x = image_start_x + x_tile * Tile_Size;
            tile.y = image_start_y + y_tile * Tile_Size;
            tile.w = (x_tile == x_tile_count - 1) ? image_width - (x_tile_count - 1) * Tile_Size : Tile_Size;
            tile.h = (y_tile == y_tile_count - 1) ? image_height - (y_tile_count - 1) * Tile_Size : Tile_Size;

            std::vector<RGB> tile_radiance = render_tile(tile);
            merge_tile_into_image(tile_radiance, tile.x - image_start_x, tile.y - image_start_y, tile.w, tile.h);
        }
    }

    int time = int(elapsed_milliseconds(t));
    printf("image rendered in %d ms\n", time);

    write_exr_image("image.exr", image.data(), image_width, image_height);
    *active = false;
}
