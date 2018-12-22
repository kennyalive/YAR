#include "camera.h"
#include "colorimetry.h"
#include "intersection.h"
#include "kdtree.h"
#include "kdtree_builder.h"
#include "light.h"
#include "spectrum.h"
#include "triangle_mesh.h"

#include "io/io.h"
#include "lib/common.h"
#include "lib/mesh.h"
#include "lib/scene.h"
#include "lib/vector.h"

#include <vector>

Vector3 f_diffuse(Vector3 albedo) {
    return albedo * Pi_Inv;
}

RGB compute_direct_lighting(const Local_Geometry& local_geom, const Scene_Data& scene, const Vector3& wo, Vector3 albedo) {
    RGB L;
    for (const RGB_Point_Light_Data& light : scene.rgb_point_lights) {
        Vector3 light_vec = (light.position - local_geom.position);
        float light_dist_sq_inv = 1.f / light_vec.squared_length();
        Vector3 light_dir = light_vec * std::sqrt(light_dist_sq_inv);
        L += f_diffuse(albedo) * light.intensity * (light_dist_sq_inv * std::max(0.f, dot(local_geom.normal, light_dir)));
    }
    return L;
}

void render_reference_image(const Scene_Data& scene_data, const Matrix3x4& camera_to_world_vk, bool* active) {
    const int w = 1280;
    const int h = 720;

    Matrix3x4 camera_to_world = camera_to_world_vk;
    for (int i = 0; i < 3; i++) {
        float tmp = camera_to_world.a[i][1];
        camera_to_world.a[i][1] = -camera_to_world.a[i][2];
        camera_to_world.a[i][2] = tmp;
    }

    Camera camera(camera_to_world, Vector2(float(w), float(h)), 60.f);

    std::vector<Mesh_KdTree> kdtrees(scene_data.meshes.size());
    std::vector<Triangle_Mesh> meshes(scene_data.meshes.size());

    for (size_t i = 0; i < scene_data.meshes.size(); i++) {
        Timestamp t;
        meshes[i] = Triangle_Mesh::from_mesh_data(scene_data.meshes[i]);
        kdtrees[i] = build_kdtree(meshes[i]);
        int time = int(elapsed_milliseconds(t));
        printf("KdTree %zd build time = %dms\n", i, time);

    }

    TwoLevel_KdTree kdtree = build_kdtree(kdtrees);
    printf("two-level tree created\n");

    std::vector<Vector3> image(w * h);

    Timestamp t;
    Vector3* pixel = image.data();
    for (int i = 0; i < h; i++) {
        for (int j = 0; j < w; j++) {
            Ray ray = camera.generate_ray(Vector2(j + 0.5f, i + 0.5f));
            RGB L;
            if (Local_Geometry local_geom; kdtree.intersect(ray, local_geom) != Infinity) {
                Vector3 wo = (ray.origin - local_geom.position).normalized();
                L = compute_direct_lighting(local_geom, scene_data, wo, local_geom.k_diffuse);
            }
            *pixel++ = Vector3(L[0], L[1], L[2]);
        }
    }

    int time = int(elapsed_milliseconds(t));
    printf("image rendered in %d ms\n", time);

    write_exr_image("image.exr", image.data(), w, h);
    *active = false;
}
