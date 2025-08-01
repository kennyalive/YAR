#include "std.h"
#include "common.h"
#include "light.h"

Triangle_Mesh Diffuse_Rectangular_Light::get_geometry() const {
    Triangle_Mesh mesh;

    mesh.vertices.resize(4);
    float x = size.x / 2.0f;
    float y = size.y / 2.0f;
    mesh.vertices[0] = transform_point(light_to_world_transform, Vector3(-x, -y, 0.f));
    mesh.vertices[1] = transform_point(light_to_world_transform, Vector3( x, -y, 0.f));
    mesh.vertices[2] = transform_point(light_to_world_transform, Vector3( x,  y, 0.f));
    mesh.vertices[3] = transform_point(light_to_world_transform, Vector3(-x,  y, 0.f));

    mesh.normals.resize(4);
    Vector3 n = light_to_world_transform.get_column(2);
    std::fill(mesh.normals.begin(), mesh.normals.end(), n);

    mesh.indices = { 0, 1, 2, 0, 2, 3};
    return mesh;
}

bool Lights::has_lights() const {
    return
        point_lights.size() ||
        spot_lights.size() ||
        directional_lights.size() ||
        diffuse_rectangular_lights.size() ||
        diffuse_sphere_lights.size() ||
        diffuse_triangle_mesh_lights.size() ||
        has_environment_light;
}

void Lights::update_total_light_count() {
    total_light_count =
        (int)point_lights.size() + 
        (int)spot_lights.size() +
        (int)directional_lights.size() +
        (int)diffuse_rectangular_lights.size() +
        (int)diffuse_sphere_lights.size() +
        (int)diffuse_triangle_mesh_lights.size() +
        has_environment_light;
}
