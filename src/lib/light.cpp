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


