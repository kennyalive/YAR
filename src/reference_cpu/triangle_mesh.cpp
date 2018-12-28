#include "triangle_mesh.h"

Triangle_Mesh Triangle_Mesh::from_mesh_data(const Mesh_Data& mesh_data) {
    Triangle_Mesh mesh;
    mesh.vertices.resize(mesh_data.vertices.size());
    mesh.normals.resize(mesh_data.vertices.size());
    mesh.texcoords.resize(mesh_data.vertices.size());
    mesh.face_indices.resize(mesh_data.indices.size());

   for (int i = 0; i < (int)mesh_data.vertices.size(); i++) {
       mesh.vertices[i] = mesh_data.vertices[i].pos;
       mesh.normals[i] = mesh_data.vertices[i].normal;
       mesh.texcoords[i] = mesh_data.vertices[i].uv;
   }
   for (int i = 0; i < (int)mesh_data.indices.size(); i++) {
       mesh.face_indices[i] = (int32_t)mesh_data.indices[i];
   }

   mesh.k_diffuse = mesh_data.k_diffuse;
   mesh.k_specular = mesh_data.k_specular;

    return mesh;
}

Bounding_Box Triangle_Mesh::get_triangle_bounds(int32_t triangle_index) const {
  const int32_t* indices = &face_indices[triangle_index*3];
  auto bounds = Bounding_Box(vertices[indices[0]]);
  bounds.add_point(vertices[indices[1]]);
  bounds.add_point(vertices[indices[2]]);
  return bounds;
}
        
Bounding_Box Triangle_Mesh::get_bounds() const {
  Bounding_Box bounds;
  for (int32_t i = 0; i < get_triangle_count(); i++) {
    bounds = Bounding_Box::get_union(bounds, get_triangle_bounds(i));
  }
  return bounds;
}

void Triangle_Mesh::print_info() const {
    size_t mesh_size = vertices.size() * sizeof(Vector3) + texcoords.size() * sizeof(Vector2) +
        face_indices.size() * sizeof(int32_t);

    printf("[mesh]\n");
    printf("vertex count = %d\n", get_vertex_count());
    printf("triangle count = %d\n", get_triangle_count());
    printf("mesh size = %zdK\n", mesh_size / 1024);
    printf("\n");
}
