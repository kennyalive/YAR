#include "triangle_mesh.h"

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
