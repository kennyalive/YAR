#pragma once

#include "bounding_box.h"
#include <array>
#include <vector>

struct Indexed_Triangle_Mesh {
  std::vector<Vector> vertices;
  std::vector<std::array<int32_t, 3>> face_indices;

  int32_t get_triangle_count() const {
      return static_cast<int32_t>(face_indices.size());
  }

  int32_t get_vertex_count() const {
      return static_cast<int32_t>(vertices.size());
  }

  void get_triangle(int32_t triangle_index, Vector& p0, Vector& p1, Vector& p2) const {
      auto& indices = face_indices[triangle_index];
      p0 = vertices[indices[0]];
      p1 = vertices[indices[1]];
      p2 = vertices[indices[2]];
  }

  Bounding_Box get_triangle_bounds(int32_t triangle_index) const;
  Bounding_Box get_bounds() const;
  void print_info() const;
};

struct Simple_Triangle_Mesh {
    std::vector<Vector> triangles; // 3*triangle_count vertices

    int32_t get_triangle_count() const {
        assert(triangles.size() % 3 == 0);
        return static_cast<int32_t>(triangles.size() / 3);
    }

    void get_triangle(int32_t triangle_index, Vector& p0, Vector& p1, Vector& p2) const {
        assert(triangle_index < get_triangle_count());
        const Vector* p = &triangles[3*triangle_index];
        p0 = p[0];
        p1 = p[1];
        p2 = p[2];
    }

    Bounding_Box get_triangle_bounds(int32_t triangle_index) const;
    Bounding_Box get_bounds() const;
    void print_info() const;

    static Simple_Triangle_Mesh from_indexed_mesh(const Indexed_Triangle_Mesh& mesh);
};
