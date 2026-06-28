#pragma once

#include "kdtree.h"

// Builds kdtree for a triangle mesh.
KdTree build_triangle_mesh_kdtree(const Triangle_Mesh_Geometry_Data* triangle_mesh_geometry_data);

// Builds kdtree that represents the entire scene.
// The leaf nodes contain references to kdtrees associated with scene geometry.
KdTree build_scene_kdtree(const Scene_Geometry_Data* scene_geometry_data);
