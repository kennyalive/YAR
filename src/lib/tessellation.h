#pragma once

#include "triangle_mesh.h"

Triangle_Mesh create_cube_mesh(float size);
Triangle_Mesh create_sphere_mesh(float radius, int subdivision_level, bool texture_v_is_zero_at_bottom);
