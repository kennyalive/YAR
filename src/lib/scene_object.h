#pragma once

#include "geometry.h"
#include "light.h"
#include "material.h"
#include "matrix.h"

struct Scene_Object {
    Geometry_Handle geometry;
    Material_Handle material;
    Light_Handle area_light;

    // Supported tranforms: translation, rotation, scale (both uniform and non-uniform).
    // Other transform types (e.g. shear) are not supported.
    Matrix3x4 object_to_world_transform;
    Matrix3x4 world_to_object_transform;

    // Traditionally normal is tranformed by: Normal_Xform = transpose(inverse(Object_Xform)).
    // For Object_Xform with only rotation and scale in 3x3 submatrix the above can be re-written
    // in this form: Normal_Xform = Object_Xform * Inverse_Scale_Xform^2.
    Vector3 inv_scale_squared = Vector3(1);

    // This flag can be enabled when geometry defines enclosed volume (no cracks). It allows to properly
    // track transitions between dielectric boundaries. Tracing of nested dielectrics does not care about
    // normal orientation conventions - we keep additional state that allows to track current media.
    // When this flag is not enabled (or can not be enabled due to enclosed volume requirement) then we
    // do ad-hoc dielectric transition tracking using original shading normal orientation to define the
    // notion of inside/outside.
    bool participate_in_nested_dielectrics_tracking = false;
};
