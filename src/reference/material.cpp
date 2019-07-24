#include "std.h"
#include "lib/common.h"
#include "lib/material.h"
#include "lib/vector.h"

ColorRGB compute_bsdf(const Materials& materials, Material_Handle mtl, Vector3 wi, Vector3 wo) {
    switch (mtl.type) {
        case Material_Type::lambertian:
            ASSERT(mtl.index < materials.lambertian.size());
            return materials.lambertian[mtl.index].albedo * Pi_Inv;
        default:
            ASSERT(false);
            return Color_Black;
    }
}
