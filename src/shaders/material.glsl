#include "shared_main.h"
#include "shared_material.h"

const int Material_None = 0; 
const int Material_Lambertian = 1;

layout(std430, set=MATERIAL_SET_INDEX, binding=LAMBERTIAN_MATERIAL_BINDING)
buffer Lambertian_Material_Buffer {
    Lambertian_Material lambertian_materials[];
};

