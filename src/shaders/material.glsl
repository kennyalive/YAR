const int Material_None = 0;
const int Material_Lambertian = 1;

#ifndef MATERIAL_SET_INDEX
#define MATERIAL_SET_INDEX 1
#endif

struct Material_Handle {
    int type;
    int index;
};

struct Lambertian_Material {
    float r, g, b; // albedo in [0, 1] range
    int albedo_texture_index;
};

layout(std430, set=MATERIAL_SET_INDEX, binding=0) buffer Lambertian_Material_Buffer {
    Lambertian_Material lambertian_materials[];
};

