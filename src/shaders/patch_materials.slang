#define MATERIAL_SET_INDEX 0
#include "material_resources.slangi"

void patch_black(inout int texture_index)
{
    if (texture_index == -1)
        texture_index = Black_2D_Texture_Index;
    else
        texture_index += Predefined_Texture_Count;
}

[numthreads(1, 1, 1)]
void main()
{
    uint count, stride;
    lambertian_materials.GetDimensions(count, stride);
    
    for (uint i = 0; i < count; i++)
        patch_black(lambertian_materials[i].albedo_texture_index);
}
