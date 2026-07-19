import shared;
import resources;

[[vk::binding(0, 0)]]
ParameterBlock<Materials> materials;

void patch_black(inout int texture_index)
{
    if (texture_index == -1)
        texture_index = None_Texture_Index;
    else
        texture_index += First_Project_Image;
}

[numthreads(1, 1, 1)]
void main()
{
    uint count, stride;
    materials.lambertian_materials.GetDimensions(count, stride);
    
    for (uint i = 0; i < count; i++)
        patch_black(materials.lambertian_materials[i].albedo_texture_index);
}
