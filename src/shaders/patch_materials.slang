import shared;
import resources;

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
    lambertian_materials.GetDimensions(count, stride);

    for (uint i = 0; i < count; i++)
        patch_black(lambertian_materials[i].albedo_texture_index);
}
