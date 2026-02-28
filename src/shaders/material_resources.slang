module material_resources;

import shared_main;
import shared_material;

public static const int Material_Lambertian = 0;
static const int Material_None = -1;

public struct Materials {
    public RWStructuredBuffer<Lambertian_Material> lambertian_materials;
}
