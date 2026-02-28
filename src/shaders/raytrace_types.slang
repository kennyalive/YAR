module raytrace_types;

public struct Attribs {
    public float2 barycentrics;
};

public struct Ray_Payload {
    public float3 color;
};

public struct Shadow_Ray_Payload {
    public float shadow_factor; // 0.0 or 1.0
};

public struct Vertex {
    public float3 p;
    public float3 n;
    public float2 uv;
};

public struct Ray {
    public float3 origin;
    public float3 dir;
};
