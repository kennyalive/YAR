module shading_context;

public struct Shading_Context {
    public float3 Wo; // outgoing direction
    public float3 P;  // shading point position in world coordinates
    public float3 Ng; // geometric normal oriented to be in the same hemisphere as Wo.
    public float3 N;  // shading normal
    public float2 UV; // surface UV parameterization
};
