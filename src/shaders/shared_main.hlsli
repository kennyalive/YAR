#ifndef SHARED_MAIN_HLSL
#define SHARED_MAIN_HLSL

// shared_xxx.h files contain definitions that are shared between
// shader and cpp code. For shared structures this ensures that
// CPU and GPU sees the same memory layout.
// shared_main.h should be included before any other shared_xxx.h

#ifndef MATERIAL_SET_INDEX
#define MATERIAL_SET_INDEX 1
#endif
static const int LAMBERTIAN_MATERIAL_BINDING = 0;

// Default texture indices.
static const int Black_2D_Texture_Index = 0;
static const int White_2D_Texture_Index = 1;
static const int Predefined_Texture_Count = 2;

#endif
