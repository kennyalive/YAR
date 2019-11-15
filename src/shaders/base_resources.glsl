#include "shared_main.h"

layout(set=BASE_SET_INDEX, binding=0) uniform texture2D images_2d[];
layout(set=BASE_SET_INDEX, binding=1) uniform sampler image_sampler; 

layout(std430, set=BASE_SET_INDEX, binding=BASE_SET_BINDING_INSTANCE_INFO)
readonly buffer Instance_Info_Buffer {
    Instance_Info instance_infos[];
};
 