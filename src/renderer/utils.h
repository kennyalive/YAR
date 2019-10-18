#pragma once

#include "vk.h"
#include "lib/vector.h"

struct Descriptor_Writes {
    static constexpr uint32_t max_writes = 32;

    struct Accel_Info {
        VkWriteDescriptorSetAccelerationStructureNV accel;
        VkAccelerationStructureNV handle; // referenced by accel
    };

    union Resource_Info {
        VkDescriptorImageInfo   image;
        VkDescriptorBufferInfo  buffer;
        Accel_Info              accel_info;
    };

    VkDescriptorSet         descriptor_set;
    VkWriteDescriptorSet    descriptor_writes[max_writes];
    Resource_Info           resource_infos[max_writes];
    uint32_t                write_count;

    Descriptor_Writes(VkDescriptorSet set) {
        descriptor_set = set;
        write_count = 0;
    }
    ~Descriptor_Writes() {
        commit();
    }

    Descriptor_Writes& sampled_image        (uint32_t binding, VkImageView image_view, VkImageLayout layout);
    Descriptor_Writes& sampled_image_array  (uint32_t binding, uint32_t array_size, const VkDescriptorImageInfo* image_infos);
    Descriptor_Writes& storage_image        (uint32_t binding, VkImageView image_view);
    Descriptor_Writes& sampler              (uint32_t binding, VkSampler sampler);
    Descriptor_Writes& uniform_buffer       (uint32_t binding, VkBuffer buffer, VkDeviceSize offset, VkDeviceSize range);
    Descriptor_Writes& storage_buffer       (uint32_t binding, VkBuffer buffer, VkDeviceSize offset, VkDeviceSize range);
    Descriptor_Writes& storage_buffer_array (uint32_t binding, uint32_t array_size, const VkDescriptorBufferInfo* buffer_infos);
    Descriptor_Writes& accelerator          (uint32_t binding, VkAccelerationStructureNV acceleration_structure);
    void commit();
};

struct Descriptor_Set_Layout {
    static constexpr uint32_t max_bindings = 32;

    VkDescriptorSetLayoutBinding bindings[max_bindings];
    uint32_t binding_count;

    Descriptor_Set_Layout() {
        binding_count = 0;
    }

    Descriptor_Set_Layout& sampled_image        (uint32_t binding, VkShaderStageFlags stage_flags);
    Descriptor_Set_Layout& sample_image_array   (uint32_t binding, uint32_t array_size, VkShaderStageFlags stage_flags);
    Descriptor_Set_Layout& storage_image        (uint32_t binding, VkShaderStageFlags stage_flags);
    Descriptor_Set_Layout& sampler              (uint32_t binding, VkShaderStageFlags stage_flags);
    Descriptor_Set_Layout& uniform_buffer       (uint32_t binding, VkShaderStageFlags stage_flags);
    Descriptor_Set_Layout& storage_buffer       (uint32_t binding, VkShaderStageFlags stage_flags);
    Descriptor_Set_Layout& storage_buffer_array (uint32_t binding, uint32_t array_size, VkShaderStageFlags stage_flags);
    Descriptor_Set_Layout& accelerator          (uint32_t binding, VkShaderStageFlags stage_flags);
    VkDescriptorSetLayout create(const char* name);
};

//
// GPU time queries.
//
struct GPU_Time_Keeper;

struct GPU_Time_Scope {
    GPU_Time_Keeper* parent;
    uint32_t start_query; // end query == (start_query + 1)
    float length_ms;

    void begin();
    void end();
};

struct GPU_Time_Scope_Helper {
    GPU_Time_Scope_Helper(GPU_Time_Scope* time_scope) {
        this->time_scope = time_scope;
        time_scope->begin();
    }
    ~GPU_Time_Scope_Helper() {
        time_scope->end();
    }
private:
    GPU_Time_Scope* time_scope;
};

#define GPU_TIME_SCOPE(time_scope) GPU_Time_Scope_Helper gpu_time_scope##__LINE__(time_scope)

struct GPU_Time_Keeper {
    static constexpr uint32_t max_scopes = 128;

    GPU_Time_Scope scopes[max_scopes];
    uint32_t scope_count = 0;

    GPU_Time_Scope* frame_active_scopes[max_scopes];
    int frame_active_scope_count = 0;

    GPU_Time_Scope* allocate_time_scope();
    void initialize_time_scopes();
    void retrieve_query_results();
};
