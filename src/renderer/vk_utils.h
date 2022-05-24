#pragma once

#include "vk.h"

VkPipelineLayout create_pipeline_layout(
    std::initializer_list<VkDescriptorSetLayout> set_layouts,
    std::initializer_list<VkPushConstantRange> push_constant_ranges,
    const char* name);

VkPipeline create_compute_pipeline(const std::string& spirv_file, VkPipelineLayout pipeline_layout, const char* name);
VkDescriptorSet allocate_descriptor_set(VkDescriptorSetLayout set_layout);

struct Shader_Module {
    Shader_Module(const std::string& spirv_file) {
        handle = vk_load_spirv((get_data_directory() / spirv_file).string());
    }
    ~Shader_Module() {
        vkDestroyShaderModule(vk.device, handle, nullptr);
    }
    VkShaderModule handle;
};

struct Descriptor_Writes {
    static constexpr uint32_t max_writes = 32;

    struct Accel_Info {
        VkWriteDescriptorSetAccelerationStructureKHR accel;
        VkAccelerationStructureKHR handle; // referenced by accel
    };

    struct Accel_Info_NV {
        VkWriteDescriptorSetAccelerationStructureNV accel;
        VkAccelerationStructureNV handle; // referenced by accel
    };

    union Resource_Info {
        VkDescriptorImageInfo image;
        VkDescriptorBufferInfo buffer;
        Accel_Info_NV accel_info_nv;
        Accel_Info accel_info;
    };

    VkDescriptorSet descriptor_set;
    VkWriteDescriptorSet descriptor_writes[max_writes];
    Resource_Info resource_infos[max_writes];
    uint32_t write_count;

    Descriptor_Writes(VkDescriptorSet set) {
        descriptor_set = set;
        write_count = 0;
    }
    ~Descriptor_Writes() {
        commit();
    }

    Descriptor_Writes& sampled_image(uint32_t binding, VkImageView image_view, VkImageLayout layout);
    Descriptor_Writes& sampled_image_array (uint32_t binding, uint32_t array_size, const VkDescriptorImageInfo* image_infos);
    Descriptor_Writes& storage_image(uint32_t binding, VkImageView image_view);
    Descriptor_Writes& sampler(uint32_t binding, VkSampler sampler);
    Descriptor_Writes& uniform_buffer(uint32_t binding, VkBuffer buffer, VkDeviceSize offset, VkDeviceSize range);
    Descriptor_Writes& storage_buffer(uint32_t binding, VkBuffer buffer, VkDeviceSize offset, VkDeviceSize range);
    Descriptor_Writes& storage_buffer_array(uint32_t binding, uint32_t array_size, const VkDescriptorBufferInfo* buffer_infos);
    Descriptor_Writes& accelerator(uint32_t binding, VkAccelerationStructureNV acceleration_structure);
    Descriptor_Writes& accelerator(uint32_t binding, VkAccelerationStructureKHR acceleration_structure);
    void commit();
};

struct Descriptor_Set_Layout {
    static constexpr uint32_t max_bindings = 32;

    VkDescriptorSetLayoutBinding bindings[max_bindings];
    uint32_t binding_count;

    Descriptor_Set_Layout() {
        binding_count = 0;
    }

    Descriptor_Set_Layout& sampled_image(uint32_t binding, VkShaderStageFlags stage_flags);
    Descriptor_Set_Layout& sampled_image_array(uint32_t binding, uint32_t array_size, VkShaderStageFlags stage_flags);
    Descriptor_Set_Layout& storage_image(uint32_t binding, VkShaderStageFlags stage_flags);
    Descriptor_Set_Layout& sampler(uint32_t binding, VkShaderStageFlags stage_flags);
    Descriptor_Set_Layout& uniform_buffer(uint32_t binding, VkShaderStageFlags stage_flags);
    Descriptor_Set_Layout& storage_buffer(uint32_t binding, VkShaderStageFlags stage_flags);
    Descriptor_Set_Layout& storage_buffer_array(uint32_t binding, uint32_t array_size, VkShaderStageFlags stage_flags);
    Descriptor_Set_Layout& accelerator(uint32_t binding, VkShaderStageFlags stage_flags);
    Descriptor_Set_Layout& accelerator_nv(uint32_t binding, VkShaderStageFlags stage_flags);
    VkDescriptorSetLayout create(const char* name);
};

//
// GPU time queries.
//
struct GPU_Time_Keeper;

struct GPU_Time_Scope {
    std::string name;
    GPU_Time_Keeper* parent;
    uint32_t start_query[2]; // end query == (start_query[frame_index] + 1)
    float length_ms;
    std::vector<GPU_Time_Scope*> child_scopes;

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

    GPU_Time_Scope* allocate_time_scope(const std::string& name);
    void initialize_time_scopes();
    void retrieve_query_results();
};

//
// GPU debug markers.
//
void begin_gpu_marker_scope(VkCommandBuffer command_buffer, const char* name);
void end_gpu_marker_scope(VkCommandBuffer command_buffer);
void write_gpu_marker(VkCommandBuffer command_buffer, const char* name);

struct GPU_Marker_Scope {
    GPU_Marker_Scope(VkCommandBuffer command_buffer, const char* name) {
        this->command_buffer = command_buffer;
        begin_gpu_marker_scope(command_buffer, name);
    }
    ~GPU_Marker_Scope() {
        end_gpu_marker_scope(command_buffer);
    }

private:
    VkCommandBuffer command_buffer;
};

#define GPU_MARKER_SCOPE(command_buffer, name) GPU_Marker_Scope gpu_marker_scope##__LINE__(command_buffer, name)

